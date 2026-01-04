"""ROS2 node for LLM-powered responses using Claude API with conversation support."""

import os
import time
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import anthropic
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


def _load_env_file():
    """Load .env file from common locations."""
    search_paths = [
        Path.cwd() / ".env",
        Path.home() / "Documents/ros2-home-assistant/.env",
    ]

    for env_path in search_paths:
        if env_path.exists():
            for line in env_path.read_text().splitlines():
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    key, value = line.split("=", 1)
                    os.environ.setdefault(key.strip(), value.strip())
            return


_load_env_file()


SYSTEM_PROMPT_TEMPLATE = """You are Ross, a voice assistant.

Location: {location} | Timezone: {timezone}

Your responses are READ ALOUD. Long responses are confusing and waste the user's time.

- 1-2 short sentences max.
- Use the absolute minimum amount of words that effectively answer the question.
- Answer ONLY what was asked, nothing extra.
- Optimize your answer to be easily understood when read aloud.
- Skip context the user already knows"""


CUSTOM_TOOLS = [
    {
        "name": "get_time",
        "description": "Get the current date and time. Use for questions about what time or day it is.",
        "input_schema": {
            "type": "object",
            "properties": {
                "timezone": {
                    "type": "string",
                    "description": "Timezone (e.g., 'America/Los_Angeles', 'America/New_York'). Defaults to local time.",
                }
            },
            "required": [],
        },
    },
]


class LLMNode(Node):
    """ROS2 node that processes user requests through Claude API with conversation support."""

    def __init__(self):
        super().__init__("llm")

        # Declare parameters
        self.declare_parameter("input_topic", "user_requests")
        self.declare_parameter("output_topic", "assistant_response")
        self.declare_parameter("model", "claude-sonnet-4-20250514")
        self.declare_parameter("max_tokens", 4096)
        self.declare_parameter("web_search", True)
        self.declare_parameter("location", "Seattle, WA")
        self.declare_parameter("timezone", "America/Los_Angeles")
        self.declare_parameter("conversation_timeout", 30.0)
        self.declare_parameter("max_history_turns", 10)

        # Get parameter values
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._model = self.get_parameter("model").value
        self._max_tokens = self.get_parameter("max_tokens").value
        self._web_search = self.get_parameter("web_search").value
        self._location = self.get_parameter("location").value
        self._timezone = self.get_parameter("timezone").value
        self._conversation_timeout = self.get_parameter("conversation_timeout").value
        self._max_history_turns = self.get_parameter("max_history_turns").value

        # Build system prompt with context
        self._system_prompt = SYSTEM_PROMPT_TEMPLATE.format(
            location=self._location,
            timezone=self._timezone,
        )

        # Initialize Anthropic client
        api_key = os.environ.get("ANTHROPIC_API_KEY")
        if not api_key:
            self.get_logger().error(
                "ANTHROPIC_API_KEY not set. Create .env file in repo root."
            )
            raise RuntimeError("ANTHROPIC_API_KEY not set")

        self._client = anthropic.Anthropic(api_key=api_key)

        # Conversation state
        self._conversation_history = []
        self._conversation_active = False
        self._last_activity_time = 0.0
        self._is_processing = False
        self._tts_speaking = False

        # Create subscriber and publishers
        self._subscription = self.create_subscription(
            String, input_topic, self._request_callback, 10
        )
        self._tts_subscription = self.create_subscription(
            Bool, "tts_speaking", self._tts_state_callback, 10
        )
        self._response_publisher = self.create_publisher(String, output_topic, 10)
        self._state_publisher = self.create_publisher(Bool, "conversation_active", 10)

        # Create timer for timeout checking (runs every second)
        self._timeout_timer = self.create_timer(1.0, self._check_timeout)

        search_status = "enabled" if self._web_search else "disabled"
        self.get_logger().info(
            f"LLM node started with conversation support. "
            f'Listening on "{input_topic}", publishing to "{output_topic}" '
            f"(web search: {search_status}, timeout: {self._conversation_timeout}s)"
        )

    def _publish_state(self, active: bool):
        """Publish conversation state."""
        msg = Bool()
        msg.data = active
        self._state_publisher.publish(msg)

    def _tts_state_callback(self, msg: Bool):
        """Handle TTS speaking state updates."""
        was_speaking = self._tts_speaking
        self._tts_speaking = msg.data

        # Reset timeout when TTS finishes speaking
        if was_speaking and not self._tts_speaking:
            self._last_activity_time = time.time()

    def _check_timeout(self):
        """Check if conversation has timed out."""
        if not self._conversation_active or self._is_processing or self._tts_speaking:
            return

        elapsed = time.time() - self._last_activity_time
        if elapsed >= self._conversation_timeout:
            self.get_logger().info("Conversation timed out")
            self._end_conversation(publish_goodbye=True)

    def _end_conversation(self, publish_goodbye: bool = True):
        """End the current conversation and clear history."""
        self._conversation_active = False
        self._conversation_history = []
        self._publish_state(False)

        if publish_goodbye:
            out_msg = String()
            out_msg.data = "Goodbye!"
            self._response_publisher.publish(out_msg)
            self.get_logger().info("Conversation ended: Goodbye!")

    def _execute_tool(self, name: str, input_data: dict) -> str:
        """Execute a tool and return the result."""
        if name == "get_time":
            tz_name = input_data.get("timezone", self._timezone)
            try:
                tz = ZoneInfo(tz_name)
            except Exception:
                tz = ZoneInfo(self._timezone)
            now = datetime.now(tz)
            return now.strftime("%A, %B %d, %Y at %I:%M %p %Z")

        return f"Unknown tool: {name}"

    def _request_callback(self, msg: String) -> None:
        """Handle incoming user requests."""
        user_request = msg.data.strip()

        # Check for end conversation marker
        if user_request.startswith("[END]"):
            self._end_conversation(publish_goodbye=True)
            return

        if not user_request:
            self.get_logger().warning("Received empty request, ignoring")
            return

        # Ignore requests while already processing
        if self._is_processing:
            self.get_logger().info(f'Ignoring request (busy): "{user_request}"')
            return

        self._is_processing = True
        self.get_logger().info(f'Processing request: "{user_request}"')

        # Mark conversation as active and set activity time
        # (set time now so timeout doesn't trigger immediately on error)
        self._conversation_active = True
        self._last_activity_time = time.time()
        self._publish_state(True)

        try:
            # Build tools list
            tools = []
            if self._web_search:
                tools.append({"type": "web_search_20250305", "name": "web_search"})
            tools.extend(CUSTOM_TOOLS)

            # Add user message to history
            self._conversation_history.append({"role": "user", "content": user_request})

            # Trim history if too long (keep last N turns = 2N messages)
            max_messages = self._max_history_turns * 2
            if len(self._conversation_history) > max_messages:
                self._conversation_history = self._conversation_history[-max_messages:]

            # Create working copy of messages for this request's tool loop
            messages = list(self._conversation_history)

            # Agentic loop to handle tool calls
            while True:
                response = self._client.messages.create(
                    model=self._model,
                    max_tokens=self._max_tokens,
                    system=[{
                        "type": "text",
                        "text": self._system_prompt,
                        "cache_control": {"type": "ephemeral"},
                    }],
                    messages=messages,
                    tools=tools,
                )

                # Log tool usage and web search results
                for block in response.content:
                    if block.type == "tool_use":
                        self.get_logger().info(f"Tool called: {block.name}")
                    elif block.type == "web_search_tool_result":
                        # content is a list of search result objects
                        results = getattr(block, "content", [])
                        self.get_logger().info(
                            f"Web search returned {len(results)} sources:"
                        )
                        for result in results[:5]:
                            title = getattr(result, "title", "Unknown")
                            url = getattr(result, "url", "")
                            self.get_logger().info(f"  - {title}: {url}")

                # Check if we need to execute tools
                if response.stop_reason == "tool_use":
                    # Add assistant's response to messages
                    messages.append({"role": "assistant", "content": response.content})

                    # Execute each tool and collect results
                    tool_results = []
                    for block in response.content:
                        if block.type == "tool_use":
                            result = self._execute_tool(block.name, block.input)
                            tool_results.append({
                                "type": "tool_result",
                                "tool_use_id": block.id,
                                "content": result,
                            })

                    messages.append({"role": "user", "content": tool_results})
                else:
                    # No more tool calls, extract final text
                    break

            # Extract text from final response
            text_parts = []
            for block in response.content:
                if block.type == "text":
                    text_parts.append(block.text)
            response_text = " ".join(text_parts).strip()

            if not response_text:
                self.get_logger().warning("No text in response")
                return

            # Add assistant response to conversation history
            self._conversation_history.append(
                {"role": "assistant", "content": response_text}
            )

            # Publish response
            out_msg = String()
            out_msg.data = response_text
            self._response_publisher.publish(out_msg)

            self.get_logger().info(f'Response: "{response_text}"')

            # Reset activity time for timeout
            self._last_activity_time = time.time()

        except anthropic.APIError as e:
            self.get_logger().error(f"Claude API error: {e}")
            # Notify user of error
            out_msg = String()
            out_msg.data = "Sorry, I encountered an error. Please try again."
            self._response_publisher.publish(out_msg)
        finally:
            self._is_processing = False


def main(args=None):
    rclpy.init(args=args)

    node = LLMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
