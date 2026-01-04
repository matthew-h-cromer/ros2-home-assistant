"""ROS2 node for LLM-powered responses using Claude API."""

import os
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import anthropic
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _load_env_file():
    """Load .env file from common locations."""
    # Check multiple possible locations
    search_paths = [
        Path.cwd() / ".env",  # Current working directory
        Path.home() / "Documents/ros2-home-assistant/.env",  # Common dev location
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


SYSTEM_PROMPT_TEMPLATE = """You are Ross, a home assistant. Your goal is to provide maximally helpful and concise responses.

Context:
- Location: {location}
- Timezone: {timezone}

Guidelines:
- Be direct and professional. No unnecessary pleasantries.
- Give actionable answers. Skip preamble.
- You only get one response - make it count. This is not a conversation.
- If you cannot help, say so briefly.
- Do not patronize or over-explain.
- Keep responses under 2-3 sentences when possible.
- Use web search for current information (weather, news, sports, prices, etc.)."""


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
    """ROS2 node that processes user requests through Claude API."""

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

        # Get parameter values
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._model = self.get_parameter("model").value
        self._max_tokens = self.get_parameter("max_tokens").value
        self._web_search = self.get_parameter("web_search").value
        self._location = self.get_parameter("location").value
        self._timezone = self.get_parameter("timezone").value

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

        # Create subscriber and publisher
        self._subscription = self.create_subscription(
            String,
            input_topic,
            self._request_callback,
            10,
        )
        self._publisher = self.create_publisher(String, output_topic, 10)

        search_status = "enabled" if self._web_search else "disabled"
        self.get_logger().info(
            f'LLM node started. Listening on "{input_topic}", '
            f'publishing to "{output_topic}" (web search: {search_status}, '
            f'location: {self._location})'
        )

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

        if not user_request:
            self.get_logger().warning("Received empty request, ignoring")
            return

        self.get_logger().info(f'Processing request: "{user_request}"')

        try:
            # Build tools list
            tools = []
            if self._web_search:
                tools.append({"type": "web_search_20250305", "name": "web_search"})
            tools.extend(CUSTOM_TOOLS)

            messages = [{"role": "user", "content": user_request}]

            # Agentic loop to handle tool calls
            while True:
                response = self._client.messages.create(
                    model=self._model,
                    max_tokens=self._max_tokens,
                    system=self._system_prompt,
                    messages=messages,
                    tools=tools,
                )

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

            # Extract text from final response (concatenate all text blocks)
            text_parts = []
            for block in response.content:
                if block.type == "text":
                    text_parts.append(block.text)
            response_text = " ".join(text_parts).strip()

            if not response_text:
                self.get_logger().warning("No text in response")
                return

            out_msg = String()
            out_msg.data = response_text
            self._publisher.publish(out_msg)

            self.get_logger().info(f'Response: "{response_text}"')

        except anthropic.APIError as e:
            self.get_logger().error(f"Claude API error: {e}")


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
