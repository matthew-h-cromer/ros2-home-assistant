"""ROS2 node for detecting wake phrases and managing conversation flow."""

import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .wake_detector import WakeDetector


# Patterns for detecting goodbye phrases (searched anywhere in text)
GOODBYE_PATTERNS = [
    r"\bgoodbye\b",
    r"\bbye\b",
    r"\bthanks\b",
    r"\bthank[\s,-]+you\b",
    r"\bthat'?s[\s,-]+all\b",
    r"\bnever[\s,-]+mind\b",
    r"\bcancel\b",
    # Dismissal phrases
    r"\bshut[\s,-]+up\b",
    r"\bshush\b",
    r"\bhush\b",
    r"\bgo[\s,-]+away\b",
    r"\bstop[\s,-]+talking\b",
]


class WakeNode(Node):
    """ROS2 node that manages conversation flow with wake word detection."""

    def __init__(self):
        super().__init__("wake_detection")

        # Declare parameters
        self.declare_parameter("input_topic", "speech")
        self.declare_parameter("output_topic", "user_requests")
        self.declare_parameter("wake_word", "ross")

        # Get parameter values
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        wake_word = self.get_parameter("wake_word").value

        # Initialize wake detector
        self._detector = WakeDetector(wake_word=wake_word)

        # Compile goodbye patterns
        self._goodbye_patterns = [
            re.compile(pattern, re.IGNORECASE) for pattern in GOODBYE_PATTERNS
        ]

        # Conversation state (updated via subscription)
        self._conversation_active = False
        self._waiting_for_response = False
        self._tts_was_speaking = False  # Track TTS state for edge detection

        # Create subscribers
        self._speech_subscription = self.create_subscription(
            String, input_topic, self._speech_callback, 10
        )
        self._state_subscription = self.create_subscription(
            Bool, "conversation_active", self._state_callback, 10
        )
        self._response_subscription = self.create_subscription(
            String, "assistant_response", self._response_callback, 10
        )
        self._tts_subscription = self.create_subscription(
            Bool, "tts_speaking", self._tts_speaking_callback, 10
        )

        # Create publisher
        self._publisher = self.create_publisher(String, output_topic, 10)

        self.get_logger().info(
            f'Wake detection started (wake word: "{wake_word}"). '
            f'Listening on "{input_topic}", publishing to "{output_topic}"'
        )

    def _state_callback(self, msg: Bool) -> None:
        """Handle conversation state updates."""
        self._conversation_active = msg.data

    def _response_callback(self, msg: String) -> None:
        """Handle assistant response - clears waiting state."""
        self._waiting_for_response = False

    def _tts_speaking_callback(self, msg: Bool) -> None:
        """Track TTS speaking state."""
        self._tts_was_speaking = msg.data

    def _is_goodbye(self, text: str) -> bool:
        """Check if text contains a goodbye phrase."""
        text = text.strip().lower()
        return any(p.search(text) for p in self._goodbye_patterns)

    def _publish_request(self, text: str) -> None:
        """Publish a user request."""
        self._waiting_for_response = True
        out_msg = String()
        out_msg.data = text
        self._publisher.publish(out_msg)

    def _speech_callback(self, msg: String) -> None:
        """Handle incoming speech messages."""
        text = msg.data.strip()

        if not text:
            return

        # Ignore input while waiting for LLM response
        if self._waiting_for_response:
            self.get_logger().debug(f'Ignoring (waiting for response): "{text}"')
            return

        if self._conversation_active:
            # In active conversation - check for goodbye or pass through
            if self._is_goodbye(text):
                self.get_logger().info(f'Goodbye detected: "{text}"')
                self._publish_request("[END]")
            else:
                # Pass through as follow-up (no wake word needed)
                self.get_logger().info(f'Follow-up: "{text}"')
                self._publish_request(text)
        else:
            # Not in conversation - require wake word
            result = self._detector.detect(text)

            if result.triggered:
                if result.command:
                    self.get_logger().info(
                        f'Triggered: "{result.wake_phrase}" -> "{result.command}"'
                    )
                    self._publish_request(result.command)
                else:
                    # Wake word only - prompt user for command
                    self.get_logger().info(
                        f'Wake word only: "{result.wake_phrase}"'
                    )
                    self._publish_request("The user said my name. Respond briefly to acknowledge and ask how you can help.")


def main(args=None):
    rclpy.init(args=args)

    node = WakeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
