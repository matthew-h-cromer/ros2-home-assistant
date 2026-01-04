"""ROS2 node for detecting trigger phrases and managing conversation flow."""

import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .trigger_detector import TriggerDetector


# Patterns for detecting goodbye phrases
GOODBYE_PATTERNS = [
    r"^goodbye\.?$",
    r"^bye\.?$",
    r"^bye[\s,-]+ross\.?$",
    r"^goodbye[\s,-]+ross\.?$",
    r"^thanks[\s,-]+ross\.?$",
    r"^thank[\s,-]+you[\s,-]+ross\.?$",
    r"^that'?s[\s,-]+all\.?$",
    r"^never[\s,-]+mind\.?$",
    r"^cancel\.?$",
]


class TriggerNode(Node):
    """ROS2 node that manages conversation flow with wake word detection."""

    def __init__(self):
        super().__init__("trigger_detection")

        # Declare parameters
        self.declare_parameter("input_topic", "speech")
        self.declare_parameter("output_topic", "user_requests")
        self.declare_parameter("trigger_phrases", ["hey ross"])

        # Get parameter values
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        trigger_phrases = self.get_parameter("trigger_phrases").value

        # Initialize trigger detector
        self._detector = TriggerDetector(trigger_phrases=trigger_phrases)

        # Compile goodbye patterns
        self._goodbye_patterns = [
            re.compile(pattern, re.IGNORECASE) for pattern in GOODBYE_PATTERNS
        ]

        # Conversation state (updated via subscription)
        self._conversation_active = False

        # Create subscribers
        self._speech_subscription = self.create_subscription(
            String, input_topic, self._speech_callback, 10
        )
        self._state_subscription = self.create_subscription(
            Bool, "conversation_active", self._state_callback, 10
        )

        # Create publisher
        self._publisher = self.create_publisher(String, output_topic, 10)

        self.get_logger().info(
            f'Trigger detection started. Listening on "{input_topic}", '
            f'publishing to "{output_topic}"'
        )

    def _state_callback(self, msg: Bool) -> None:
        """Handle conversation state updates."""
        self._conversation_active = msg.data

    def _is_goodbye(self, text: str) -> bool:
        """Check if text is a goodbye phrase."""
        text = text.strip().lower()
        for pattern in self._goodbye_patterns:
            if pattern.match(text):
                return True
        return False

    def _publish_request(self, text: str) -> None:
        """Publish a user request."""
        out_msg = String()
        out_msg.data = text
        self._publisher.publish(out_msg)

    def _speech_callback(self, msg: String) -> None:
        """Handle incoming speech messages."""
        text = msg.data.strip()

        if not text:
            return

        if self._conversation_active:
            # In active conversation - check for goodbye or pass through
            if self._is_goodbye(text):
                self.get_logger().info(f'Goodbye detected: "{text}"')
                self._publish_request(f"[END] {text}")
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
                        f'Triggered: "{result.trigger_phrase}" -> "{result.command}"'
                    )
                    self._publish_request(result.command)
                else:
                    # Wake word only - prompt user for command
                    self.get_logger().info(
                        f'Wake word only: "{result.trigger_phrase}"'
                    )
                    self._publish_request("The user said my name. Respond briefly to acknowledge and ask how you can help.")


def main(args=None):
    rclpy.init(args=args)

    node = TriggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
