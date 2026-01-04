"""ROS2 node for detecting trigger phrases in speech."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .trigger_detector import TriggerDetector


class TriggerNode(Node):
    """ROS2 node that detects trigger phrases and publishes user requests."""

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

        # Create subscriber and publisher
        self._subscription = self.create_subscription(
            String,
            input_topic,
            self._speech_callback,
            10,
        )
        self._publisher = self.create_publisher(String, output_topic, 10)

        self.get_logger().info(
            f'Trigger detection started. Listening on "{input_topic}", '
            f'publishing to "{output_topic}"'
        )

    def _speech_callback(self, msg: String) -> None:
        """Handle incoming speech messages."""
        result = self._detector.detect(msg.data)

        if result.triggered:
            out_msg = String()
            out_msg.data = result.command
            self._publisher.publish(out_msg)
            self.get_logger().info(
                f'Triggered: "{result.trigger_phrase}" -> "{result.command}"'
            )


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
