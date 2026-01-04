"""ROS2 node for text-to-speech using Piper TTS."""

import queue
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .audio_output import AudioOutput
from .constants import DEFAULT_PIPER_VOICE
from .piper_synthesizer import PiperSynthesizer


class TTSNode(Node):
    """ROS2 node that synthesizes and plays speech from text."""

    def __init__(self):
        super().__init__("tts")

        # Declare parameters
        self.declare_parameter("input_topic", "assistant_response")
        self.declare_parameter("voice", DEFAULT_PIPER_VOICE)
        self.declare_parameter("device_index", -1)
        self.declare_parameter("length_scale", 1.0)

        # Get parameter values
        input_topic = self.get_parameter("input_topic").value
        voice = self.get_parameter("voice").value
        device_index = self.get_parameter("device_index").value
        length_scale = self.get_parameter("length_scale").value

        # Thread-safe queue for text to synthesize
        self._text_queue: queue.Queue[str] = queue.Queue()

        # Thread-safe flags
        self._running = threading.Event()
        self._running.set()
        self._is_speaking = threading.Event()

        # Initialize Piper synthesizer
        self.synthesizer = PiperSynthesizer(
            voice=voice,
            length_scale=length_scale,
            logger=self.get_logger(),
        )

        # Initialize audio output
        device: Optional[int] = None if device_index < 0 else device_index
        self.audio_output = AudioOutput(
            sample_rate=self.synthesizer.sample_rate,
            device_index=device,
            logger=self.get_logger(),
        )
        device_info = self.audio_output.get_device_info()
        self.get_logger().info(
            f"Using speaker: {device_info['name']} "
            f"({device_info['hostapi']}, {device_info['channels']}ch)"
        )

        # Start synthesis thread
        self._synthesis_thread = threading.Thread(
            target=self._synthesis_loop,
            daemon=True,
        )
        self._synthesis_thread.start()

        # Create subscriber
        self._subscription = self.create_subscription(
            String,
            input_topic,
            self._text_callback,
            10,
        )

        # Publisher for speaking state (for barge-in support)
        self._speaking_publisher = self.create_publisher(Bool, "tts_speaking", 10)

        # Timer to publish speaking state (100ms = 10Hz)
        self._state_timer = self.create_timer(0.1, self._publish_state)

        self.get_logger().info(
            f'TTS node started. Listening on "{input_topic}", voice: {voice}'
        )

    def _text_callback(self, msg: String) -> None:
        """Handle incoming text messages."""
        text = msg.data.strip()
        if text:
            self.get_logger().info(f'Speaking: "{text}"')
            # Set speaking flag immediately to block speech recognition
            self._is_speaking.set()
            self._text_queue.put(text)

    def _synthesis_loop(self) -> None:
        """Background thread for speech synthesis."""
        while self._running.is_set():
            try:
                text = self._text_queue.get(timeout=0.1)

                # Synthesize and play (blocking)
                # Note: _is_speaking is already set by _text_callback
                audio = self.synthesizer.synthesize(text)
                if audio.size > 0:
                    self.audio_output.play_blocking(audio)
                self._is_speaking.clear()

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Synthesis error: {e}")
                self._is_speaking.clear()

    def _publish_state(self) -> None:
        """Publish speaking state."""
        msg = Bool()
        msg.data = self._is_speaking.is_set()
        self._speaking_publisher.publish(msg)

    def destroy_node(self) -> None:
        """Clean shutdown."""
        self._running.clear()
        if hasattr(self, "_synthesis_thread"):
            self._synthesis_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = TTSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
