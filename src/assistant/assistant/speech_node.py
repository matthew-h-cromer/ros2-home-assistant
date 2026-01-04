"""ROS2 node for real-time speech recognition using Whisper."""

import queue
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .audio_stream import AudioStream
from .constants import DEFAULT_WHISPER_MODEL
from .vad import SpeechSegment, VoiceActivityDetector
from .whisper_recognizer import WhisperTranscriber


class SpeechRecognitionNode(Node):
    """ROS2 node that captures audio and publishes recognized speech."""

    def __init__(self):
        super().__init__("speech_recognition")

        # Declare parameters
        self.declare_parameter("model", DEFAULT_WHISPER_MODEL)
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("device_index", -1)
        self.declare_parameter("topic_name", "speech")
        self.declare_parameter("vad_threshold", 0.5)
        self.declare_parameter("vad_silence_ms", 500)

        # Get parameter values
        model = self.get_parameter("model").value
        sample_rate = self.get_parameter("sample_rate").value
        device_index = self.get_parameter("device_index").value
        topic_name = self.get_parameter("topic_name").value
        vad_threshold = self.get_parameter("vad_threshold").value
        vad_silence_ms = self.get_parameter("vad_silence_ms").value

        # Validate sample rate (Silero VAD requires exactly 16000 Hz)
        if sample_rate != 16000:
            self.get_logger().warning(
                f"Sample rate {sample_rate} Hz not supported by VAD, forcing to 16000 Hz"
            )
            sample_rate = 16000

        # Create publisher
        self.publisher_ = self.create_publisher(String, topic_name, 10)

        # Thread-safe queues
        self.segment_queue: queue.Queue[SpeechSegment] = queue.Queue()
        self.result_queue: queue.Queue[str] = queue.Queue()

        # Thread-safe running flag
        self._running = threading.Event()
        self._running.set()

        # Initialize VAD
        self.vad = VoiceActivityDetector(
            sample_rate=sample_rate,
            threshold=vad_threshold,
            min_silence_duration_ms=vad_silence_ms,
            logger=self.get_logger(),
        )

        # Initialize Whisper transcriber
        self.transcriber = WhisperTranscriber(
            model_name=model,
            logger=self.get_logger(),
        )

        # Initialize audio stream
        device: Optional[int] = None if device_index < 0 else device_index
        self.audio_stream = AudioStream(
            sample_rate=sample_rate,
            device_index=device,
            callback=self._audio_callback,
            logger=self.get_logger(),
        )
        device_info = self.audio_stream.get_device_info()
        self.get_logger().info(
            f"Using microphone: {device_info['name']} "
            f"({device_info['hostapi']}, {device_info['channels']}ch, "
            f"{int(device_info['sample_rate'])}Hz)"
        )

        # Start audio and transcription threads
        self.audio_thread = threading.Thread(target=self._audio_loop, daemon=True)
        self.audio_thread.start()

        self.transcription_thread = threading.Thread(
            target=self._transcription_loop, daemon=True
        )
        self.transcription_thread.start()

        # Timer to check for results (50ms = 20Hz)
        self.timer = self.create_timer(0.05, self._check_results)

        self.get_logger().info(f'Speech recognition started. Publishing to "{topic_name}"')

    def _audio_callback(self, audio_data: bytes) -> None:
        """Called by audio stream with new audio data."""
        if not self._running.is_set():
            return

        segment = self.vad.process(audio_data)
        if segment:
            self.segment_queue.put(segment)

    def _audio_loop(self) -> None:
        """Audio capture loop running in separate thread."""
        try:
            self.audio_stream.start()
            while self._running.is_set():
                time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"Audio loop error: {e}")
        finally:
            self.audio_stream.stop()

    def _transcription_loop(self) -> None:
        """Transcription loop running in separate thread."""
        while self._running.is_set():
            try:
                segment = self.segment_queue.get(timeout=0.1)
                text = self.transcriber.transcribe(segment.audio)
                if text.strip():
                    self.result_queue.put(text)
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Transcription error: {e}")

    def _check_results(self) -> None:
        """Timer callback to publish any recognized speech."""
        while not self.result_queue.empty():
            try:
                text = self.result_queue.get_nowait()
                msg = String()
                msg.data = text

                self.publisher_.publish(msg)
                self.get_logger().info(f'Recognized: "{text}"')

            except queue.Empty:
                break

    def destroy_node(self) -> None:
        """Clean shutdown."""
        self._running.clear()
        if hasattr(self, "audio_thread"):
            self.audio_thread.join(timeout=1.0)
        if hasattr(self, "transcription_thread"):
            self.transcription_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = SpeechRecognitionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
