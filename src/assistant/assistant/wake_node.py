"""ROS2 node for detecting wake phrases and managing conversation flow."""

import re

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .audio_output import AudioOutput
from .wake_detector import WakeDetector

# Audio settings for chimes
CHIME_SAMPLE_RATE = 22050
CHIME_FREQUENCY = 880  # A5 note for ready chime
CHIME_DURATION = 0.15  # seconds
WAKE_FREQUENCIES = (660, 880)  # E5 -> A5 ascending for wake word
WAKE_NOTE_DURATION = 0.08  # seconds per note (snappier)
LOADING_FREQUENCIES = (523, 659, 784)  # C5 -> E5 -> G5 (major arpeggio)
LOADING_NOTE_DURATION = 0.08  # seconds per note
LOADING_INTERVAL = 1.2  # seconds between repeats


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

        # Initialize audio output for chimes
        self._audio = AudioOutput(sample_rate=CHIME_SAMPLE_RATE)
        self._audio.start()
        self._chime = self._generate_chime()
        self._wake_chime = self._generate_wake_chime()
        self._loading_chime = self._generate_loading_chime()
        self._loading_timer = None

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

    def _generate_chime(self) -> np.ndarray:
        """Generate a short ready-for-input chime sound."""
        t = np.linspace(0, CHIME_DURATION, int(CHIME_SAMPLE_RATE * CHIME_DURATION))
        # Sine wave with exponential decay envelope
        envelope = np.exp(-t * 15)
        wave = np.sin(2 * np.pi * CHIME_FREQUENCY * t) * envelope
        # Add a subtle harmonic for richness
        wave += 0.3 * np.sin(2 * np.pi * CHIME_FREQUENCY * 2 * t) * envelope
        # Normalize and convert to int16
        wave = wave / np.max(np.abs(wave)) * 0.7
        return (wave * 32767).astype(np.int16)

    def _generate_wake_chime(self) -> np.ndarray:
        """Generate an ascending two-tone chime for wake word."""
        samples_per_note = int(CHIME_SAMPLE_RATE * WAKE_NOTE_DURATION)
        waves = []
        for freq in WAKE_FREQUENCIES:
            t = np.linspace(0, WAKE_NOTE_DURATION, samples_per_note)
            envelope = np.exp(-t * 18)  # Faster decay for snappier sound
            wave = np.sin(2 * np.pi * freq * t) * envelope
            wave += 0.3 * np.sin(2 * np.pi * freq * 2 * t) * envelope
            waves.append(wave)
        combined = np.concatenate(waves)
        combined = combined / np.max(np.abs(combined)) * 0.7
        return (combined * 32767).astype(np.int16)

    def _generate_loading_chime(self) -> np.ndarray:
        """Generate an ascending 3-note arpeggio for loading indicator."""
        samples_per_note = int(CHIME_SAMPLE_RATE * LOADING_NOTE_DURATION)
        waves = []
        for freq in LOADING_FREQUENCIES:
            t = np.linspace(0, LOADING_NOTE_DURATION, samples_per_note)
            envelope = np.exp(-t * 20)  # Quick decay
            wave = np.sin(2 * np.pi * freq * t) * envelope
            wave += 0.3 * np.sin(2 * np.pi * freq * 2 * t) * envelope
            waves.append(wave)
        combined = np.concatenate(waves)
        combined = combined / np.max(np.abs(combined)) * 0.3  # Softer volume
        return (combined * 32767).astype(np.int16)

    def _play_chime(self) -> None:
        """Play the ready-for-input chime."""
        self._audio.play(self._chime)

    def _play_wake_chime(self) -> None:
        """Play the wake word chime."""
        self._audio.play(self._wake_chime)

    def _start_loading_chime(self) -> None:
        """Start the repeating loading chime after initial delay."""
        self._loading_timer = self.create_timer(
            LOADING_INTERVAL, self._play_loading_tick
        )

    def _stop_loading_chime(self) -> None:
        """Stop the loading chime."""
        if self._loading_timer is not None:
            self._loading_timer.cancel()
            self._loading_timer = None

    def _play_loading_tick(self) -> None:
        """Timer callback to play loading chime."""
        self._audio.play(self._loading_chime)

    def _state_callback(self, msg: Bool) -> None:
        """Handle conversation state updates."""
        self._conversation_active = msg.data

    def _response_callback(self, msg: String) -> None:
        """Handle assistant response - clears waiting state and stops loading chime."""
        self._stop_loading_chime()
        self._waiting_for_response = False

    def _tts_speaking_callback(self, msg: Bool) -> None:
        """Play chime when TTS finishes during active conversation."""
        is_speaking = msg.data

        # Detect falling edge: TTS just stopped speaking
        if self._tts_was_speaking and not is_speaking:
            # Only play if conversation still active (not after "Goodbye!")
            if self._conversation_active:
                # Slight delay for natural pause before chime
                self._ready_chime_timer = self.create_timer(
                    0.15, self._play_ready_chime
                )

        self._tts_was_speaking = is_speaking

    def _play_ready_chime(self) -> None:
        """Play ready-to-listen chime (one-shot from timer)."""
        # Cancel timer to prevent repeat
        if hasattr(self, "_ready_chime_timer"):
            self._ready_chime_timer.cancel()
        # Re-check conversation state in case it changed during delay
        if self._conversation_active:
            self._play_chime()

    def _is_goodbye(self, text: str) -> bool:
        """Check if text contains a goodbye phrase."""
        text = text.strip().lower()
        return any(p.search(text) for p in self._goodbye_patterns)

    def _publish_request(self, text: str) -> None:
        """Publish a user request."""
        self._waiting_for_response = True
        self._start_loading_chime()
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
                self._play_wake_chime()
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
        node._audio.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
