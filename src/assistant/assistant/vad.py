"""Voice Activity Detection using Silero VAD."""

import logging
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
from silero_vad import load_silero_vad, VADIterator

# Silero VAD requires specific chunk sizes
# For 16kHz: 512 samples = 32ms per chunk
VAD_CHUNK_SAMPLES = 512

# Maximum audio buffer size (30 seconds at 16kHz, 16-bit = 960KB)
# Prevents unbounded memory growth if processing falls behind
MAX_BUFFER_BYTES = 16000 * 2 * 30


@dataclass
class SpeechSegment:
    """A detected speech segment ready for transcription."""

    audio: np.ndarray  # float32 numpy array, normalized -1.0 to 1.0
    sample_rate: int = 16000


class VoiceActivityDetector:
    """Detects speech boundaries in streaming audio using Silero VAD.

    Buffers audio, processes it through VAD in fixed-size chunks,
    and emits complete speech segments when silence is detected.
    """

    def __init__(
        self,
        sample_rate: int = 16000,
        threshold: float = 0.5,
        min_silence_duration_ms: int = 500,
        logger: Optional[logging.Logger] = None,
    ):
        """Initialize the VAD.

        Args:
            sample_rate: Audio sample rate (must be 16000 for Silero)
            threshold: VAD confidence threshold (0.0-1.0)
            min_silence_duration_ms: Silence duration to end utterance
            logger: Logger instance for status messages
        """
        if sample_rate != 16000:
            raise ValueError("Silero VAD requires 16kHz sample rate")

        self.sample_rate = sample_rate
        self._threshold = threshold
        self._min_silence_ms = min_silence_duration_ms
        self._logger = logger

        # Load Silero VAD
        self._log("Loading Silero VAD model...")
        vad_model = load_silero_vad(onnx=True)
        self._vad = VADIterator(
            vad_model,
            sampling_rate=sample_rate,
            threshold=threshold,
            min_silence_duration_ms=min_silence_duration_ms,
        )
        self._log("Silero VAD ready.")

        # Audio buffering
        self._audio_buffer = bytearray()
        self._speech_buffer: List[np.ndarray] = []
        self._is_speaking = False

    def _log(self, message: str) -> None:
        """Log a message if logger is available."""
        if self._logger:
            self._logger.info(message)

    def process(self, audio_data: bytes) -> Optional[SpeechSegment]:
        """Process audio bytes and return speech segment when complete.

        Args:
            audio_data: Raw audio bytes (16-bit PCM, 16kHz)

        Returns:
            SpeechSegment when utterance complete, None if still listening
        """
        # Check for buffer overflow before adding new data
        if len(self._audio_buffer) + len(audio_data) > MAX_BUFFER_BYTES:
            self._log("Audio buffer overflow, resetting VAD state")
            self.reset()

        self._audio_buffer.extend(audio_data)

        # Process in VAD-sized chunks
        chunk_bytes = VAD_CHUNK_SAMPLES * 2  # 16-bit = 2 bytes per sample

        while len(self._audio_buffer) >= chunk_bytes:
            # Extract chunk
            chunk_data = bytes(self._audio_buffer[:chunk_bytes])
            del self._audio_buffer[:chunk_bytes]

            # Convert to float32 for VAD
            chunk_int16 = np.frombuffer(chunk_data, dtype=np.int16)
            chunk_float = chunk_int16.astype(np.float32) / 32768.0

            # Run VAD with error handling
            try:
                speech_dict = self._vad(chunk_float, return_seconds=False)
            except Exception as e:
                self._log(f"VAD error: {e}, resetting state")
                self.reset()
                return None

            if speech_dict:
                if "start" in speech_dict:
                    # Speech started
                    self._is_speaking = True
                    self._speech_buffer.append(chunk_float)

                elif "end" in speech_dict:
                    # Speech ended - return segment
                    self._speech_buffer.append(chunk_float)
                    segment = self._build_segment()
                    self.reset()
                    return segment

            elif self._is_speaking:
                # Continuing speech
                self._speech_buffer.append(chunk_float)

        return None

    def reset(self) -> None:
        """Reset VAD state for a new utterance."""
        self._is_speaking = False
        self._speech_buffer.clear()
        self._audio_buffer.clear()
        self._vad.reset_states()

    def _build_segment(self) -> Optional[SpeechSegment]:
        """Build a SpeechSegment from the accumulated buffer."""
        if not self._speech_buffer:
            return None

        audio = np.concatenate(self._speech_buffer)
        return SpeechSegment(audio=audio, sample_rate=self.sample_rate)
