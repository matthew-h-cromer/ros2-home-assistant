"""Whisper speech transcription using faster-whisper."""

import logging
import os
from typing import Optional

import numpy as np

from .constants import MODELS_DIR

# Set HuggingFace cache to repo-local directory before importing models
os.environ.setdefault("HF_HOME", os.path.join(MODELS_DIR, "huggingface"))

from faster_whisper import WhisperModel

# Known model sizes (approximate download sizes)
WHISPER_MODEL_SIZES = {
    "tiny": "75 MB",
    "base": "140 MB",
    "small": "460 MB",
    "medium": "1.5 GB",
    "large": "3 GB",
}


class WhisperTranscriber:
    """Transcribes audio segments using faster-whisper.

    This class handles only transcription - it expects pre-segmented
    audio (e.g., from VAD) rather than raw streaming audio.
    """

    def __init__(
        self,
        model_name: str = "small",
        language: str = "en",
        device: str = "cpu",
        compute_type: str = "int8",
        logger: Optional[logging.Logger] = None,
    ):
        """Initialize Whisper model.

        Args:
            model_name: Model size (tiny, base, small, medium, large)
            language: Language code for transcription
            device: Compute device (cpu or cuda)
            compute_type: Model precision (int8, float16, etc.)
            logger: Logger instance for status messages
        """
        self.language = language
        self._logger = logger

        size_str = WHISPER_MODEL_SIZES.get(model_name, "unknown size")
        self._log(f"Loading Whisper model: {model_name} ({size_str})")

        self.model = WhisperModel(
            model_name,
            device=device,
            compute_type=compute_type,
        )
        self._log("Whisper model ready.")

    def _log(self, message: str) -> None:
        """Log a message if logger is available."""
        if self._logger:
            self._logger.info(message)

    def transcribe(self, audio: np.ndarray) -> str:
        """Transcribe audio segment to text.

        Args:
            audio: Audio as float32 numpy array (normalized -1.0 to 1.0)

        Returns:
            Transcribed text, or empty string on error
        """
        if audio.size == 0:
            return ""

        try:
            segments, _ = self.model.transcribe(
                audio,
                language=self.language,
                beam_size=5,
            )
            text = " ".join(segment.text.strip() for segment in segments)
            return text
        except Exception as e:
            if self._logger:
                self._logger.error(f"Transcription failed: {e}")
            return ""
