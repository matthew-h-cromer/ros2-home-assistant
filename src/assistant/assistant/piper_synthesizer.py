"""Piper TTS synthesis wrapper."""

import logging
import os
from pathlib import Path
from typing import Generator, Optional

import numpy as np

from .constants import DEFAULT_PIPER_VOICE, MODELS_DIR, PIPER_SAMPLE_RATE
from .hf_downloader import download_piper_voice

# Piper models stored in repo-local directory
PIPER_MODELS_DIR = os.path.join(MODELS_DIR, "piper")


class PiperSynthesizer:
    """Synthesizes speech from text using Piper TTS.

    This class handles text-to-speech synthesis using Piper,
    a fast neural TTS engine designed for offline use.
    """

    def __init__(
        self,
        voice: str = DEFAULT_PIPER_VOICE,
        speaker_id: Optional[int] = None,
        length_scale: float = 1.0,
        logger: Optional[logging.Logger] = None,
    ):
        """Initialize Piper synthesizer.

        Args:
            voice: Voice model name (e.g., "en_US-lessac-medium")
            speaker_id: Speaker ID for multi-speaker models (None for single)
            length_scale: Controls speech rate (1.0 = normal, <1 faster, >1 slower)
            logger: Logger instance for status messages
        """
        self._voice = voice
        self._speaker_id = speaker_id
        self._length_scale = length_scale
        self._logger = logger
        self._piper = None
        self._syn_config = None

        self._log(f"Loading Piper voice: {voice}")
        self._load_model()
        self._log("Piper TTS ready.")

    def _log(self, message: str) -> None:
        """Log a message if logger is available."""
        if self._logger:
            self._logger.info(message)

    def _load_model(self) -> None:
        """Load the Piper model, downloading if necessary."""
        from piper import PiperVoice, SynthesisConfig

        model_path = Path(PIPER_MODELS_DIR) / f"{self._voice}.onnx"
        config_path = Path(PIPER_MODELS_DIR) / f"{self._voice}.onnx.json"

        if not model_path.exists():
            download_piper_voice(self._voice, PIPER_MODELS_DIR, self._logger)

        self._piper = PiperVoice.load(
            str(model_path),
            config_path=str(config_path) if config_path.exists() else None,
            use_cuda=False,
        )

        # Create synthesis config with our settings
        self._syn_config = SynthesisConfig(
            speaker_id=self._speaker_id,
            length_scale=self._length_scale,
        )

    @property
    def sample_rate(self) -> int:
        """Return the output sample rate of synthesized audio."""
        if self._piper:
            return self._piper.config.sample_rate
        return PIPER_SAMPLE_RATE

    def synthesize(self, text: str) -> np.ndarray:
        """Synthesize text to audio.

        Args:
            text: Text to synthesize

        Returns:
            Audio as int16 numpy array at model's sample rate
        """
        if not text.strip():
            return np.array([], dtype=np.int16)

        try:
            audio_chunks = []
            for chunk in self._piper.synthesize(text, self._syn_config):
                audio_chunks.append(chunk.audio_int16_array)

            if audio_chunks:
                return np.concatenate(audio_chunks)
            return np.array([], dtype=np.int16)

        except Exception as e:
            if self._logger:
                self._logger.error(f"Synthesis failed: {e}")
            return np.array([], dtype=np.int16)

    def synthesize_stream(self, text: str) -> Generator[np.ndarray, None, None]:
        """Synthesize text to streaming audio chunks.

        Args:
            text: Text to synthesize

        Yields:
            Audio chunks as int16 numpy arrays
        """
        if not text.strip():
            return

        try:
            for chunk in self._piper.synthesize(text, self._syn_config):
                yield chunk.audio_int16_array

        except Exception as e:
            if self._logger:
                self._logger.error(f"Streaming synthesis failed: {e}")
