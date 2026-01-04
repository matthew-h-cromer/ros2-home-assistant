"""Audio playback using sounddevice."""

import logging
import queue
import threading
from typing import Optional

import numpy as np
import sounddevice as sd


class AudioOutput:
    """
    Handles audio playback using sounddevice.

    Uses a background thread for non-blocking playback with
    queue-based audio submission.
    """

    def __init__(
        self,
        sample_rate: int = 22050,
        device_index: Optional[int] = None,
        channels: int = 1,
        logger: Optional[logging.Logger] = None,
    ):
        """Initialize audio output.

        Args:
            sample_rate: Sample rate in Hz
            device_index: Output device index (None for default)
            channels: Number of audio channels (1 for mono)
            logger: Logger instance for status messages
        """
        self.sample_rate = sample_rate
        self.device_index = device_index
        self.channels = channels
        self._logger = logger

        self._audio_queue: queue.Queue[Optional[np.ndarray]] = queue.Queue()
        self._running = threading.Event()
        self._playback_thread: Optional[threading.Thread] = None
        self._is_playing = threading.Event()

    def start(self) -> None:
        """Start the playback thread."""
        self._running.set()
        self._playback_thread = threading.Thread(
            target=self._playback_loop,
            daemon=True,
        )
        self._playback_thread.start()

    def stop(self) -> None:
        """Stop playback and clean up."""
        self._running.clear()
        self._audio_queue.put(None)
        if self._playback_thread:
            self._playback_thread.join(timeout=1.0)

    def play(self, audio: np.ndarray) -> None:
        """Queue audio for playback (non-blocking).

        Args:
            audio: Audio as int16 numpy array
        """
        if audio.size > 0:
            self._audio_queue.put(audio)

    def play_blocking(self, audio: np.ndarray) -> None:
        """Play audio and wait for completion.

        Args:
            audio: Audio as int16 numpy array
        """
        if audio.size == 0:
            return

        try:
            audio_float = audio.astype(np.float32) / 32768.0
            sd.play(audio_float, self.sample_rate, device=self.device_index)
            sd.wait()
        except Exception as e:
            if self._logger:
                self._logger.error(f"Playback error: {e}")

    def is_playing(self) -> bool:
        """Check if audio is currently playing."""
        return self._is_playing.is_set()

    def clear_queue(self) -> None:
        """Clear any pending audio in the queue."""
        while not self._audio_queue.empty():
            try:
                self._audio_queue.get_nowait()
            except queue.Empty:
                break

    def _playback_loop(self) -> None:
        """Background thread for audio playback."""
        while self._running.is_set():
            try:
                audio = self._audio_queue.get(timeout=0.1)

                if audio is None:
                    continue

                self._is_playing.set()
                self.play_blocking(audio)
                self._is_playing.clear()

            except queue.Empty:
                continue
            except Exception as e:
                if self._logger:
                    self._logger.error(f"Playback loop error: {e}")
                self._is_playing.clear()

    def get_device_info(self) -> dict:
        """Get information about the audio output device being used."""
        if self.device_index is not None:
            device_info = sd.query_devices(self.device_index)
        else:
            device_info = sd.query_devices(kind="output")

        hostapi_info = sd.query_hostapis(device_info["hostapi"])

        return {
            "name": device_info["name"],
            "hostapi": hostapi_info["name"],
            "channels": device_info["max_output_channels"],
            "sample_rate": device_info["default_samplerate"],
        }

    @staticmethod
    def list_devices() -> list:
        """List available audio output devices."""
        return sd.query_devices()
