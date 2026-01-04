"""Audio capture from microphone using sounddevice."""

import logging
from typing import Callable, Optional

import sounddevice as sd


class AudioStream:
    """
    Handles microphone audio capture using sounddevice.

    sounddevice is preferred over PyAudio because it has easier
    installation on Raspberry Pi and better cross-platform support.
    """

    def __init__(
        self,
        sample_rate: int = 16000,
        device_index: Optional[int] = None,
        channels: int = 1,
        block_size: int = 8000,
        callback: Optional[Callable[[bytes], None]] = None,
        logger: Optional[logging.Logger] = None,
    ):
        """
        Initialize the audio stream.

        Args:
            sample_rate: Sample rate in Hz (16000 required for Silero VAD)
            device_index: Microphone device index (None for default)
            channels: Number of audio channels (1 for mono)
            block_size: Samples per block (8000 = 0.5 seconds at 16kHz)
            callback: Function to call with audio bytes
            logger: Logger instance for status messages
        """
        self.sample_rate = sample_rate
        self.device_index = device_index
        self.channels = channels
        self.block_size = block_size
        self.callback = callback
        self._logger = logger
        self._stream: Optional[sd.RawInputStream] = None

    def _audio_callback(self, indata, frames, time_info, status) -> None:
        """Internal callback from sounddevice."""
        if status and self._logger:
            self._logger.warning(f"Audio status: {status}")

        if self.callback:
            self.callback(bytes(indata))

    def start(self) -> None:
        """Start capturing audio."""
        self._stream = sd.RawInputStream(
            samplerate=self.sample_rate,
            blocksize=self.block_size,
            device=self.device_index,
            dtype="int16",
            channels=self.channels,
            callback=self._audio_callback,
        )
        self._stream.start()

    def stop(self) -> None:
        """Stop capturing audio."""
        if self._stream:
            self._stream.stop()
            self._stream.close()
            self._stream = None

    def get_device_info(self) -> dict:
        """Get information about the audio device being used."""
        if self.device_index is not None:
            device_info = sd.query_devices(self.device_index)
        else:
            device_info = sd.query_devices(kind="input")

        # Get host API name
        hostapi_info = sd.query_hostapis(device_info["hostapi"])

        return {
            "name": device_info["name"],
            "hostapi": hostapi_info["name"],
            "channels": device_info["max_input_channels"],
            "sample_rate": device_info["default_samplerate"],
        }

    @staticmethod
    def list_devices() -> list:
        """List available audio input devices."""
        return sd.query_devices()
