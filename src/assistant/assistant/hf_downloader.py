"""Hugging Face model downloader utility."""

import logging
import os
import urllib.request
from pathlib import Path
from typing import Optional


def download_file(
    url: str,
    dest_path: Path,
    logger: Optional[logging.Logger] = None,
) -> None:
    """Download a file from a URL.

    Args:
        url: URL to download from
        dest_path: Destination path for the file
        logger: Optional logger for status messages
    """
    if logger:
        logger.info(f"Downloading {url}")
    urllib.request.urlretrieve(url, dest_path)


def download_piper_voice(
    voice: str,
    models_dir: str,
    logger: Optional[logging.Logger] = None,
) -> tuple[Path, Path]:
    """Download a Piper voice model from Hugging Face.

    Args:
        voice: Voice name (e.g., "en_US-lessac-medium")
        models_dir: Directory to save model files
        logger: Optional logger for status messages

    Returns:
        Tuple of (model_path, config_path)
    """
    base_url = "https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0"

    # Parse voice name: en_US-lessac-medium -> lang=en, region=en_US, name=lessac, quality=medium
    parts = voice.split("-")
    lang_region = parts[0]  # en_US
    name = parts[1]  # lessac
    quality = parts[2] if len(parts) > 2 else "medium"
    lang = lang_region.split("_")[0]  # en

    # Build URL paths
    voice_path = f"{lang}/{lang_region}/{name}/{quality}/{voice}"
    model_url = f"{base_url}/{voice_path}.onnx"
    config_url = f"{base_url}/{voice_path}.onnx.json"

    os.makedirs(models_dir, exist_ok=True)
    model_path = Path(models_dir) / f"{voice}.onnx"
    config_path = Path(models_dir) / f"{voice}.onnx.json"

    if logger:
        logger.info(f"Downloading Piper voice model: {voice}")

    download_file(model_url, model_path, logger)
    download_file(config_url, config_path, logger)

    if logger:
        logger.info("Voice model downloaded.")

    return model_path, config_path
