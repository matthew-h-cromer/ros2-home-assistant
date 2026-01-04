"""Shared constants for assistant package."""

import os

# Default Whisper model
DEFAULT_WHISPER_MODEL = "small"

# Models directory (repo-root/models/)
_PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(_PACKAGE_DIR)))
MODELS_DIR = os.path.join(_REPO_ROOT, "models")
