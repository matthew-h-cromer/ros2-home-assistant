"""Trigger phrase detection for voice commands."""

import re
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class TriggerResult:
    """Result of trigger detection."""

    triggered: bool
    command: str  # Text after trigger phrase (empty if not triggered)
    trigger_phrase: str  # Matched trigger phrase (empty if not triggered)


class TriggerDetector:
    """Detects trigger phrases in transcribed text.

    Supports multiple trigger phrases and returns the text
    following the trigger (the command portion).
    """

    # Common Whisper transcription variations for "hey ross"
    DEFAULT_PATTERNS = [
        r"hey[,\s]+ross",
        r"hey[,\s]*ross",
        r"a[,\s]+ross",  # Common mishearing
        r"eh[,\s]+ross",  # Common mishearing
    ]

    def __init__(
        self,
        trigger_phrases: Optional[List[str]] = None,
        case_sensitive: bool = False,
    ):
        """Initialize trigger detector.

        Args:
            trigger_phrases: List of trigger phrases to detect.
                If None, uses default "hey ross" patterns.
            case_sensitive: Whether matching is case-sensitive.
        """
        self._case_sensitive = case_sensitive
        self._patterns: List[re.Pattern] = []

        if trigger_phrases:
            for phrase in trigger_phrases:
                escaped = re.escape(phrase)
                # Allow comma or whitespace between words
                pattern = escaped.replace(r"\ ", r"[,\s]+")
                self._patterns.append(self._compile_pattern(pattern))
        else:
            for pattern in self.DEFAULT_PATTERNS:
                self._patterns.append(self._compile_pattern(pattern))

    def _compile_pattern(self, pattern: str) -> re.Pattern:
        """Compile a pattern with appropriate flags."""
        # Match at start, capture everything after trigger
        full_pattern = rf"^\s*(?:{pattern})[,:\s]*(.*)$"
        flags = 0 if self._case_sensitive else re.IGNORECASE
        return re.compile(full_pattern, flags)

    def detect(self, text: str) -> TriggerResult:
        """Check if text starts with a trigger phrase.

        Args:
            text: Transcribed text to check.

        Returns:
            TriggerResult with triggered=True and command text if triggered,
            or triggered=False with empty command if not.
        """
        if not text or not text.strip():
            return TriggerResult(triggered=False, command="", trigger_phrase="")

        for pattern in self._patterns:
            match = pattern.match(text)
            if match:
                command = match.group(1).strip()
                # Extract the matched trigger phrase
                trigger_end = match.start(1) if match.group(1) else len(text)
                trigger_phrase = text[:trigger_end].strip().rstrip(",: ")

                return TriggerResult(
                    triggered=True,
                    command=command,
                    trigger_phrase=trigger_phrase,
                )

        return TriggerResult(triggered=False, command="", trigger_phrase="")
