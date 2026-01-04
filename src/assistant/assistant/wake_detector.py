"""Wake phrase detection for voice commands."""

import re
from dataclasses import dataclass


@dataclass
class WakeResult:
    """Result of wake detection."""

    triggered: bool
    command: str  # Text after wake phrase (empty if not triggered)
    wake_phrase: str  # Matched wake phrase (empty if not triggered)


class WakeDetector:
    """Detects wake phrases in transcribed text.

    Handles common speech-to-text variations and mishearings.
    Normalizes input for robust matching.
    """

    # Characters to strip from start/end of text and wake phrases
    STRIP_CHARS = " \t\n.,!?;:"

    # Regex for normalizing whitespace
    WHITESPACE_RE = re.compile(r"\s+")

    # Separator pattern: allows comma, colon, or whitespace between words
    SEP = r"[\s,:]+"

    # Optional separator (for variations like "heyross")
    OPT_SEP = r"[\s,:]*"

    def __init__(self, wake_word: str = "ross"):
        """Initialize wake detector.

        Args:
            wake_word: The core wake word to detect (default: "ross").
                       Patterns are automatically generated for common
                       variations and speech-to-text mishearings.
        """
        self._wake_word = wake_word.lower().strip()
        self._patterns = self._build_patterns()

    def _build_patterns(self) -> list[tuple[re.Pattern, str]]:
        """Build regex patterns for wake word detection.

        Returns list of (pattern, description) tuples, ordered by specificity
        (most specific first to prefer "hey ross" over just "ross").
        """
        w = re.escape(self._wake_word)
        patterns = []

        # "Hey <wake_word>" and common variations (most specific)
        hey_variants = [
            (rf"hey{self.SEP}{w}", "hey <wake>"),
            (rf"hey{self.OPT_SEP}{w}", "hey<wake>"),  # No space: "heyross"
            (rf"a{self.SEP}{w}", "a <wake>"),  # Common Whisper mishearing
            (rf"eh{self.SEP}{w}", "eh <wake>"),  # Another mishearing
            (rf"hay{self.SEP}{w}", "hay <wake>"),  # Phonetic variation
            (rf"hi{self.SEP}{w}", "hi <wake>"),  # "Hi Ross"
            (rf"okay{self.SEP}{w}", "okay <wake>"),  # "Okay Ross"
            (rf"ok{self.SEP}{w}", "ok <wake>"),
            (rf"yo{self.SEP}{w}", "yo <wake>"),
        ]

        for pattern, desc in hey_variants:
            patterns.append((self._compile(pattern), desc))

        # Just the wake word (less specific, checked last)
        patterns.append((self._compile(w), "<wake>"))

        return patterns

    def _compile(self, pattern: str) -> re.Pattern:
        """Compile pattern to match at start of text, capturing the rest."""
        # ^           - start of string
        # (pattern)   - capture the wake phrase
        # [\s,:]*     - optional separator after wake phrase
        # (.*)        - capture everything after (the command)
        # $           - end of string
        full = rf"^({pattern})[\s,:]*(.*)$"
        return re.compile(full, re.IGNORECASE)

    def _normalize(self, text: str) -> str:
        """Normalize text for matching.

        - Strips leading/trailing punctuation and whitespace
        - Collapses multiple whitespace to single space
        - Preserves case (patterns are case-insensitive)
        """
        text = text.strip(self.STRIP_CHARS)
        text = self.WHITESPACE_RE.sub(" ", text)
        return text

    def detect(self, text: str) -> WakeResult:
        """Check if text starts with a wake phrase.

        Args:
            text: Transcribed text to check.

        Returns:
            WakeResult with triggered=True and command text if triggered,
            or triggered=False with empty command if not.
        """
        if not text:
            return WakeResult(triggered=False, command="", wake_phrase="")

        normalized = self._normalize(text)
        if not normalized:
            return WakeResult(triggered=False, command="", wake_phrase="")

        for pattern, _desc in self._patterns:
            match = pattern.match(normalized)
            if match:
                wake_phrase = match.group(1).strip(self.STRIP_CHARS)
                command = match.group(2).strip(self.STRIP_CHARS)
                return WakeResult(
                    triggered=True,
                    command=command,
                    wake_phrase=wake_phrase,
                )

        return WakeResult(triggered=False, command="", wake_phrase="")
