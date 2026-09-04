"""Normalize local ASR output into a bounded multilingual speech contract.

This module intentionally does *not* perform ASR or infer a language itself.
Its caller must supply the language and confidence reported by the local ASR
backend. The normalizer only accepts commands that it can classify without
inventing either value, and preserves the original transcript byte-for-byte
for downstream domain parsers.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import re
from typing import Any


SUPPORTED_LANGUAGES = frozenset({"zh", "en", "fr"})
MIN_CONFIDENCE = 0.60


@dataclass(frozen=True, slots=True)
class SpeechCommand:
    """A validated, ASR-derived command.

    ``detected_language`` is deliberately explicit: it is the language
    emitted by ASR, not a language selected by a caller. ``language`` remains
    a read-only compatibility alias for the pre-Phase-7 contract.
    """

    transcript: str
    detected_language: str
    confidence: float
    timestamp_ns: int
    intent: str

    def __post_init__(self) -> None:
        _validate_transcript(self.transcript)
        _validate_language(self.detected_language)
        _validate_confidence(self.confidence)
        _validate_timestamp(self.timestamp_ns)
        if not isinstance(self.intent, str) or not self.intent:
            raise ValueError("INVALID_SPEECH_INTENT")

    @property
    def language(self) -> str:
        """Compatibility alias for callers of the earlier speech contract."""

        return self.detected_language

    def to_dict(self) -> dict[str, Any]:
        """Return JSON-ready data using the Phase-7 field names."""

        return asdict(self)


# Existing pickup support is retained, while drawing remains generic and is
# routed to the Phase-6 drawing-domain parser by ``speech_to_drawing``.
PHRASES = {
    "PICK_GREEN": {
        "zh": ("拿起绿球", "抓起绿球", "拿绿色球"),
        "en": ("pick up the green ball", "grab the green ball"),
        "fr": ("prends la boule verte", "attrape la boule verte"),
    },
}


# These command/shape pairs deliberately mirror the bounded Phase-6 parser
# vocabulary needed by Phase 7. In particular, accepting a local-language
# shape which the downstream parser cannot understand would create a command
# that cannot be executed, so it is rejected here instead.
_DRAWING_PATTERNS: dict[str, tuple[re.Pattern[str], tuple[str, ...]]] = {
    "zh": (
        re.compile("画"),
        ("圆", "圈", "方形", "正方形", "三角形", "爱心", "心形", "波浪", "直线", "线条"),
    ),
    "en": (
        re.compile(r"\bdraw\b"),
        ("circle", "square", "triangle", "heart", "wave", "wavy", "line"),
    ),
    "fr": (
        re.compile(r"\bdessin(?:e|es|ez|er|ons|ent)\b"),
        # Phase 6 currently exposes French aliases for heart and the shared
        # token ``triangle``. Do not claim French circle/square support until
        # the drawing-domain parser actually supports those words.
        ("triangle", "coeur", "cœur"),
    ),
}


def _canon(text: str) -> str:
    return re.sub(r"[\s,.!?;:，。！？；：'\"]+", " ", text.strip().lower()).strip()


def _validate_transcript(transcript: object) -> None:
    if not isinstance(transcript, str) or not transcript.strip():
        raise ValueError("EMPTY_TRANSCRIPT")


def _validate_language(detected_language: object) -> None:
    if not isinstance(detected_language, str) or detected_language not in SUPPORTED_LANGUAGES:
        raise ValueError("UNSUPPORTED_LANGUAGE")


def _validate_confidence(confidence: object) -> None:
    if isinstance(confidence, bool) or not isinstance(confidence, (int, float)):
        raise ValueError("INVALID_CONFIDENCE")
    value = float(confidence)
    if not math.isfinite(value) or not 0.0 <= value <= 1.0:
        raise ValueError("INVALID_CONFIDENCE")
    if value < MIN_CONFIDENCE:
        raise ValueError("UNCERTAIN_SPEECH")


def _validate_timestamp(timestamp_ns: object) -> None:
    if isinstance(timestamp_ns, bool) or not isinstance(timestamp_ns, int) or timestamp_ns <= 0:
        raise ValueError("INVALID_TIMESTAMP")


def _is_drawing_command(canonical_text: str, detected_language: str) -> bool:
    verb_pattern, shape_tokens = _DRAWING_PATTERNS[detected_language]
    return bool(verb_pattern.search(canonical_text)) and any(token in canonical_text for token in shape_tokens)


def _resolve_detected_language(detected_language: object, legacy_language: object) -> str:
    """Accept the legacy named ``language`` argument without ambiguity."""

    if detected_language is not None and legacy_language is not None and detected_language != legacy_language:
        raise ValueError("CONFLICTING_LANGUAGE")
    resolved = detected_language if detected_language is not None else legacy_language
    if resolved is None:
        raise ValueError("MISSING_DETECTED_LANGUAGE")
    _validate_language(resolved)
    return resolved


def normalize(
    transcript: str,
    detected_language: str | None = None,
    confidence: float | None = None,
    timestamp_ns: int | None = None,
    *,
    language: str | None = None,
) -> SpeechCommand:
    """Validate one ASR result and classify only known bounded commands.

    The stable Phase-7 positional contract is ``(transcript,
    detected_language, confidence, timestamp_ns)``. The legacy named
    ``language=`` argument remains supported, but both names may only be used
    when they agree. The language is never inferred from text here.
    """

    _validate_transcript(transcript)
    resolved_language = _resolve_detected_language(detected_language, language)
    _validate_confidence(confidence)
    _validate_timestamp(timestamp_ns)

    text = _canon(transcript)
    for intent, variants in PHRASES.items():
        if any(_canon(variant) in text for variant in variants[resolved_language]):
            return SpeechCommand(
                transcript=transcript,
                detected_language=resolved_language,
                confidence=float(confidence),
                timestamp_ns=timestamp_ns,
                intent=intent,
            )
    if _is_drawing_command(text, resolved_language):
        return SpeechCommand(
            transcript=transcript,
            detected_language=resolved_language,
            confidence=float(confidence),
            timestamp_ns=timestamp_ns,
            intent="DRAW",
        )
    raise ValueError("UNKNOWN_SPEECH_INTENT")
