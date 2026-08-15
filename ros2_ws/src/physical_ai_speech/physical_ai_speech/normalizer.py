"""Normalize already-recognized zh/en/fr speech without inventing confidence."""

from dataclasses import dataclass
import re

SUPPORTED_LANGUAGES = {"zh", "en", "fr"}
MIN_CONFIDENCE = 0.60


@dataclass(frozen=True)
class SpeechCommand:
    transcript: str
    language: str
    confidence: float
    intent: str


PHRASES = {
    "PICK_GREEN": {
        "zh": ("拿起绿球", "抓起绿球", "拿绿色球"),
        "en": ("pick up the green ball", "grab the green ball"),
        "fr": ("prends la boule verte", "attrape la boule verte"),
    },
    "DRAW_HEART": {
        "zh": ("画一个爱心", "画个爱心", "画一个心形"),
        "en": ("draw a heart", "draw the heart"),
        "fr": ("dessine un coeur", "dessine un cœur"),
    },
}


def _canon(text: str) -> str:
    return re.sub(r"[\s,.!?;:，。！？；：'\"]+", " ", text.strip().lower()).strip()


def normalize(transcript: str, language: str, confidence: float) -> SpeechCommand:
    if language not in SUPPORTED_LANGUAGES:
        raise ValueError("UNSUPPORTED_LANGUAGE")
    if not 0.0 <= confidence <= 1.0:
        raise ValueError("INVALID_CONFIDENCE")
    if confidence < MIN_CONFIDENCE:
        raise ValueError("UNCERTAIN_SPEECH")
    text = _canon(transcript)
    for intent, variants in PHRASES.items():
        if any(_canon(v) in text for v in variants[language]):
            return SpeechCommand(transcript=transcript, language=language, confidence=confidence, intent=intent)
    raise ValueError("UNKNOWN_SPEECH_INTENT")
