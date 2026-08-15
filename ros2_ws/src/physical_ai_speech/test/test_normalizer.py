import pytest
from physical_ai_speech.normalizer import normalize


@pytest.mark.parametrize("text,lang", [("拿起绿球", "zh"), ("Please pick up the green ball", "en"), ("Prends la boule verte", "fr")])
def test_three_languages_normalize_same_intent(text, lang):
    cmd = normalize(text, lang, 0.91, 123456789)
    assert cmd.intent == "PICK_GREEN"
    assert cmd.timestamp_ns == 123456789


def test_low_confidence_is_rejected():
    with pytest.raises(ValueError, match="UNCERTAIN_SPEECH"):
        normalize("draw a heart", "en", 0.4, 123456789)


def test_unknown_command_is_not_guessed():
    with pytest.raises(ValueError, match="UNKNOWN_SPEECH_INTENT"):
        normalize("open the window", "en", 0.99, 123456789)


def test_missing_timestamp_is_rejected():
    with pytest.raises(ValueError, match="INVALID_TIMESTAMP"):
        normalize("draw a heart", "en", 0.99, 0)
