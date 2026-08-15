import pytest
from physical_ai_speech.normalizer import normalize


@pytest.mark.parametrize("text,lang", [("拿起绿球", "zh"), ("Please pick up the green ball", "en"), ("Prends la boule verte", "fr")])
def test_three_languages_normalize_same_intent(text, lang):
    assert normalize(text, lang, 0.91).intent == "PICK_GREEN"


def test_low_confidence_is_rejected():
    with pytest.raises(ValueError, match="UNCERTAIN_SPEECH"):
        normalize("draw a heart", "en", 0.4)


def test_unknown_command_is_not_guessed():
    with pytest.raises(ValueError, match="UNKNOWN_SPEECH_INTENT"):
        normalize("open the window", "en", 0.99)
