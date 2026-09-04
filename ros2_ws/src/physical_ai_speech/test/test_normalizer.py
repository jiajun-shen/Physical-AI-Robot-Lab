import pytest

from physical_ai_speech.normalizer import SpeechCommand, normalize


@pytest.mark.parametrize(
    "text,detected_language",
    [
        ("画一个圆。", "zh"),
        ("画一个圈", "zh"),
        ("Draw a square.", "en"),
        ("Dessine un triangle.", "fr"),
    ],
)
def test_required_three_languages_normalize_generic_draw(text, detected_language):
    command = normalize(text, detected_language, 0.91, 123456789)

    assert command.intent == "DRAW"
    assert command.transcript == text
    assert command.detected_language == detected_language
    assert command.language == detected_language  # Legacy compatibility alias.
    assert command.timestamp_ns == 123456789
    assert command.to_dict()["detected_language"] == detected_language


@pytest.mark.parametrize(
    "text,detected_language",
    [
        ("拿起绿球", "zh"),
        ("Please pick up the green ball", "en"),
        ("Prends la boule verte", "fr"),
    ],
)
def test_existing_pickup_contract_is_retained(text, detected_language):
    command = normalize(text, detected_language, 0.91, 123456789)

    assert command.intent == "PICK_GREEN"
    assert command.detected_language == detected_language


def test_legacy_named_language_argument_is_supported():
    command = normalize("Draw a square", confidence=0.91, timestamp_ns=123456789, language="en")

    assert command.detected_language == "en"


def test_low_confidence_is_rejected():
    with pytest.raises(ValueError, match="UNCERTAIN_SPEECH"):
        normalize("Draw a square", "en", 0.4, 123456789)


@pytest.mark.parametrize("confidence", [float("nan"), float("inf"), -0.01, 1.01, True, "0.91"])
def test_invalid_confidence_is_rejected(confidence):
    with pytest.raises(ValueError, match="INVALID_CONFIDENCE"):
        normalize("Draw a square", "en", confidence, 123456789)


def test_unknown_command_is_not_guessed():
    with pytest.raises(ValueError, match="UNKNOWN_SPEECH_INTENT"):
        normalize("open the window", "en", 0.99, 123456789)


@pytest.mark.parametrize("timestamp_ns", [0, -1, True, 1.5])
def test_missing_or_invalid_timestamp_is_rejected(timestamp_ns):
    with pytest.raises(ValueError, match="INVALID_TIMESTAMP"):
        normalize("Draw a square", "en", 0.99, timestamp_ns)


def test_unsupported_or_conflicting_language_is_rejected():
    with pytest.raises(ValueError, match="UNSUPPORTED_LANGUAGE"):
        normalize("Draw a square", "de", 0.99, 123456789)
    with pytest.raises(ValueError, match="CONFLICTING_LANGUAGE"):
        normalize("Draw a square", "en", 0.99, 123456789, language="fr")


def test_speech_command_constructor_validates_contract():
    with pytest.raises(ValueError, match="EMPTY_TRANSCRIPT"):
        SpeechCommand("", "en", 0.99, 123456789, "DRAW")
    with pytest.raises(ValueError, match="UNCERTAIN_SPEECH"):
        SpeechCommand("Draw a square", "en", 0.59, 123456789, "DRAW")
