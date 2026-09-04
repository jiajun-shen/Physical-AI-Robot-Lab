import pytest

from physical_ai_speech.normalizer import SpeechCommand, normalize
from physical_ai_speech.speech_to_drawing import (
    DRAWING_INTERFACE,
    route_speech_command_to_drawing,
    route_to_drawing,
)


@pytest.mark.parametrize(
    "text,detected_language,shape",
    [
        ("画一个圆。", "zh", "circle"),
        ("画一个圈", "zh", "circle"),
        ("Draw a square.", "en", "square"),
        ("Dessine un triangle.", "fr", "triangle"),
    ],
)
def test_required_transcripts_route_unchanged_to_phase6(text, detected_language, shape):
    command = normalize(text, detected_language, 0.91, 123456789)
    dispatch = route_to_drawing(command)
    record = route_speech_command_to_drawing(command)

    assert dispatch.drawing_intent.shape.value == shape
    assert dispatch.drawing_intent.source_text == text
    assert record["interface"] == DRAWING_INTERFACE
    assert record["status"] == "ACCEPTED"
    assert record["speech_command"]["transcript"] == text
    assert record["speech_command"]["detected_language"] == detected_language
    assert record["drawing_intent"]["shape"] == shape
    assert record["drawing_intent"]["source_text"] == text


def test_non_drawing_intent_never_enters_phase6_drawing_interface():
    command = normalize("pick up the green ball", "en", 0.91, 123456789)

    with pytest.raises(ValueError, match="UNSUPPORTED_DOWNSTREAM_INTENT"):
        route_to_drawing(command)


def test_manual_draw_command_with_unparseable_text_is_rejected_by_phase6():
    command = SpeechCommand("Draw a hexagon", "en", 0.91, 123456789, "DRAW")

    with pytest.raises(ValueError, match="DRAWING_INTERFACE_REJECTED"):
        route_to_drawing(command)
