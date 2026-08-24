"""Bounded handoff from Phase-7 speech to the Phase-6 drawing contract."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from physical_ai_speech.normalizer import SpeechCommand

if TYPE_CHECKING:
    from physical_ai_drawing.drawing_intent import DrawingIntent


DRAWING_INTERFACE = "physical_ai_drawing.parse_drawing_instruction"


@dataclass(frozen=True, slots=True)
class SpeechDrawingDispatch:
    """A successful, inspectable handoff to the Phase-6 parser.

    The bridge contains no geometry generation or task planning. Its sole job
    is to give the exact ASR transcript to the existing drawing-domain parser
    and retain the typed result for downstream execution.
    """

    command: SpeechCommand
    drawing_intent: DrawingIntent

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-ready downstream record with exact source text."""

        return {
            "interface": DRAWING_INTERFACE,
            "status": "ACCEPTED",
            "speech_command": self.command.to_dict(),
            "drawing_intent": serialize_drawing_intent(self.drawing_intent),
        }


def serialize_drawing_intent(drawing_intent: DrawingIntent) -> dict[str, Any]:
    """Serialize the public Phase-6 ``DrawingIntent`` fields explicitly."""

    return {
        "shape": drawing_intent.shape.value,
        "size_m": drawing_intent.size_m,
        "position": drawing_intent.position.value,
        "orientation_rad": drawing_intent.orientation_rad,
        "source_text": drawing_intent.source_text,
    }


def route_to_drawing(command: SpeechCommand) -> SpeechDrawingDispatch:
    """Parse one generic draw command through the unchanged Phase-6 interface.

    The original transcript is passed directly, rather than translated or
    reconstructed. Any downstream parsing failure is reported as a rejected
    drawing handoff; it is never silently downgraded to a guessed intent.
    """

    if not isinstance(command, SpeechCommand):
        raise ValueError("INVALID_SPEECH_COMMAND")
    if command.intent != "DRAW":
        raise ValueError("UNSUPPORTED_DOWNSTREAM_INTENT")

    try:
        from physical_ai_drawing.drawing_intent import DrawingIntentError, parse_drawing_instruction

        # Do not alter this value: Phase-7 evidence must prove that ASR output
        # entered Phase 6 as the original transcript.
        drawing_intent = parse_drawing_instruction(command.transcript)
    except DrawingIntentError as error:
        raise ValueError(f"DRAWING_INTERFACE_REJECTED:{error}") from error

    if drawing_intent.source_text != command.transcript:
        raise RuntimeError("DRAWING_SOURCE_TEXT_MISMATCH")
    return SpeechDrawingDispatch(command=command, drawing_intent=drawing_intent)


def route_speech_command_to_drawing(command: SpeechCommand) -> dict[str, Any]:
    """Return the stable JSON-ready Phase-7-to-Phase-6 dispatch record."""

    return route_to_drawing(command).to_dict()
