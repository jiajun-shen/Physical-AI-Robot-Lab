"""Drawing-domain language and geometry contracts for Phase 6.

The module deliberately understands only drawing instructions.  It produces a
typed :class:`DrawingIntent` whose geometry is subsequently generated in board
local coordinates; it never contains robot joint values or world coordinates.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
import math
import re


class DrawingIntentError(ValueError):
    """A drawing instruction cannot be represented safely."""


class Shape(StrEnum):
    CIRCLE = "circle"
    SQUARE = "square"
    TRIANGLE = "triangle"
    HEART = "heart"
    WAVE = "wave"
    LINE = "line"


class BoardPosition(StrEnum):
    CENTER = "center"
    LEFT = "left"
    RIGHT = "right"
    TOP = "top"
    BOTTOM = "bottom"


@dataclass(frozen=True)
class DrawingIntent:
    """A unit-safe specification in the drawing-board coordinate system.

    ``size_m`` is the largest requested in-plane extent, ``position`` is a
    semantic board placement, and ``orientation_rad`` rotates the generated
    curve around the board normal.  Neither field is an ABB/world shortcut.
    """

    shape: Shape
    size_m: float = 0.10
    position: BoardPosition = BoardPosition.CENTER
    orientation_rad: float = 0.0
    source_text: str = ""

    def __post_init__(self) -> None:
        if not math.isfinite(self.size_m) or not 0.04 <= self.size_m <= 0.24:
            raise DrawingIntentError("size_m must be finite and within [0.04, 0.24] m")
        if not math.isfinite(self.orientation_rad) or abs(self.orientation_rad) > 2.0 * math.pi:
            raise DrawingIntentError("orientation_rad must be finite and within [-2π, 2π]")


_SHAPES: tuple[tuple[Shape, tuple[str, ...]], ...] = (
    (Shape.CIRCLE, ("circle", "圆形", "画圆", "一个圆", "圆")),
    (Shape.SQUARE, ("square", "正方形", "方形")),
    (Shape.TRIANGLE, ("triangle", "三角形")),
    (Shape.HEART, ("heart", "coeur", "cœur", "爱心", "心形")),
    (Shape.WAVE, ("wave", "wavy", "波浪线", "波浪")),
    (Shape.LINE, ("line", "直线", "线条")),
)


def _canonical(text: str) -> str:
    return re.sub(r"[\s,.;:!?，。；：！？'\"]+", " ", text.lower()).strip()


def _shape_from_text(text: str) -> Shape:
    matches = [shape for shape, aliases in _SHAPES if any(alias in text for alias in aliases)]
    if len(matches) != 1:
        if not matches:
            raise DrawingIntentError("UNKNOWN_DRAWING_SHAPE")
        raise DrawingIntentError("AMBIGUOUS_DRAWING_SHAPE")
    return matches[0]


def _size_from_text(text: str) -> float:
    explicit = re.search(r"(?<!\d)(0(?:\.\d+)?|1(?:\.0+)?)\s*(?:m|米|met(?:er|re)s?)\b", text)
    if explicit:
        value = float(explicit.group(1))
        if not 0.04 <= value <= 0.24:
            raise DrawingIntentError("DRAWING_SIZE_OUT_OF_RANGE")
        return value
    if any(token in text for token in ("small", "smaller", "little", "小一点", "小些", "小的", "小")):
        return 0.10
    if any(token in text for token in ("large", "bigger", "big", "大一点", "大的", "大")):
        return 0.18
    # The default is the calibrated, currently executable board trace size.
    # Larger requests remain typed and are safety-gated by live MoveIt.
    return 0.10


def _position_from_text(text: str) -> BoardPosition:
    positions = []
    if any(token in text for token in ("left", "左边", "左侧", "左")):
        positions.append(BoardPosition.LEFT)
    if any(token in text for token in ("right", "右边", "右侧", "右")):
        positions.append(BoardPosition.RIGHT)
    if any(token in text for token in ("top", "upper", "上方", "上边", "上")):
        positions.append(BoardPosition.TOP)
    if any(token in text for token in ("bottom", "lower", "下方", "下边", "下")):
        positions.append(BoardPosition.BOTTOM)
    if not positions:
        return BoardPosition.CENTER
    if len(positions) != 1:
        raise DrawingIntentError("AMBIGUOUS_BOARD_POSITION")
    return positions[0]


def _orientation_from_text(text: str) -> float:
    match = re.search(r"(?:rotate|rotated|rotation|旋转)\s*(?:by)?\s*(-?\d+(?:\.\d+)?)\s*(?:°|deg|degrees|度)", text)
    if not match:
        return 0.0
    value = math.radians(float(match.group(1)))
    if abs(value) > 2.0 * math.pi:
        raise DrawingIntentError("DRAWING_ORIENTATION_OUT_OF_RANGE")
    return value


def parse_drawing_instruction(instruction: str) -> DrawingIntent:
    """Parse one bounded zh/en/fr drawing command into a typed intent.

    This is intentionally a deterministic drawing-domain parser.  General
    task planning belongs to Phase 8 and must not be smuggled into Phase 6.
    """

    if not isinstance(instruction, str) or not instruction.strip():
        raise DrawingIntentError("EMPTY_DRAWING_INSTRUCTION")
    text = _canonical(instruction)
    return DrawingIntent(
        shape=_shape_from_text(text),
        size_m=_size_from_text(text),
        position=_position_from_text(text),
        orientation_rad=_orientation_from_text(text),
        source_text=instruction,
    )
