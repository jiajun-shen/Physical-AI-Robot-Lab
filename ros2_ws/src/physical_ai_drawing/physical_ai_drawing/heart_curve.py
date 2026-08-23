"""Legacy-compatible heart helper backed by the board-local geometry layer."""

from dataclasses import dataclass
from typing import List

from .drawing_intent import DrawingIntent, Shape
from .geometry import extent as local_extent
from .geometry import generate_local_geometry


@dataclass(frozen=True)
class Point3:
    x: float
    y: float
    z: float


def generate_heart(scale_m: float = 0.12, samples: int = 240, board_x: float = 0.0) -> List[Point3]:
    """Return a generated heart in a caller-named plane for historical tests.

    New runtime code must use ``DrawingIntent`` and ``BoardFrame`` rather than
    treating ``board_x`` as a world coordinate.
    """

    if not (0.04 <= scale_m <= 0.24):
        raise ValueError("scale_m must be within [0.04, 0.24] m")
    if samples < 32:
        raise ValueError("samples must be >= 32")
    local = generate_local_geometry(DrawingIntent(shape=Shape.HEART, size_m=scale_m), samples=samples)
    return [Point3(board_x, point.horizontal_m, point.vertical_m) for point in local]


def extent(points: List[Point3]) -> tuple[float, float]:
    return (
        max(point.y for point in points) - min(point.y for point in points),
        max(point.z for point in points) - min(point.z for point in points),
    )
