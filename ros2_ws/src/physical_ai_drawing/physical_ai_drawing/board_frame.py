"""Named drawing-board frame mapping; no drawing geometry is authored in world XYZ."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from .drawing_intent import DrawingIntentError
from .geometry import BoardPoint


Vector3 = tuple[float, float, float]


def _dot(left: Vector3, right: Vector3) -> float:
    return sum(first * second for first, second in zip(left, right))


def _norm(value: Vector3) -> float:
    return math.sqrt(_dot(value, value))


@dataclass(frozen=True)
class BoardFrame:
    """A live board-center observation plus calibrated local axes.

    The center is supplied by perception at runtime.  The axes and front-face
    offset are a calibrated board-frame definition, not an authored drawing
    path in the robot/world frame.
    """

    center_world_m: Vector3
    normal_world: Vector3
    horizontal_world: Vector3
    vertical_world: Vector3
    front_surface_offset_m: float

    def __post_init__(self) -> None:
        vectors = (self.center_world_m, self.normal_world, self.horizontal_world, self.vertical_world)
        if not all(math.isfinite(component) for vector in vectors for component in vector):
            raise DrawingIntentError("board frame contains non-finite values")
        for label, vector in (("normal", self.normal_world), ("horizontal", self.horizontal_world), ("vertical", self.vertical_world)):
            if not math.isclose(_norm(vector), 1.0, rel_tol=0.0, abs_tol=1e-6):
                raise DrawingIntentError(f"board {label} axis must be unit length")
        if any(abs(_dot(first, second)) > 1e-6 for first, second in ((self.normal_world, self.horizontal_world), (self.normal_world, self.vertical_world), (self.horizontal_world, self.vertical_world))):
            raise DrawingIntentError("board axes must be orthogonal")
        if not math.isfinite(self.front_surface_offset_m) or self.front_surface_offset_m < 0.0:
            raise DrawingIntentError("board front surface offset must be finite and non-negative")

    def to_world(self, point: BoardPoint, *, standoff_m: float) -> Vector3:
        if not math.isfinite(standoff_m) or standoff_m < 0.0:
            raise DrawingIntentError("drawing standoff must be finite and non-negative")
        normal_distance = self.front_surface_offset_m + standoff_m + point.normal_m
        return tuple(
            self.center_world_m[index]
            + normal_distance * self.normal_world[index]
            + point.horizontal_m * self.horizontal_world[index]
            + point.vertical_m * self.vertical_world[index]
            for index in range(3)
        )

    def map_points(self, points: Iterable[BoardPoint], *, standoff_m: float) -> list[Vector3]:
        return [self.to_world(point, standoff_m=standoff_m) for point in points]
