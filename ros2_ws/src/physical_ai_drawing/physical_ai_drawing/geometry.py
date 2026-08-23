"""Runtime-generated board-local drawing geometry."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, pi, sin
from typing import Iterable

from .drawing_intent import BoardPosition, DrawingIntent, DrawingIntentError, Shape


@dataclass(frozen=True)
class BoardPoint:
    """A point in board-local coordinates: normal, horizontal, vertical."""

    normal_m: float
    horizontal_m: float
    vertical_m: float


def _normalize(points: Iterable[tuple[float, float]], size_m: float) -> list[tuple[float, float]]:
    raw = list(points)
    if len(raw) < 2:
        raise DrawingIntentError("drawing curve needs at least two points")
    min_u, max_u = min(point[0] for point in raw), max(point[0] for point in raw)
    min_v, max_v = min(point[1] for point in raw), max(point[1] for point in raw)
    span = max(max_u - min_u, max_v - min_v)
    if span <= 0.0:
        raise DrawingIntentError("drawing curve has zero extent")
    return [((u - (min_u + max_u) / 2.0) * size_m / span, (v - (min_v + max_v) / 2.0) * size_m / span) for u, v in raw]


def _closed_polygon(vertices: tuple[tuple[float, float], ...], samples: int) -> list[tuple[float, float]]:
    segment_count = len(vertices)
    if samples < segment_count + 1:
        raise DrawingIntentError(f"{segment_count}-edge polygon needs at least {segment_count + 1} samples")
    points: list[tuple[float, float]] = []
    for index in range(samples):
        progress = index / (samples - 1)
        coordinate = progress * segment_count
        edge = min(segment_count - 1, int(coordinate))
        fraction = coordinate - edge
        start, end = vertices[edge], vertices[(edge + 1) % segment_count]
        points.append((start[0] + (end[0] - start[0]) * fraction, start[1] + (end[1] - start[1]) * fraction))
    return points


def _raw_shape(shape: Shape, samples: int) -> list[tuple[float, float]]:
    if samples < 16:
        raise DrawingIntentError("samples must be >= 16")
    if shape is Shape.CIRCLE:
        return [(cos(2.0 * pi * i / (samples - 1)), sin(2.0 * pi * i / (samples - 1))) for i in range(samples)]
    if shape is Shape.HEART:
        return [
            (
                16.0 * sin(2.0 * pi * i / (samples - 1)) ** 3,
                13.0 * cos(2.0 * pi * i / (samples - 1))
                - 5.0 * cos(4.0 * pi * i / (samples - 1))
                - 2.0 * cos(6.0 * pi * i / (samples - 1))
                - cos(8.0 * pi * i / (samples - 1)),
            )
            for i in range(samples)
        ]
    if shape is Shape.SQUARE:
        return _closed_polygon(((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)), samples)
    if shape is Shape.TRIANGLE:
        return _closed_polygon(((0.0, 1.0), (-0.866, -0.5), (0.866, -0.5)), samples)
    if shape is Shape.WAVE:
        return [(-1.0 + 2.0 * i / (samples - 1), 0.42 * sin(4.0 * pi * i / (samples - 1))) for i in range(samples)]
    if shape is Shape.LINE:
        return [(-1.0 + 2.0 * i / (samples - 1), 0.0) for i in range(samples)]
    raise DrawingIntentError(f"unsupported drawing shape {shape}")


def position_offset(position: BoardPosition, *, size_m: float, horizontal_half_m: float, vertical_half_m: float) -> tuple[float, float]:
    """Return a safe semantic placement offset within calibrated board bounds."""

    margin = 0.025
    max_horizontal = horizontal_half_m - size_m / 2.0 - margin
    max_vertical = vertical_half_m - size_m / 2.0 - margin
    if max_horizontal < 0.0 or max_vertical < 0.0:
        raise DrawingIntentError("drawing size does not fit the calibrated board")
    if position is BoardPosition.LEFT:
        return (-max_horizontal, 0.0)
    if position is BoardPosition.RIGHT:
        return (max_horizontal, 0.0)
    if position is BoardPosition.TOP:
        return (0.0, max_vertical)
    if position is BoardPosition.BOTTOM:
        return (0.0, -max_vertical)
    return (0.0, 0.0)


def generate_local_geometry(
    intent: DrawingIntent,
    *,
    samples: int = 49,
    horizontal_half_m: float = 0.35,
    vertical_half_m: float = 0.225,
) -> list[BoardPoint]:
    """Generate a new metric curve in board-local coordinates for every call."""

    raw = _normalize(_raw_shape(intent.shape, samples), intent.size_m)
    offset_u, offset_v = position_offset(
        intent.position,
        size_m=intent.size_m,
        horizontal_half_m=horizontal_half_m,
        vertical_half_m=vertical_half_m,
    )
    cosine, sine = cos(intent.orientation_rad), sin(intent.orientation_rad)
    result = []
    for horizontal, vertical in raw:
        rotated_u = horizontal * cosine - vertical * sine + offset_u
        rotated_v = horizontal * sine + vertical * cosine + offset_v
        if abs(rotated_u) > horizontal_half_m or abs(rotated_v) > vertical_half_m:
            raise DrawingIntentError("rotated drawing exceeds calibrated board bounds")
        result.append(BoardPoint(0.0, rotated_u, rotated_v))
    return result


def extent(points: Iterable[BoardPoint]) -> tuple[float, float]:
    values = list(points)
    return (
        max(point.horizontal_m for point in values) - min(point.horizontal_m for point in values),
        max(point.vertical_m for point in values) - min(point.vertical_m for point in values),
    )
