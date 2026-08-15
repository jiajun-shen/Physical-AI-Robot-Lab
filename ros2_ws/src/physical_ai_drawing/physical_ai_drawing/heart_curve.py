"""Generate a normalized, metric Cartesian heart at runtime."""

from dataclasses import dataclass
from math import cos, pi, sin
from typing import List


@dataclass(frozen=True)
class Point3:
    x: float
    y: float
    z: float


def generate_heart(scale_m: float = 0.12, samples: int = 240, board_x: float = 0.0) -> List[Point3]:
    """Return heart points in a board plane x=board_x.

    `scale_m` is the full maximum of the normalized y/z span, not a joint-space multiplier.
    """
    if not (0.02 <= scale_m <= 0.30):
        raise ValueError("scale_m must be within [0.02, 0.30] m")
    if samples < 32:
        raise ValueError("samples must be >= 32")
    raw = []
    for i in range(samples):
        t = 2.0 * pi * i / (samples - 1)
        u = 16.0 * sin(t) ** 3
        v = 13.0 * cos(t) - 5.0 * cos(2*t) - 2.0 * cos(3*t) - cos(4*t)
        raw.append((u, v))
    u_mid = (max(p[0] for p in raw) + min(p[0] for p in raw)) / 2.0
    v_mid = (max(p[1] for p in raw) + min(p[1] for p in raw)) / 2.0
    span = max(max(p[0] for p in raw)-min(p[0] for p in raw), max(p[1] for p in raw)-min(p[1] for p in raw))
    gain = scale_m / span
    return [Point3(board_x, (u-u_mid)*gain, (v-v_mid)*gain) for u, v in raw]


def extent(points: List[Point3]) -> tuple[float, float]:
    return (max(p.y for p in points)-min(p.y for p in points), max(p.z for p in points)-min(p.z for p in points))
