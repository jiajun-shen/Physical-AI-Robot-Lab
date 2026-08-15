"""Machine-checkable Phase-5 pick/place evidence rules.

This module deliberately validates evidence; it does not manufacture a grasp result.
"""

from dataclasses import dataclass
from math import hypot
from typing import Mapping, Sequence

REQUIRED_STAGES = (
    "OBSERVE_PICK",
    "PLAN_APPROACH",
    "OPEN",
    "APPROACH",
    "CLOSE",
    "LIFT_VERIFY",
    "TRANSPORT",
    "LOWER",
    "RELEASE",
    "RETREAT",
    "OBSERVE_PLACE",
    "VERIFY_PLACE",
)

MIN_LIFT_M = 0.05
MAX_BESIDE_DISTANCE_M = 0.12
MIN_NON_OVERLAP_DISTANCE_M = 0.075


@dataclass(frozen=True)
class Pose3:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class PickPlaceEvidence:
    stages: Sequence[str]
    pick_green: Pose3
    lifted_green: Pose3
    placed_green: Pose3
    placed_red: Pose3
    planning_collision_aware: bool
    vision_only_before: bool
    vision_only_after: bool
    object_retained_during_transport: bool
    released_on_support: bool


def validate_pick_place(e: PickPlaceEvidence) -> Mapping[str, bool]:
    """Return named acceptance checks without hiding a failed condition."""
    stages_exact = tuple(e.stages) == REQUIRED_STAGES
    lift_delta = e.lifted_green.z - e.pick_green.z
    beside = hypot(e.placed_green.x - e.placed_red.x, e.placed_green.y - e.placed_red.y)
    return {
        "stage_sequence": stages_exact,
        "collision_aware_planning": e.planning_collision_aware,
        "fresh_vision_boundary": e.vision_only_before and e.vision_only_after,
        "lift_proves_retention": lift_delta >= MIN_LIFT_M,
        "transport_retention": e.object_retained_during_transport,
        "released_on_support": e.released_on_support,
        "beside_threshold": beside <= MAX_BESIDE_DISTANCE_M,
        "non_overlap": beside >= MIN_NON_OVERLAP_DISTANCE_M,
    }


def episode_passes(e: PickPlaceEvidence) -> bool:
    return all(validate_pick_place(e).values())
