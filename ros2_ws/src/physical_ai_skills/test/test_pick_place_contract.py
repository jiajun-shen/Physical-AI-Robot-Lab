import pytest

from physical_ai_skills.pick_place_contract import (
    PickPlaceEvidence, Pose3, REQUIRED_STAGES, episode_passes, validate_pick_place,
)


def _good():
    return PickPlaceEvidence(
        stages=REQUIRED_STAGES,
        pick_green=Pose3(0.50, 0.10, 0.05),
        lifted_green=Pose3(0.50, 0.10, 0.12),
        placed_green=Pose3(0.62, 0.20, 0.05),
        placed_red=Pose3(0.72, 0.20, 0.05),
        planning_collision_aware=True,
        vision_only_before=True,
        vision_only_after=True,
        object_retained_during_transport=True,
        released_on_support=True,
    )


def test_self_attested_dataclass_can_never_claim_phase5_acceptance():
    with pytest.raises(RuntimeError, match="not Phase-5 acceptance"):
        episode_passes(_good())


def test_finger_animation_without_lift_cannot_pass():
    e = _good()
    e = PickPlaceEvidence(**{**e.__dict__, "lifted_green": Pose3(0.50, 0.10, 0.06)})
    checks = validate_pick_place(e)
    assert not checks["lift_proves_retention"]
    with pytest.raises(RuntimeError, match="not Phase-5 acceptance"):
        episode_passes(e)


def test_ground_truth_decision_boundary_cannot_pass():
    e = _good()
    e = PickPlaceEvidence(**{**e.__dict__, "vision_only_after": False})
    with pytest.raises(RuntimeError, match="not Phase-5 acceptance"):
        episode_passes(e)
