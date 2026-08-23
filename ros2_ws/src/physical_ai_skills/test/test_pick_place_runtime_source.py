"""Source contracts for the Phase-5 runtime acceptance boundary."""

from __future__ import annotations

import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RUNTIME_PATH = PACKAGE_ROOT / "physical_ai_skills" / "pick_place_runtime.py"
RUNTIME_CONFIG_PATH = PACKAGE_ROOT / "config" / "verified_pick_place.yaml"
LEGACY_CONTRACT_PATH = PACKAGE_ROOT / "physical_ai_skills" / "pick_place_contract.py"


def test_runtime_is_parseable_and_has_the_exact_online_state_prefix() -> None:
    source = RUNTIME_PATH.read_text(encoding="utf-8")
    ast.parse(source, filename=str(RUNTIME_PATH))
    assert '"OBSERVE_PICK"' in source
    assert '"RETREAT"' in source
    assert 'PENDING_POST_PLACE_VISION_EVALUATION' in source
    assert 'fresh_vision_only_post_place_evidence' in source


def test_online_runtime_has_no_gazebo_object_pose_or_seed_manifest_path() -> None:
    source = RUNTIME_PATH.read_text(encoding="utf-8").lower()
    for forbidden in (
        "gz model -p",
        "gz topic",
        "model_pose",
        "seed_manifest",
        "ground_truth_service",
    ):
        assert forbidden not in source
    # The one permitted Gazebo command resolves only the robot model identity
    # before dynamically adding the guarded simulation-only retention system.
    assert '["gz", "model", "-m", model_name]' in source
    assert 'dynamic_entity_system_add_after_vision_tcp_close_guard' in source


def test_runtime_requires_two_fresh_rgbd_static_obstacles_for_every_motion() -> None:
    source = RUNTIME_PATH.read_text(encoding="utf-8")
    assert "wait_fresh_static_obstacles" in source
    assert "fresh_phase5_rgbd_static_obstacle_localizer" in source
    assert "require_static_obstacles_in_scene" in source
    assert '"planning_scene_static_obstacle_ids": static_scene_ids' in source
    assert "MoveIt planning scene does not contain every fresh RGB-D static obstacle" in source


def test_descending_side_grasp_config_and_legacy_contract_are_fail_closed() -> None:
    config = yaml.safe_load(RUNTIME_CONFIG_PATH.read_text(encoding="utf-8"))
    assert config["scope"] == "SIMULATION_ONLY"
    candidates = config["robot"]["side_grasp_candidates"]
    assert [candidate["id"] for candidate in candidates] == [
        "y_plus_down_45_roll_180",
        "y_minus_down_45_roll_180",
        "y_plus_down_45_roll_90",
        "y_plus_down_25_roll_180",
        "northwest_down_45_roll_90",
        "y_minus_down_45_roll_270",
        "y_minus_down_25_roll_270",
    ]
    assert all(len(candidate["orientation_xyzw"]) == 4 for candidate in candidates)
    assert "top_down_orientation_xyzw" not in config["robot"]
    source = RUNTIME_PATH.read_text(encoding="utf-8")
    assert "fresh_rgbd_scene_live_moveit_side_grasp_pair_probe" in source
    assert "REJECTED_BY_COLLISION_AWARE_MOVEIT" in source
    assert "nonvertical and descending" in source
    assert '"direction": list(direction)' in source
    assert '"grasp_tcp": list(grasp_tcp)' in source
    assert "destination.update(observed)" in source
    assert '"/phase5/perception/guard_world_object_poses"' in source
    assert '"/phase5/perception/transport_world_object_poses"' in source
    assert '"/phase5/perception/lift_world_object_poses"' in source
    assert "guard_only=True" in source
    assert '"transport_only": True' in source
    assert '"lift_only": True' in source
    assert "visual_checkpoint_backoff_m" in source
    assert '"visual checkpoint guard"' in source
    assert '"visual_checkpoint"' in source
    assert "tcp_lookup_at_vision_source_stamp" in source
    assert "visual_checkpoint_tf_aligned_before_observed_close" in source
    assert "fresh_proximate_vision_tcp_sample" in source
    assert "post_attachment_vision_tcp_sample" in source
    assert 'self.collect_retention(' in source
    assert 'self.motion.get("lift_retention_samples")' in source
    assert "last transport green/TCP relative offset" in source
    assert "pre_release_attached_settle_sec" in source
    assert '"post_release_clearance"' in source
    assert "vertical_clearance_motion" in source
    assert '"primary_rgbd_fallback_after_guard_rejection"' in source
    assert '"proximate_rgbd_fallback_after_guard_primary_rejection"' in source
    assert "guard TCP-relative drift" in source
    assert "start_positions" in source
    assert config["attachment"]["dynamic_plugin_name"] == "gz::sim::systems::DetachableJoint"
    legacy = LEGACY_CONTRACT_PATH.read_text(encoding="utf-8")
    assert "legacy checklist, not Phase-5 acceptance" in legacy
    assert "raise RuntimeError" in legacy


def test_gripper_endpoint_requires_joint_state_observation_not_action_state_shape() -> None:
    source = RUNTIME_PATH.read_text(encoding="utf-8")
    assert "self.observe_gripper(target, label)" in source
    assert "observe_gripper_contact_close" in source
    assert "bounded_contact_close_joint_state" in source
    assert "controller_stalled_with_observed_contact_range" in source
    assert "joint_states_endpoint_observation" in source
    assert "unnamed_controller_state_informational" in source
    assert "if state_names:" in source
    config = yaml.safe_load(RUNTIME_CONFIG_PATH.read_text(encoding="utf-8"))
    assert config["gripper"]["minimum_contact_position_m"] == 0.020
    assert config["motion"]["pre_release_attached_settle_sec"] == 1.0
