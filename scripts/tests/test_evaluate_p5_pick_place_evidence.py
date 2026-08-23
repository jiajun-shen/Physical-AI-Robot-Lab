"""Pure-Python checks for the standalone Phase-5 evidence evaluator."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "evaluate_p5_pick_place_evidence.py"
SPEC = importlib.util.spec_from_file_location("evaluate_p5_pick_place_evidence", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
EVALUATOR = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = EVALUATOR
SPEC.loader.exec_module(EVALUATOR)


P4_CLASSES = (
    "green_ball",
    "red_ball",
    "blue_cube",
    "yellow_cube",
    "drawing_board",
)
STATIC_MARKER_CLASSES = (
    "p5_orange_top_fiducial",
    "p5_violet_top_fiducial",
)
STATIC_IDS = [f"p5_vision_static_{marker}" for marker in STATIC_MARKER_CLASSES]


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _header(stamp_ns: int, frame_id: str) -> dict[str, int | str]:
    return {
        "stamp_sec": stamp_ns // 1_000_000_000,
        "stamp_nanosec": stamp_ns % 1_000_000_000,
        "stamp_ns": stamp_ns,
        "frame_id": frame_id,
    }


def _pose(
    class_id: str,
    detection_id: str,
    position: list[float],
    stamp_ns: int,
    frame_id: str,
) -> dict[str, object]:
    return {
        "header": _header(stamp_ns, frame_id),
        "source_header": _header(stamp_ns, "lab_camera_optical_frame"),
        "detection_id": detection_id,
        "class_id": class_id,
        "position_m": position,
        "confidence": 0.95,
        "depth_m": 1.0,
        "depth_mad_m": 0.002,
        "valid_depth_samples": 32,
        "orientation_valid": False,
        "source_mode": 1,
    }


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True), encoding="utf-8")


def _write_p4(root: Path, stamp_ns: int, positions: dict[str, list[float]]) -> Path:
    evidence = root / "evidence"
    evidence.mkdir(parents=True, exist_ok=True)
    detections = []
    camera_poses = []
    world_poses = []
    for index, class_id in enumerate(P4_CLASSES):
        detection_id = f"{class_id}-{stamp_ns}-{index}"
        detections.append(
            {
                "header": _header(stamp_ns, "lab_camera_optical_frame"),
                "detection_id": detection_id,
                "class_id": class_id,
                "confidence": 0.95,
                "source_mode": 1,
            }
        )
        camera_poses.append(
            _pose(class_id, detection_id, positions[class_id], stamp_ns, "lab_camera_optical_frame")
        )
        world_poses.append(_pose(class_id, detection_id, positions[class_id], stamp_ns, "world"))
    _write_json(
        evidence / "vision_outputs.json",
        {"detections": detections, "camera_poses": camera_poses, "world_poses": world_poses},
    )
    for name, contents in (
        ("rgb.png", b"rgb"),
        ("annotated_rgb.png", b"annotated"),
        ("depth.npy", b"depth"),
        ("camera_info.json", b"{}"),
        ("tf_chain.json", b"{}"),
    ):
        (evidence / name).write_bytes(contents)
    artifact_hashes = {
        f"evidence/{name}": _sha256(evidence / name)
        for name in (
            "rgb.png",
            "annotated_rgb.png",
            "depth.npy",
            "camera_info.json",
            "tf_chain.json",
            "vision_outputs.json",
        )
    }
    evidence_path = evidence / "vision_evidence.json"
    _write_json(
        evidence_path,
        {
            "schema_version": 1,
            "scope": "SIMULATION_ONLY",
            "sealed_at_utc": "2026-08-15T00:00:00+00:00",
            "perception_mode": "vision",
            "ground_truth_input_used": False,
            "expected_classes": list(P4_CLASSES),
            "source_header": _header(stamp_ns, "lab_camera_optical_frame"),
            "minimum_source_stamp_ns": stamp_ns - 1,
            "artifact_sha256": artifact_hashes,
            "status": {
                "p4_rgb_transport": "PASS",
                "p4_depth_transport": "PASS",
                "p4_camera_info": "PASS",
                "p4_tf_chain": "PASS",
                "p4_all_five_typed_outputs": "PASS",
                "p4_vision_input_boundary": "PASS",
            },
        },
    )
    return evidence_path


def _runtime_pose(
    class_id: str, detection_id: str, position: list[float], stamp_ns: int, confidence: float = 0.95
) -> dict[str, object]:
    return {
        "class_id": class_id,
        "detection_id": detection_id,
        "position_m": position,
        "source_stamp_ns": stamp_ns,
        "confidence": confidence,
        "source_mode": "VISION",
    }


def _action(
    target: float, observed: float, sim_time_ns: int, *, contact_close: bool = False
) -> dict[str, object]:
    observation: dict[str, object] = {
        "target_position_m": target,
        "observed_position_m": observed,
        "sim_time_ns": sim_time_ns,
        "wall_time_utc": "2026-08-15T00:00:00+00:00",
    }
    if contact_close:
        observation.update(
            {
                "minimum_contact_position_m": 0.020,
                "endpoint_observation_semantics": "bounded_contact_close_joint_state",
            }
        )
    return {
        "action_status": 4,
        "action_goal_accepted": True,
        "terminal_status_semantics": (
            "controller_succeeded_with_observed_contact_range"
            if contact_close
            else "controller_succeeded_with_observed_target"
        ),
        "result_position_m": observed,
        "result_effort": "UNTRUSTED_NOT_USED",
        "result_stalled": False,
        "result_reached_goal": True,
        "observation": observation,
    }


def _tracking(position: list[float], stamp_ns: int) -> dict[str, object]:
    tcp = [position[0] - 0.03, position[1], position[2]]
    return {
        "vision": _runtime_pose("green_ball", "green_ball-5000000000-0", position, stamp_ns),
        "tcp": {"position_m": tcp, "tf_stamp_ns": stamp_ns},
        "green_minus_tcp_m": [0.03, 0.0, 0.0],
        "tf_alignment": "tcp_lookup_at_vision_source_stamp",
        "sampled_at": {
            "sim_time_ns": stamp_ns,
            "wall_time_utc": "2026-08-15T00:00:00+00:00",
        },
        "tcp_relative_drift_m": 0.0,
    }


def _motion(label: str, offset: int) -> dict[str, object]:
    return {
        "label": label,
        "collision_aware": True,
        "target": {
            "frame_id": "world",
            "position_m": [0.4 + offset * 0.01, 0.0, 0.9],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
        "planning": {
            "value": 1,
            "waypoint_count": 2,
            "retiming": {"method": "velocity_only_conservative_retime"},
        },
        "execution": {"value": 1, "action_status": 4},
        "observed_endpoint": {f"joint_{index}": float(index) for index in range(1, 7)},
        "planning_scene_static_obstacle_ids": list(STATIC_IDS),
    }


def _static_obstacle_records(pre_stamp: int) -> list[dict[str, object]]:
    records: list[dict[str, object]] = []
    for index, (marker_class, position) in enumerate(
        (
            ("p5_orange_top_fiducial", [0.55, 0.15, 0.954]),
            ("p5_violet_top_fiducial", [0.80, -0.15, 0.954]),
        )
    ):
        marker = _runtime_pose(
            marker_class,
            f"{marker_class}-{pre_stamp + 1_000_000_000}-{index}",
            position,
            pre_stamp + 1_000_000_000,
        )
        records.append(
            {
                "object_id": f"p5_vision_static_{marker_class}",
                "marker": marker,
                "marker_top_to_box_center_z_offset_m": 0.104,
                "collision_box_center_m": [position[0], position[1], 0.85],
                "collision_box_size_xyz_m": [0.160, 0.160, 0.220],
                "source": "fresh_phase5_rgbd_static_obstacle_localizer",
            }
        )
    return records


def _write_execution(path: Path, pre_stamp: int) -> None:
    green = _runtime_pose("green_ball", "green_ball-5000000000-0", [0.45, 0.0, 0.785], pre_stamp)
    red = _runtime_pose("red_ball", "red_ball-5000000000-1", [0.75, 0.0, 0.785], pre_stamp)
    planned_green = _runtime_pose("green_ball", "planned_from_red_vision", [0.65, 0.0, 0.785], pre_stamp, 1.0)
    close_guard = _tracking([0.45, 0.0, 0.785], 6_000_000_000)
    close_guard.pop("tcp_relative_drift_m")
    lift = [_tracking([0.45, 0.0, 0.85], 9_000_000_000)]
    transport = [
        _tracking([0.50, 0.0, 0.85], 10_000_000_000),
        _tracking([0.58, 0.0, 0.85], 11_000_000_000),
        _tracking([0.65, 0.0, 0.85], 12_000_000_000),
    ]
    lower = [_tracking([0.65, 0.0, 0.785], 15_000_000_000)]
    lift[0]["retention_baseline"] = "post_attachment_vision_tcp_sample"
    static_obstacles = _static_obstacle_records(pre_stamp)
    motions = [_motion(label, index) for index, label in enumerate(EVALUATOR.MOTION_LABELS)]
    open_action = _action(0.040, 0.040, 3)
    close_action = _action(0.037, 0.037, 5, contact_close=True)
    release_action = _action(0.040, 0.040, 9)
    attached_state = {
        "state": "attached",
        "sim_time_ns": 5,
        "wall_time_utc": "2026-08-15T00:00:00+00:00",
    }
    detached_state = {
        "state": "detached",
        "sim_time_ns": 9,
        "wall_time_utc": "2026-08-15T00:00:00+00:00",
    }
    attachment = {
        "guard": {
            "fresh_vision_tcp_sample": close_guard,
            "vision_sample_timing": "visual_checkpoint_tf_aligned_before_observed_close",
            "tcp_distance_m": 0.03,
            "observed_close_position_m": 0.037,
            "force_or_effort_used": False,
        },
        "activation": {"activation": "dynamic_entity_system_add_after_vision_tcp_close_guard"},
        "attached_state": attached_state,
        "planning_scene_object_ids": ["p5_green_attached"],
    }
    release = {
        "gripper_open": release_action,
        "detached_state": detached_state,
        "planning_scene_object_ids": ["p5_vision_green_ball"],
        "post_place_minimum_source_stamp_ns": 16_000_000_000,
        "support_proof": "PENDING_FRESH_POST_RELEASE_RGBD_EVALUATION",
    }
    details = {
        "OBSERVE_PICK": {
            "vision_only": True,
            "pre_pick_source_stamp_ns": pre_stamp,
            "green": green,
            "red": red,
            "planning_scene_object_ids": ["p5_work_table", *STATIC_IDS],
            "static_obstacle_vision": static_obstacles,
            "ground_truth_input_used": False,
        },
        "PLAN_APPROACH": {"planned_pregrasp": motions[0]},
        "OPEN": open_action,
        "APPROACH": {
            "visual_checkpoint": motions[1],
            "visual_checkpoint_guard": close_guard,
            "unobserved_final_approach_m": 0.05,
            "planned_grasp": motions[2],
        },
        "CLOSE": close_action,
        "LIFT_VERIFY": {
            "motion": motions[3],
            "attachment": attachment,
            "vision_retention_samples": lift,
            "lift_delta_m": 0.065,
            "minimum_lift_m": 0.05,
        },
        "TRANSPORT": {
            "motion": motions[4],
            "vision_retention_samples": transport,
            "transport_planar_distance_m": 0.20,
            "attachment_state": "attached",
        },
        "LOWER": {"motion": motions[5], "vision_retention_samples": lower},
        "RELEASE": release,
        "RETREAT": {
            "vertical_clearance_motion": motions[6],
            "motion": motions[7],
            "gripper_is_open": release_action["observation"],
            "attachment_state": "detached",
            "retreat_target_policy": "p5_safe_startup_tcp_for_unobstructed_phase4_post_place_vision",
            "released_target_collision_exemption": {
                "object_id": "p5_vision_green_ball",
                "scope": "open_finger_contact_clearance_after_recorded_detach_only",
                "planning_scene_object_ids": ["p5_work_table", *STATIC_IDS],
            },
        },
    }
    stage_records = [
        {
            "stage": stage,
            "status": "PASS",
            "details": details[stage],
            "sim_time_ns": index + 1,
            "wall_time_utc": "2026-08-15T00:00:00+00:00",
        }
        for index, stage in enumerate(EVALUATOR.RUNTIME_STAGES)
    ]
    _write_json(
        path,
        {
            "schema_version": 1,
            "scope": "SIMULATION_ONLY",
            "seed": 42,
            "online_ground_truth_input_used": False,
            "pre_pick_evidence": {
                "source_stamp_ns": pre_stamp,
                "green": green,
                "red": red,
                "sealed_artifact": "external_phase4_vision_evidence",
            },
            "required_stages_executed": list(EVALUATOR.RUNTIME_STAGES),
            "stage_records": stage_records,
            "motion_records": motions,
            "attachment_states": [attached_state, detached_state],
            "attachment": attachment,
            "retention": {
                "baseline": close_guard,
                "lift_samples": lift,
                "transport_samples": transport,
                "lower_samples": lower,
                "lift_delta_m": 0.065,
                "transport_planar_distance_m": 0.20,
            },
            "release": release,
            "post_place": {
                "minimum_source_stamp_ns": 16_000_000_000,
                "planned_green": planned_green,
                "red_pre_pick": red,
                "evaluation_required": "fresh_vision_only_post_place_evidence",
            },
            "status": "PENDING_POST_PLACE_VISION_EVALUATION",
            "finished_at_utc": "2026-08-15T00:00:01+00:00",
        },
    )


class EvaluateP5PickPlaceEvidenceTests(unittest.TestCase):
    def _fixture(self) -> tuple[tempfile.TemporaryDirectory[str], Path, Path, Path]:
        directory = tempfile.TemporaryDirectory()
        base = Path(directory.name)
        positions_pre = {
            "green_ball": [0.45, 0.0, 0.785],
            "red_ball": [0.75, 0.0, 0.785],
            "blue_cube": [0.3, -0.2, 0.78],
            "yellow_cube": [0.9, 0.2, 0.78],
            "drawing_board": [1.1, 0.0, 0.975],
        }
        positions_post = {**positions_pre, "green_ball": [0.65, 0.0, 0.785]}
        pre = _write_p4(base / "pre", 5_000_000_000, positions_pre)
        post = _write_p4(base / "post", 17_000_000_000, positions_post)
        execution = base / "execution_evidence.json"
        _write_execution(execution, 5_000_000_000)
        return directory, pre, execution, post

    def test_complete_current_schema_passes(self) -> None:
        directory, pre, execution, post = self._fixture()
        self.addCleanup(directory.cleanup)
        result = EVALUATOR.evaluate(pre, execution, post)
        self.assertEqual(result["status"], "PASS")
        self.assertEqual([item["stage"] for item in result["stage_records"]], list(EVALUATOR.REQUIRED_STAGES))
        self.assertAlmostEqual(result["measurements"]["final_green_red_planar_distance_m"], 0.10)

    def test_bounded_stalled_contact_close_still_requires_visual_retention(self) -> None:
        directory, pre, execution, post = self._fixture()
        self.addCleanup(directory.cleanup)
        payload = json.loads(execution.read_text(encoding="utf-8"))
        close = payload["stage_records"][4]["details"]
        close.update(
            {
                "action_status": 6,
                "terminal_status_semantics": "controller_stalled_with_observed_contact_range",
                "result_position_m": 0.027,
                "result_stalled": True,
                "result_reached_goal": False,
            }
        )
        close["observation"]["observed_position_m"] = 0.027
        payload["attachment"]["guard"]["observed_close_position_m"] = 0.027
        payload["stage_records"][5]["details"]["attachment"]["guard"][
            "observed_close_position_m"
        ] = 0.027
        _write_json(execution, payload)
        self.assertEqual(EVALUATOR.evaluate(pre, execution, post)["status"], "PASS")

    def test_contact_close_below_calibrated_minimum_fails_closed(self) -> None:
        directory, pre, execution, post = self._fixture()
        self.addCleanup(directory.cleanup)
        payload = json.loads(execution.read_text(encoding="utf-8"))
        close = payload["stage_records"][4]["details"]
        close["observation"]["minimum_contact_position_m"] = 0.019
        _write_json(execution, payload)
        with self.assertRaisesRegex(EVALUATOR.EvaluationError, "contact-close range"):
            EVALUATOR.evaluate(pre, execution, post)

    def test_tampered_phase4_artifact_fails_and_writes_fail_output(self) -> None:
        directory, pre, execution, post = self._fixture()
        self.addCleanup(directory.cleanup)
        (pre.parent / "rgb.png").write_bytes(b"tampered")
        output = Path(directory.name) / "evaluation.json"
        code = EVALUATOR.main(
            [
                "--pre-pick-evidence",
                str(pre),
                "--execution-evidence",
                str(execution),
                "--post-place-evidence",
                str(post),
                "--output",
                str(output),
            ]
        )
        self.assertEqual(code, 1)
        self.assertEqual(json.loads(output.read_text(encoding="utf-8"))["status"], "FAIL")

    def test_stale_post_place_vision_fails_closed(self) -> None:
        directory, pre, execution, post = self._fixture()
        self.addCleanup(directory.cleanup)
        payload = json.loads(execution.read_text(encoding="utf-8"))
        payload["post_place"]["minimum_source_stamp_ns"] = 17_000_000_000
        payload["release"]["post_place_minimum_source_stamp_ns"] = 17_000_000_000
        payload["stage_records"][8]["details"]["post_place_minimum_source_stamp_ns"] = 17_000_000_000
        _write_json(execution, payload)
        with self.assertRaisesRegex(EVALUATOR.EvaluationError, "not fresh after release"):
            EVALUATOR.evaluate(pre, execution, post)

    def test_non_collision_aware_motion_fails_closed(self) -> None:
        directory, pre, execution, post = self._fixture()
        self.addCleanup(directory.cleanup)
        payload = json.loads(execution.read_text(encoding="utf-8"))
        payload["motion_records"][0]["collision_aware"] = False
        payload["stage_records"][1]["details"]["planned_pregrasp"]["collision_aware"] = False
        _write_json(execution, payload)
        with self.assertRaisesRegex(EVALUATOR.EvaluationError, "not collision-aware"):
            EVALUATOR.evaluate(pre, execution, post)

    def test_missing_static_obstacle_from_motion_scene_fails_closed(self) -> None:
        directory, pre, execution, post = self._fixture()
        self.addCleanup(directory.cleanup)
        payload = json.loads(execution.read_text(encoding="utf-8"))
        payload["motion_records"][0]["planning_scene_static_obstacle_ids"] = [STATIC_IDS[0]]
        payload["stage_records"][1]["details"]["planned_pregrasp"][
            "planning_scene_static_obstacle_ids"
        ] = [STATIC_IDS[0]]
        _write_json(execution, payload)
        with self.assertRaisesRegex(EVALUATOR.EvaluationError, "static RGB-D obstacles"):
            EVALUATOR.evaluate(pre, execution, post)


if __name__ == "__main__":
    unittest.main()
