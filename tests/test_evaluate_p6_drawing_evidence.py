import importlib.util
import json
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "evaluate_p6_drawing_evidence.py"
SPEC = importlib.util.spec_from_file_location("evaluate_p6_drawing_evidence", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


STATIC_MARKERS = (
    "p5_orange_top_fiducial",
    "p5_violet_top_fiducial",
)
STATIC_IDS = tuple(f"p6_vision_static_{marker}" for marker in STATIC_MARKERS)


def _scene() -> list[str]:
    return ["p6_work_table", "p6_live_vision_drawing_board", *STATIC_IDS]


def _static_obstacles() -> dict:
    records = []
    for index, class_id in enumerate(STATIC_MARKERS):
        marker_position = [0.8 - index * 0.3, 0.2, 0.955]
        records.append(
            {
                "object_id": f"p6_vision_static_{class_id}",
                "source": "fresh_phase5_rgbd_static_obstacle_localizer",
                "marker": {
                    "class_id": class_id,
                    "source_mode": "VISION",
                    "frame_id": "world",
                    "source_stamp_ns": 200,
                    "confidence": 0.7,
                    "detection_id": f"{class_id}-200",
                    "position_m": marker_position,
                },
                "marker_top_to_box_center_z_offset_m": 0.104,
                "collision_box_center_m": [marker_position[0], marker_position[1], 0.851],
                "collision_box_size_xyz_m": [0.160, 0.160, 0.220],
            }
        )
    return {
        "source_topic": "/phase5/perception/static_obstacle_poses",
        "required_marker_class_ids": list(STATIC_MARKERS),
        "records": records,
    }


def _evidence(shape: str, size: float, position: str = "center") -> dict:
    scene = _scene()
    return {
        "schema_version": 1,
        "scope": "SIMULATION_ONLY_CONTACTLESS_DRAWING_TRACE",
        "online_ground_truth_input_used": False,
        "status": {"P6_DRAWING_RUNTIME": "PASS"},
        "drawing_intent": {"shape": shape, "size_m": size, "position": position},
        "geometry": {
            "generated_at_runtime": True,
            "fixed_world_geometry_input": False,
            "coordinate_chain": "drawing-local -> drawing-board frame -> world -> MoveIt Cartesian waypoints -> ABB execution",
            "local_points": [{"normal_m": 0.0, "horizontal_m": 0.0, "vertical_m": 0.0}] * 16,
            "world_waypoints_m": [[1.077, 0.0, 0.975]] * 16,
        },
        "board_frame": {
            "name": "drawing_board",
            "observation_reference": "robot_facing_front_surface",
            "center_source": "fresh_rgbd_front_surface",
            "observation": {
                "source_mode": "VISION",
                "frame_id": "world",
                "source_stamp_ns": 100,
                "detection_id": "drawing-board-100",
                "position_m": [1.085, 0.0, 0.975],
            },
            "front_surface_world_m": [1.085, 0.0, 0.975],
            "derived_center_world_m": [1.10, 0.0, 0.975],
            "collision_center_world_m": [1.10, 0.0, 0.975],
            "normal_world": [-1.0, 0.0, 0.0],
            "horizontal_world": [0.0, 1.0, 0.0],
            "vertical_world": [0.0, 0.0, 1.0],
            "front_surface_offset_m": 0.015,
            "virtual_tip_standoff_m": 0.008,
            "board_collision_preserved": True,
            "tool_link": "drawing_tip",
            "tool_model": "phase6_collision_bearing_drawing_pen",
        },
        "planning_scene_ids": scene,
        "static_collision_obstacles": _static_obstacles(),
        "execution": {
            "orientation_xyzw": [1.0, 0.0, 0.0, 0.0],
            "pre_motion_cartesian_gate": {
                "result": "PASS",
                "segment_count": 15,
                "collision_aware": True,
                "avoid_collisions": True,
                "start_state": "planned_approach_then_entry_endpoint",
                "execution_route": "preflighted_live_entry_then_cartesian_trajectories",
                "joint_limit_margin": {
                    "minimum_required_margin_rad": 0.20,
                    "route_minimum_margin_rad": 0.50,
                    "closest_joint": "joint_5",
                    "closest_trajectory": "draw_segment_00",
                    "closest_trajectory_point_index": 0,
                    "by_joint_minimum_margin_rad": {
                        f"joint_{index}": 0.50 for index in range(1, 7)
                    },
                },
            },
            "entry_alignment": {
                "avoid_collisions": True,
                "fraction": 1.0,
                "requested_waypoint_count": 1,
                "planning_scene_ids": scene,
                "pre_motion_cartesian": {
                    "avoid_collisions": True,
                    "fraction": 1.0,
                    "requested_waypoint_count": 1,
                    "planning_scene_ids": scene,
                },
                "live_joint_limit_margin": {
                    "minimum_required_margin_rad": 0.20,
                    "route_minimum_margin_rad": 0.50,
                },
                "target_provenance": {
                    "strategy": "live_state_to_runtime_generated_first_waypoint",
                    "target_count": 1,
                },
                "expected_endpoint": {f"joint_{index}": 0.0 for index in range(1, 7)},
                "observed_endpoint": {f"joint_{index}": 0.025 for index in range(1, 7)},
                "endpoint_gate": {"tolerance_rad": 0.03, "consecutive_samples": 3},
                "retiming": {
                    "profile": "entry",
                    "duration_sec": 27.0,
                    "terminal_hold": {
                        "duration_sec": 1.0,
                        "repeats_collision_checked_endpoint": True,
                    },
                },
                "tcp_gate": {
                    "consecutive_samples": 3,
                    "expected_position_m": [1.077, 0.0, 0.975],
                    "expected_orientation_xyzw": [1.0, 0.0, 0.0, 0.0],
                    "position_tolerance_m": 0.008,
                    "orientation_tolerance_rad": 0.08,
                    "observed": {
                        "position_m": [1.077, 0.0, 0.975],
                        "orientation_xyzw": [1.0, 0.0, 0.0, 0.0],
                        "position_error_m": 0.0,
                        "orientation_error_rad": 0.0,
                    },
                },
                "execution": {"action_status": 4},
            },
            "cartesian": {
                "avoid_collisions": True,
                "fraction": 1.0,
                "execution": {"action_status": 4},
                "segments": [
                    {
                        "avoid_collisions": True,
                        "fraction": 1.0,
                        "planning_scene_ids": scene,
                        "pre_motion_cartesian": {"avoid_collisions": True, "fraction": 1.0, "planning_scene_ids": scene},
                        "live_joint_limit_margin": {
                            "minimum_required_margin_rad": 0.20,
                            "route_minimum_margin_rad": 0.50,
                        },
                        "live_execution_targets": {
                            "strategy": "live_state_to_forward_generated_waypoints",
                            "preflight_waypoint_count": 2,
                            "live_forward_target_count": 1,
                        },
                        "expected_endpoint": {f"joint_{index}": 0.0 for index in range(1, 7)},
                        "observed_endpoint": {f"joint_{index}": 0.025 for index in range(1, 7)},
                        "endpoint_gate": {"tolerance_rad": 0.03, "consecutive_samples": 3},
                        "retiming": {
                            "terminal_hold": {
                                "duration_sec": 1.0,
                                "repeats_collision_checked_endpoint": True,
                            }
                        },
                        "execution": {"action_status": 4},
                    }
                ],
            },
            "approach": {"planning": {"collision_aware": True, "planning_scene_ids": scene}, "execution": {"action_status": 4}},
            "retreat": {"planning": {"collision_aware": True, "planning_scene_ids": scene}, "execution": {"action_status": 4}},
        },
        "tcp_trace": [{"position_m": [1.0, 0.0, 1.0]}] * 5,
    }


def test_evaluator_requires_curve_polygon_and_parameter_variation(tmp_path):
    paths = []
    for name, body in (("heart", _evidence("heart", 0.14)), ("square", _evidence("square", 0.14, "left")), ("small_circle", _evidence("circle", 0.10))):
        path = tmp_path / f"{name}.json"
        path.write_text(json.dumps(body), encoding="utf-8")
        paths.append(path)
    result = MODULE.evaluate(paths)
    assert result["P6_DRAWING_EVALUATION"] == "PASS"


def test_evaluator_rejects_cartesian_path_without_collision_checking(tmp_path):
    invalid = _evidence("heart", 0.14)
    invalid["execution"]["cartesian"]["avoid_collisions"] = False
    paths = []
    for index, shape in enumerate(("heart", "square", "circle")):
        path = tmp_path / f"{index}.json"
        body = invalid if index == 0 else _evidence(shape, 0.10 + index * 0.02)
        path.write_text(json.dumps(body), encoding="utf-8")
        paths.append(path)
    try:
        MODULE.evaluate(paths)
    except MODULE.EvaluationError as exc:
        assert "collision" in str(exc)
    else:
        raise AssertionError("evaluator accepted an unguarded Cartesian path")


def test_evaluator_rejects_execution_that_skips_the_pre_motion_route_gate(tmp_path):
    invalid = _evidence("heart", 0.14)
    invalid["execution"].pop("pre_motion_cartesian_gate")
    paths = []
    for index, shape in enumerate(("heart", "square", "circle")):
        path = tmp_path / f"{index}.json"
        body = invalid if index == 0 else _evidence(shape, 0.10 + index * 0.02)
        path.write_text(json.dumps(body), encoding="utf-8")
        paths.append(path)
    try:
        MODULE.evaluate(paths)
    except MODULE.EvaluationError as exc:
        assert "before motion" in str(exc)
    else:
        raise AssertionError("evaluator accepted a route that was not preflighted")


def test_evaluator_rejects_a_live_segment_without_its_pre_motion_check(tmp_path):
    invalid = _evidence("heart", 0.14)
    invalid["execution"]["cartesian"]["segments"][0].pop("pre_motion_cartesian")
    paths = []
    for index, shape in enumerate(("heart", "square", "circle")):
        path = tmp_path / f"{index}.json"
        body = invalid if index == 0 else _evidence(shape, 0.10 + index * 0.02)
        path.write_text(json.dumps(body), encoding="utf-8")
        paths.append(path)
    try:
        MODULE.evaluate(paths)
    except MODULE.EvaluationError as exc:
        assert "pre-motion" in str(exc)
    else:
        raise AssertionError("evaluator accepted a live segment without a pre-motion check")


def test_evaluator_rejects_a_terminal_hold_that_changes_the_endpoint(tmp_path):
    invalid = _evidence("heart", 0.14)
    invalid["execution"]["cartesian"]["segments"][0]["retiming"]["terminal_hold"][
        "repeats_collision_checked_endpoint"
    ] = False
    paths = []
    for index, shape in enumerate(("heart", "square", "circle")):
        path = tmp_path / f"{index}.json"
        body = invalid if index == 0 else _evidence(shape, 0.10 + index * 0.02)
        path.write_text(json.dumps(body), encoding="utf-8")
        paths.append(path)
    try:
        MODULE.evaluate(paths)
    except MODULE.EvaluationError as exc:
        assert "terminal hold" in str(exc)
    else:
        raise AssertionError("evaluator accepted a terminal hold that changed the endpoint")


def test_evaluator_rejects_a_live_segment_that_backtracks_to_a_nominal_endpoint(tmp_path):
    invalid = _evidence("heart", 0.14)
    invalid["execution"]["cartesian"]["segments"][0]["live_execution_targets"][
        "strategy"
    ] = "nominal_previous_then_forward"
    paths = []
    for index, shape in enumerate(("heart", "square", "circle")):
        path = tmp_path / f"{index}.json"
        body = invalid if index == 0 else _evidence(shape, 0.10 + index * 0.02)
        path.write_text(json.dumps(body), encoding="utf-8")
        paths.append(path)
    try:
        MODULE.evaluate(paths)
    except MODULE.EvaluationError as exc:
        assert "forward generated targets" in str(exc)
    else:
        raise AssertionError("evaluator accepted a live segment that backtracked")


def test_evaluator_rejects_an_endpoint_that_exceeds_the_live_gate(tmp_path):
    invalid = _evidence("heart", 0.14)
    invalid["execution"]["cartesian"]["segments"][0]["observed_endpoint"]["joint_3"] = 0.031
    paths = []
    for index, shape in enumerate(("heart", "square", "circle")):
        path = tmp_path / f"{index}.json"
        body = invalid if index == 0 else _evidence(shape, 0.10 + index * 0.02)
        path.write_text(json.dumps(body), encoding="utf-8")
        paths.append(path)
    try:
        MODULE.evaluate(paths)
    except MODULE.EvaluationError as exc:
        assert "exceeds" in str(exc)
    else:
        raise AssertionError("evaluator accepted an endpoint outside the live gate")


def test_evaluator_rejects_a_pre_motion_route_without_joint_limit_margin(tmp_path):
    invalid = _evidence("heart", 0.14)
    invalid["execution"]["pre_motion_cartesian_gate"]["joint_limit_margin"][
        "route_minimum_margin_rad"
    ] = 0.19
    paths = []
    for index, shape in enumerate(("heart", "square", "circle")):
        path = tmp_path / f"{index}.json"
        body = invalid if index == 0 else _evidence(shape, 0.10 + index * 0.02)
        path.write_text(json.dumps(body), encoding="utf-8")
        paths.append(path)
    try:
        MODULE.evaluate(paths)
    except MODULE.EvaluationError as exc:
        assert "joint-limit margin" in str(exc)
    else:
        raise AssertionError("evaluator accepted a route without the hard-limit safety margin")


def test_evaluator_rejects_a_route_without_the_visual_static_collision_scene(tmp_path):
    invalid = _evidence("heart", 0.14)
    invalid["planning_scene_ids"].remove(STATIC_IDS[0])
    paths = []
    for index, shape in enumerate(("heart", "square", "circle")):
        path = tmp_path / f"{index}.json"
        body = invalid if index == 0 else _evidence(shape, 0.10 + index * 0.02)
        path.write_text(json.dumps(body), encoding="utf-8")
        paths.append(path)
    try:
        MODULE.evaluate(paths)
    except MODULE.EvaluationError as exc:
        assert "static collision" in str(exc)
    else:
        raise AssertionError("evaluator accepted a route without both visual static collision boxes")


def test_evaluator_rejects_a_motion_scene_without_the_work_table(tmp_path):
    invalid = _evidence("heart", 0.14)
    invalid["planning_scene_ids"].remove("p6_work_table")
    paths = []
    for index, shape in enumerate(("heart", "square", "circle")):
        path = tmp_path / f"{index}.json"
        body = invalid if index == 0 else _evidence(shape, 0.10 + index * 0.02)
        path.write_text(json.dumps(body), encoding="utf-8")
        paths.append(path)
    try:
        MODULE.evaluate(paths)
    except MODULE.EvaluationError as exc:
        assert "p6_work_table" in str(exc)
    else:
        raise AssertionError("evaluator accepted a collision-aware claim without the work table")


def test_evaluator_rejects_a_curve_that_skips_explicit_tcp_verified_entry(tmp_path):
    invalid = _evidence("heart", 0.14)
    invalid["execution"].pop("entry_alignment")
    paths = []
    for index, shape in enumerate(("heart", "square", "circle")):
        path = tmp_path / f"{index}.json"
        body = invalid if index == 0 else _evidence(shape, 0.10 + index * 0.02)
        path.write_text(json.dumps(body), encoding="utf-8")
        paths.append(path)
    try:
        MODULE.evaluate(paths)
    except MODULE.EvaluationError as exc:
        assert "explicit entry" in str(exc)
    else:
        raise AssertionError("evaluator accepted a curve that skipped entry alignment")


def test_evaluator_rejects_static_obstacles_from_mixed_rgbd_epochs(tmp_path):
    invalid = _evidence("heart", 0.14)
    invalid["static_collision_obstacles"]["records"][1]["marker"]["source_stamp_ns"] = 201
    paths = []
    for index, shape in enumerate(("heart", "square", "circle")):
        path = tmp_path / f"{index}.json"
        body = invalid if index == 0 else _evidence(shape, 0.10 + index * 0.02)
        path.write_text(json.dumps(body), encoding="utf-8")
        paths.append(path)
    try:
        MODULE.evaluate(paths)
    except MODULE.EvaluationError as exc:
        assert "one RGB-D epoch" in str(exc)
    else:
        raise AssertionError("evaluator accepted static obstacles stitched from different RGB-D epochs")


def test_evaluator_rejects_entry_target_that_is_not_the_first_generated_waypoint(tmp_path):
    invalid = _evidence("heart", 0.14)
    invalid["execution"]["entry_alignment"]["tcp_gate"]["expected_position_m"][0] += 0.01
    paths = []
    for index, shape in enumerate(("heart", "square", "circle")):
        path = tmp_path / f"{index}.json"
        body = invalid if index == 0 else _evidence(shape, 0.10 + index * 0.02)
        path.write_text(json.dumps(body), encoding="utf-8")
        paths.append(path)
    try:
        MODULE.evaluate(paths)
    except MODULE.EvaluationError as exc:
        assert "first generated world waypoint" in str(exc)
    else:
        raise AssertionError("evaluator accepted an entry target that was not the first generated waypoint")


def test_evaluator_rejects_world_waypoints_not_mapped_from_local_board_geometry(tmp_path):
    invalid = _evidence("heart", 0.14)
    invalid["geometry"]["world_waypoints_m"][3][1] += 0.01
    paths = []
    for index, shape in enumerate(("heart", "square", "circle")):
        path = tmp_path / f"{index}.json"
        body = invalid if index == 0 else _evidence(shape, 0.10 + index * 0.02)
        path.write_text(json.dumps(body), encoding="utf-8")
        paths.append(path)
    try:
        MODULE.evaluate(paths)
    except MODULE.EvaluationError as exc:
        assert "does not map" in str(exc)
    else:
        raise AssertionError("evaluator accepted fixed or mismatched world geometry")


def test_evaluator_rejects_a_board_surface_detached_from_its_visual_observation(tmp_path):
    invalid = _evidence("heart", 0.14)
    invalid["board_frame"]["front_surface_world_m"][0] += 0.01
    paths = []
    for index, shape in enumerate(("heart", "square", "circle")):
        path = tmp_path / f"{index}.json"
        body = invalid if index == 0 else _evidence(shape, 0.10 + index * 0.02)
        path.write_text(json.dumps(body), encoding="utf-8")
        paths.append(path)
    try:
        MODULE.evaluate(paths)
    except MODULE.EvaluationError as exc:
        assert "differs from its visual observation" in str(exc)
    else:
        raise AssertionError("evaluator accepted a board surface not backed by vision")
