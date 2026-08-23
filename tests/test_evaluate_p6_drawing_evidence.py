import importlib.util
import json
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "evaluate_p6_drawing_evidence.py"
SPEC = importlib.util.spec_from_file_location("evaluate_p6_drawing_evidence", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _evidence(shape: str, size: float, position: str = "center") -> dict:
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
            "world_waypoints_m": [[1.05, 0.0, 0.95]] * 16,
        },
        "board_frame": {"name": "drawing_board", "center_source": "fresh_rgbd_vision", "observation": {"source_mode": "VISION"}, "board_collision_preserved": True, "tool_link": "drawing_tip", "tool_model": "phase6_collision_bearing_drawing_pen"},
        "planning_scene_ids": ["p6_live_vision_drawing_board"],
        "execution": {
            "pre_motion_cartesian_gate": {
                "result": "PASS",
                "segment_count": 15,
                "collision_aware": True,
                "avoid_collisions": True,
                "start_state": "planned_approach_endpoint",
                "execution_route": "preflighted_live_start_cartesian_trajectories",
            },
            "cartesian": {
                "avoid_collisions": True,
                "fraction": 1.0,
                "execution": {"action_status": 4},
                "segments": [
                    {
                        "avoid_collisions": True,
                        "fraction": 1.0,
                        "pre_motion_cartesian": {"avoid_collisions": True, "fraction": 1.0},
                        "live_execution_targets": {
                            "strategy": "live_state_to_forward_generated_waypoints",
                            "preflight_waypoint_count": 2,
                            "live_forward_target_count": 1,
                        },
                        "expected_endpoint": {f"joint_{index}": 0.0 for index in range(1, 7)},
                        "observed_endpoint": {f"joint_{index}": 0.005 for index in range(1, 7)},
                        "endpoint_gate": {"tolerance_rad": 0.01, "consecutive_samples": 3},
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
            "approach": {"planning": {"collision_aware": True}, "execution": {"action_status": 4}},
            "retreat": {"planning": {"collision_aware": True}, "execution": {"action_status": 4}},
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


def test_evaluator_rejects_an_endpoint_that_exceeds_the_tighter_live_gate(tmp_path):
    invalid = _evidence("heart", 0.14)
    invalid["execution"]["cartesian"]["segments"][0]["observed_endpoint"]["joint_3"] = 0.011
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
