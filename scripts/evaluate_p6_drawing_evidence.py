#!/usr/bin/env python3
"""Independently validate bounded Phase-6 drawing runtime evidence."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import tempfile
import os
from typing import Any


class EvaluationError(RuntimeError):
    pass


def load(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise EvaluationError(f"missing evidence: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise EvaluationError(f"invalid JSON in {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise EvaluationError(f"evidence root is not a mapping: {path}")
    return value


def require(condition: bool, message: str) -> None:
    if not condition:
        raise EvaluationError(message)


def finite(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise EvaluationError(f"{label} is not numeric") from exc
    if not math.isfinite(result):
        raise EvaluationError(f"{label} is not finite")
    return result


def vector(value: Any, label: str, length: int) -> list[float]:
    require(isinstance(value, list) and len(value) == length, f"{label} is malformed")
    return [finite(item, f"{label}[{index}]") for index, item in enumerate(value)]


def validate_endpoint_gate(record: dict[str, Any], label: str) -> None:
    gate = record.get("endpoint_gate")
    require(isinstance(gate, dict), f"{label} lacks measured endpoint gate")
    tolerance = finite(gate.get("tolerance_rad"), f"{label} endpoint tolerance")
    require(0.0 < tolerance <= 0.03, f"{label} endpoint gate is too loose")
    require(finite(gate.get("consecutive_samples"), f"{label} endpoint sample count") >= 3.0, f"{label} endpoint gate is undersampled")
    expected = record.get("expected_endpoint")
    observed = record.get("observed_endpoint")
    require(isinstance(expected, dict) and isinstance(observed, dict), f"{label} endpoint evidence is malformed")
    require(set(expected) == set(observed) and len(expected) == 6, f"{label} endpoint joint set is invalid")
    maximum_error = max(
        abs(
            finite(observed[name], f"{label} observed {name}")
            - finite(expected[name], f"{label} expected {name}")
        )
        for name in expected
    )
    require(maximum_error <= tolerance + 1e-9, f"{label} observed endpoint exceeds its gate")


def joint_margin_gate(value: Any, label: str) -> None:
    require(isinstance(value, dict), f"{label} lacks a joint-limit margin gate")
    required = finite(value.get("minimum_required_margin_rad"), f"{label} required joint-limit margin")
    observed = finite(value.get("route_minimum_margin_rad"), f"{label} route joint-limit margin")
    require(required >= 0.20 and observed >= required, f"{label} has insufficient joint-limit margin")


STATIC_MARKERS = (
    "p5_orange_top_fiducial",
    "p5_violet_top_fiducial",
)
STATIC_SCENE_IDS = tuple(f"p6_vision_static_{marker}" for marker in STATIC_MARKERS)
REQUIRED_MOTION_SCENE_IDS = (
    "p6_work_table",
    "p6_live_vision_drawing_board",
    *STATIC_SCENE_IDS,
)


def require_scene_ids(value: Any, required_ids: tuple[str, ...], label: str) -> list[str]:
    require(isinstance(value, list), f"{label} lacks planning-scene evidence")
    identifiers = [str(item) for item in value]
    missing = sorted(set(required_ids).difference(identifiers))
    require(not missing, f"{label} omits required static collision objects: {missing}")
    return identifiers


def validate_terminal_hold(retiming: Any, label: str, *, maximum_duration: float = 2.0) -> dict[str, Any]:
    require(isinstance(retiming, dict), f"{label} lacks retiming evidence")
    hold = retiming.get("terminal_hold")
    require(isinstance(hold, dict), f"{label} lacks terminal-hold evidence")
    hold_duration = finite(hold.get("duration_sec"), f"{label} terminal-hold duration")
    require(0.0 < hold_duration <= maximum_duration, f"{label} terminal hold is outside the bounded runtime policy")
    require(hold.get("repeats_collision_checked_endpoint") is True, f"{label} terminal hold changed the collision-checked endpoint")
    return retiming


def validate_static_obstacles(value: dict[str, Any], board_stamp_ns: float, scene_ids: list[str], label: str) -> None:
    static = value.get("static_collision_obstacles")
    require(isinstance(static, dict), f"{label} lacks visual static-obstacle evidence")
    require(static.get("source_topic") == "/phase5/perception/static_obstacle_poses", f"{label} used an unexpected static-obstacle topic")
    require(tuple(static.get("required_marker_class_ids", ())) == STATIC_MARKERS, f"{label} has an invalid static-obstacle contract")
    records = static.get("records")
    require(isinstance(records, list) and len(records) == len(STATIC_MARKERS), f"{label} lacks the two required static obstacles")
    by_marker: dict[str, dict[str, Any]] = {}
    marker_stamps: set[float] = set()
    for record in records:
        require(isinstance(record, dict), f"{label} has malformed static-obstacle evidence")
        marker = record.get("marker")
        require(isinstance(marker, dict), f"{label} static obstacle has no visual marker provenance")
        class_id = marker.get("class_id")
        require(class_id in STATIC_MARKERS and class_id not in by_marker, f"{label} static marker set is invalid")
        require(record.get("object_id") == f"p6_vision_static_{class_id}", f"{label} static collision ID is invalid")
        require(record.get("source") == "fresh_phase5_rgbd_static_obstacle_localizer", f"{label} static obstacle did not come from RGB-D")
        require(marker.get("source_mode") == "VISION", f"{label} static obstacle provenance is not visual")
        require(marker.get("frame_id") == "world", f"{label} static obstacle is not in the world frame")
        require(isinstance(marker.get("detection_id"), str) and marker["detection_id"], f"{label} static obstacle has no detection identity")
        marker_stamp = finite(marker.get("source_stamp_ns"), f"{label} {class_id} source stamp")
        require(marker_stamp > board_stamp_ns, f"{label} {class_id} is not fresher than the board observation")
        marker_stamps.add(marker_stamp)
        require(finite(marker.get("confidence"), f"{label} {class_id} confidence") >= 0.60, f"{label} {class_id} confidence is below the static-scene contract")
        marker_position = vector(marker.get("position_m"), f"{label} {class_id} marker position", 3)
        offset = finite(record.get("marker_top_to_box_center_z_offset_m"), f"{label} {class_id} top-to-centre offset")
        require(math.isclose(offset, 0.104, rel_tol=0.0, abs_tol=1e-9), f"{label} {class_id} uses an invalid top-to-centre offset")
        center = vector(record.get("collision_box_center_m"), f"{label} {class_id} collision-box centre", 3)
        require(
            all(math.isclose(center[index], marker_position[index] - (offset if index == 2 else 0.0), rel_tol=0.0, abs_tol=1e-6) for index in range(3)),
            f"{label} {class_id} collision-box transform is invalid",
        )
        size = vector(record.get("collision_box_size_xyz_m"), f"{label} {class_id} collision-box size", 3)
        require(all(math.isclose(size[index], expected, rel_tol=0.0, abs_tol=1e-9) for index, expected in enumerate((0.160, 0.160, 0.220))), f"{label} {class_id} collision-box size is invalid")
        by_marker[class_id] = record
    require(tuple(by_marker) == STATIC_MARKERS, f"{label} static marker set is incomplete")
    require(len(marker_stamps) == 1, f"{label} static obstacles do not share one RGB-D epoch")
    require_scene_ids(scene_ids, STATIC_SCENE_IDS, label)


def validate_one(path: Path) -> dict[str, Any]:
    value = load(path)
    require(value.get("schema_version") == 1, f"{path}: unsupported schema")
    require(value.get("scope") == "SIMULATION_ONLY_CONTACTLESS_DRAWING_TRACE", f"{path}: unexpected scope")
    require(value.get("online_ground_truth_input_used") is False, f"{path}: online ground truth was used")
    status = value.get("status")
    require(isinstance(status, dict) and status.get("P6_DRAWING_RUNTIME") == "PASS", f"{path}: runtime did not pass")
    intent = value.get("drawing_intent")
    require(isinstance(intent, dict) and intent.get("shape") in {"circle", "square", "triangle", "heart", "wave", "line"}, f"{path}: invalid DrawingIntent")
    size = finite(intent.get("size_m"), f"{path}: intent size")
    require(0.04 <= size <= 0.24, f"{path}: intent size outside bounded range")
    geometry = value.get("geometry")
    require(isinstance(geometry, dict) and geometry.get("generated_at_runtime") is True, f"{path}: geometry was not generated at runtime")
    require(geometry.get("fixed_world_geometry_input") is False, f"{path}: geometry was authored in fixed world coordinates")
    require(geometry.get("coordinate_chain") == "drawing-local -> drawing-board frame -> world -> MoveIt Cartesian waypoints -> ABB execution", f"{path}: invalid coordinate chain")
    local, world = geometry.get("local_points"), geometry.get("world_waypoints_m")
    require(isinstance(local, list) and isinstance(world, list) and len(local) >= 16 and len(local) == len(world), f"{path}: incomplete generated geometry")
    world = [vector(waypoint, f"{path}: world waypoint {index}", 3) for index, waypoint in enumerate(world)]
    board = value.get("board_frame")
    require(isinstance(board, dict) and board.get("name") == "drawing_board", f"{path}: missing board frame")
    require(board.get("observation_reference") == "robot_facing_front_surface", f"{path}: board reference is not its robot-facing surface")
    require(board.get("center_source") == "fresh_rgbd_front_surface", f"{path}: board was not observed by RGB-D")
    observation = board.get("observation")
    require(isinstance(observation, dict) and observation.get("source_mode") == "VISION", f"{path}: board observation provenance invalid")
    require(observation.get("frame_id") == "world", f"{path}: board observation is not in the world frame")
    require(isinstance(observation.get("detection_id"), str) and observation["detection_id"], f"{path}: board observation has no detection identity")
    board_stamp_ns = finite(observation.get("source_stamp_ns"), f"{path}: board source stamp")
    observation_position = vector(observation.get("position_m"), f"{path}: board observation position", 3)
    front_surface = vector(board.get("front_surface_world_m"), f"{path}: board front surface", 3)
    derived_center = vector(board.get("derived_center_world_m"), f"{path}: derived board centre", 3)
    collision_center = vector(board.get("collision_center_world_m"), f"{path}: collision board centre", 3)
    normal = vector(board.get("normal_world"), f"{path}: board normal", 3)
    horizontal = vector(board.get("horizontal_world"), f"{path}: board horizontal axis", 3)
    vertical = vector(board.get("vertical_world"), f"{path}: board vertical axis", 3)
    front_offset = finite(board.get("front_surface_offset_m"), f"{path}: board front-surface offset")
    standoff = finite(board.get("virtual_tip_standoff_m"), f"{path}: board virtual-tip standoff")
    require(front_offset > 0.0, f"{path}: board front-surface offset is invalid")
    require(standoff > 0.0, f"{path}: board virtual-tip standoff is invalid")
    require(
        all(math.isclose(observation_position[index], front_surface[index], rel_tol=0.0, abs_tol=1e-6) for index in range(3)),
        f"{path}: board front surface differs from its visual observation",
    )
    axes = (normal, horizontal, vertical)
    require(all(math.isclose(math.sqrt(sum(component * component for component in axis)), 1.0, rel_tol=0.0, abs_tol=1e-6) for axis in axes), f"{path}: board axes are not unit length")
    require(
        all(abs(sum(left[index] * right[index] for index in range(3))) <= 1e-6 for left, right in ((normal, horizontal), (normal, vertical), (horizontal, vertical))),
        f"{path}: board axes are not orthogonal",
    )
    require(
        all(math.isclose(derived_center[index], front_surface[index] - front_offset * normal[index], rel_tol=0.0, abs_tol=1e-6) for index in range(3)),
        f"{path}: board surface-to-centre transform is invalid",
    )
    require(
        all(math.isclose(collision_center[index], derived_center[index], rel_tol=0.0, abs_tol=1e-9) for index in range(3)),
        f"{path}: collision board centre differs from the derived volume centre",
    )
    for index, local_point in enumerate(local):
        require(isinstance(local_point, dict), f"{path}: local waypoint {index} is malformed")
        local_normal = finite(local_point.get("normal_m"), f"{path}: local waypoint {index} normal")
        local_horizontal = finite(local_point.get("horizontal_m"), f"{path}: local waypoint {index} horizontal")
        local_vertical = finite(local_point.get("vertical_m"), f"{path}: local waypoint {index} vertical")
        require(abs(local_normal) <= 1e-9, f"{path}: local waypoint {index} leaves the drawing plane")
        expected_world = [
            derived_center[axis]
            + (front_offset + standoff + local_normal) * normal[axis]
            + local_horizontal * horizontal[axis]
            + local_vertical * vertical[axis]
            for axis in range(3)
        ]
        require(
            all(math.isclose(world[index][axis], expected_world[axis], rel_tol=0.0, abs_tol=1e-6) for axis in range(3)),
            f"{path}: world waypoint {index} does not map from its local board point",
        )
    require(board.get("board_collision_preserved") is True, f"{path}: board collision was not preserved")
    require(board.get("tool_link") == "drawing_tip", f"{path}: runtime did not use the Phase-6 drawing tip")
    require(board.get("tool_model") == "phase6_collision_bearing_drawing_pen", f"{path}: drawing tool provenance is invalid")
    scene = value.get("planning_scene_ids")
    require_scene_ids(scene, REQUIRED_MOTION_SCENE_IDS, f"{path}: top-level motion scene")
    validate_static_obstacles(value, board_stamp_ns, scene, str(path))
    execution = value.get("execution")
    require(isinstance(execution, dict), f"{path}: execution record missing")
    preflight = execution.get("pre_motion_cartesian_gate")
    require(isinstance(preflight, dict) and preflight.get("result") == "PASS", f"{path}: route was not accepted before motion")
    require(preflight.get("collision_aware") is True and preflight.get("avoid_collisions") is True, f"{path}: pre-motion route lacks collision checking")
    require(preflight.get("start_state") == "planned_approach_then_entry_endpoint", f"{path}: pre-motion route used an invalid start state")
    require(preflight.get("execution_route") == "preflighted_live_entry_then_cartesian_trajectories", f"{path}: execution did not use the accepted Cartesian route")
    joint_margin = preflight.get("joint_limit_margin")
    require(isinstance(joint_margin, dict), f"{path}: pre-motion route lacks a joint-limit margin gate")
    minimum_margin = finite(joint_margin.get("minimum_required_margin_rad"), f"{path}: required joint-limit margin")
    route_margin = finite(joint_margin.get("route_minimum_margin_rad"), f"{path}: route joint-limit margin")
    require(minimum_margin >= 0.20 and route_margin >= minimum_margin, f"{path}: pre-motion route has insufficient joint-limit margin")
    require(joint_margin.get("closest_joint") in {f"joint_{index}" for index in range(1, 7)}, f"{path}: pre-motion route has invalid closest joint")
    by_joint_margin = joint_margin.get("by_joint_minimum_margin_rad")
    require(isinstance(by_joint_margin, dict) and set(by_joint_margin) == {f"joint_{index}" for index in range(1, 7)}, f"{path}: pre-motion route lacks six-axis margin evidence")
    for name, margin in by_joint_margin.items():
        require(finite(margin, f"{path}: pre-motion {name} margin") >= minimum_margin, f"{path}: pre-motion {name} margin is insufficient")
    entry = execution.get("entry_alignment")
    require(isinstance(entry, dict), f"{path}: curve started without an explicit entry alignment")
    require(entry.get("avoid_collisions") is True, f"{path}: entry alignment lacks collision checking")
    require(finite(entry.get("fraction"), f"{path}: entry alignment fraction") >= 0.999999, f"{path}: entry alignment is incomplete")
    require(entry.get("execution", {}).get("action_status") == 4, f"{path}: entry alignment did not execute")
    require(finite(entry.get("requested_waypoint_count"), f"{path}: entry requested waypoint count") == 1.0, f"{path}: entry alignment must have exactly one target")
    require_scene_ids(entry.get("planning_scene_ids"), REQUIRED_MOTION_SCENE_IDS, f"{path}: entry alignment")
    entry_preflight = entry.get("pre_motion_cartesian")
    require(isinstance(entry_preflight, dict), f"{path}: entry alignment lacks its pre-motion check")
    require(entry_preflight.get("avoid_collisions") is True, f"{path}: entry pre-motion check lacks collision checking")
    require(finite(entry_preflight.get("fraction"), f"{path}: entry pre-motion fraction") >= 0.999999, f"{path}: entry pre-motion route is incomplete")
    require(finite(entry_preflight.get("requested_waypoint_count"), f"{path}: entry pre-motion requested waypoint count") == 1.0, f"{path}: entry pre-motion route must have exactly one target")
    require_scene_ids(entry_preflight.get("planning_scene_ids"), REQUIRED_MOTION_SCENE_IDS, f"{path}: entry pre-motion check")
    entry_targets = entry.get("target_provenance")
    require(isinstance(entry_targets, dict), f"{path}: entry alignment lacks target provenance")
    require(entry_targets.get("strategy") == "live_state_to_runtime_generated_first_waypoint", f"{path}: entry alignment target is not the first runtime-generated waypoint")
    require(finite(entry_targets.get("target_count"), f"{path}: entry target count") == 1.0, f"{path}: entry alignment has an invalid target count")
    joint_margin_gate(entry.get("live_joint_limit_margin"), f"{path}: entry alignment")
    validate_endpoint_gate(entry, f"{path}: entry alignment")
    entry_retiming = validate_terminal_hold(entry.get("retiming"), f"{path}: entry alignment")
    require(entry_retiming.get("profile") == "entry", f"{path}: entry alignment did not use the dedicated entry profile")
    require(finite(entry_retiming.get("duration_sec"), f"{path}: entry duration") <= 60.0, f"{path}: entry alignment exceeds its bounded duration")
    tcp_gate = entry.get("tcp_gate")
    require(isinstance(tcp_gate, dict), f"{path}: entry alignment lacks a measured TCP gate")
    require(finite(tcp_gate.get("consecutive_samples"), f"{path}: entry TCP sample count") >= 3.0, f"{path}: entry TCP gate is undersampled")
    position_tolerance = finite(tcp_gate.get("position_tolerance_m"), f"{path}: entry TCP position tolerance")
    orientation_tolerance = finite(tcp_gate.get("orientation_tolerance_rad"), f"{path}: entry TCP orientation tolerance")
    require(0.0 < position_tolerance <= 0.008, f"{path}: entry TCP position gate is too loose")
    require(0.0 < orientation_tolerance <= 0.08, f"{path}: entry TCP orientation gate is too loose")
    expected_position = vector(tcp_gate.get("expected_position_m"), f"{path}: entry TCP expected position", 3)
    expected_orientation = vector(tcp_gate.get("expected_orientation_xyzw"), f"{path}: entry TCP expected orientation", 4)
    require(
        all(math.isclose(expected_position[index], world[0][index], rel_tol=0.0, abs_tol=1e-6) for index in range(3)),
        f"{path}: entry TCP target is not the first generated world waypoint",
    )
    selected_orientation = vector(execution.get("orientation_xyzw"), f"{path}: selected drawing orientation", 4)
    require(
        all(math.isclose(expected_orientation[index], selected_orientation[index], rel_tol=0.0, abs_tol=1e-6) for index in range(4)),
        f"{path}: entry TCP orientation differs from the selected drawing orientation",
    )
    tcp_observed = tcp_gate.get("observed")
    require(isinstance(tcp_observed, dict), f"{path}: entry TCP observation is malformed")
    observed_position = vector(tcp_observed.get("position_m"), f"{path}: entry TCP observed position", 3)
    observed_orientation = vector(tcp_observed.get("orientation_xyzw"), f"{path}: entry TCP observed orientation", 4)
    measured_position_error = finite(tcp_observed.get("position_error_m"), f"{path}: entry TCP position error")
    require(measured_position_error <= position_tolerance + 1e-9, f"{path}: entry TCP position exceeds its gate")
    recomputed_position_error = math.sqrt(sum((observed_position[index] - expected_position[index]) ** 2 for index in range(3)))
    require(math.isclose(measured_position_error, recomputed_position_error, rel_tol=0.0, abs_tol=1e-6), f"{path}: entry TCP position error is inconsistent with the recorded poses")
    measured_orientation_error = finite(tcp_observed.get("orientation_error_rad"), f"{path}: entry TCP orientation error")
    require(measured_orientation_error <= orientation_tolerance + 1e-9, f"{path}: entry TCP orientation exceeds its gate")
    observed_norm = math.sqrt(sum(component * component for component in observed_orientation))
    expected_norm = math.sqrt(sum(component * component for component in expected_orientation))
    require(observed_norm > 0.0 and expected_norm > 0.0, f"{path}: entry TCP orientation is invalid")
    orientation_dot = abs(sum(observed_orientation[index] * expected_orientation[index] for index in range(4)) / (observed_norm * expected_norm))
    recomputed_orientation_error = 2.0 * math.acos(min(1.0, max(-1.0, orientation_dot)))
    require(math.isclose(measured_orientation_error, recomputed_orientation_error, rel_tol=0.0, abs_tol=1e-6), f"{path}: entry TCP orientation error is inconsistent with the recorded poses")
    cartesian = execution.get("cartesian")
    require(isinstance(cartesian, dict) and cartesian.get("avoid_collisions") is True, f"{path}: Cartesian collision checking missing")
    require(finite(cartesian.get("fraction"), f"{path}: Cartesian fraction") >= 0.999999, f"{path}: incomplete Cartesian path")
    require(cartesian.get("execution", {}).get("action_status") == 4, f"{path}: Cartesian execution did not succeed")
    segments = cartesian.get("segments")
    if segments is not None:
        require(isinstance(segments, list) and len(segments) >= 1, f"{path}: Cartesian segment record is malformed")
        for index, segment in enumerate(segments):
            require(segment.get("avoid_collisions") is True, f"{path}: Cartesian segment {index} lacks collision checking")
            require(finite(segment.get("fraction"), f"{path}: Cartesian segment {index} fraction") >= 0.999999, f"{path}: Cartesian segment {index} is incomplete")
            require_scene_ids(segment.get("planning_scene_ids"), REQUIRED_MOTION_SCENE_IDS, f"{path}: Cartesian segment {index}")
            live_preflight = segment.get("pre_motion_cartesian")
            require(isinstance(live_preflight, dict), f"{path}: Cartesian segment {index} lacks its pre-motion check")
            require(live_preflight.get("avoid_collisions") is True, f"{path}: Cartesian segment {index} pre-motion check lacks collision checking")
            require(finite(live_preflight.get("fraction"), f"{path}: Cartesian segment {index} pre-motion fraction") >= 0.999999, f"{path}: Cartesian segment {index} pre-motion route is incomplete")
            require_scene_ids(live_preflight.get("planning_scene_ids"), REQUIRED_MOTION_SCENE_IDS, f"{path}: Cartesian segment {index} pre-motion check")
            live_margin = segment.get("live_joint_limit_margin")
            require(isinstance(live_margin, dict), f"{path}: Cartesian segment {index} lacks live joint-limit margin evidence")
            live_required_margin = finite(live_margin.get("minimum_required_margin_rad"), f"{path}: Cartesian segment {index} live required joint-limit margin")
            live_route_margin = finite(live_margin.get("route_minimum_margin_rad"), f"{path}: Cartesian segment {index} live joint-limit margin")
            require(live_required_margin >= 0.20 and live_route_margin >= live_required_margin, f"{path}: Cartesian segment {index} live route has insufficient joint-limit margin")
            targets = segment.get("live_execution_targets")
            require(isinstance(targets, dict), f"{path}: Cartesian segment {index} lacks live-start target provenance")
            require(targets.get("strategy") == "live_state_to_forward_generated_waypoints", f"{path}: Cartesian segment {index} did not use forward generated targets")
            preflight_count = finite(targets.get("preflight_waypoint_count"), f"{path}: Cartesian segment {index} preflight target count")
            live_count = finite(targets.get("live_forward_target_count"), f"{path}: Cartesian segment {index} live target count")
            require(preflight_count >= 2.0 and live_count >= 1.0 and live_count == preflight_count - 1.0, f"{path}: Cartesian segment {index} has invalid live-start target provenance")
            require(segment.get("execution", {}).get("action_status") == 4, f"{path}: Cartesian segment {index} did not execute")
            validate_endpoint_gate(segment, f"{path}: Cartesian segment {index}")
            validate_terminal_hold(segment.get("retiming"), f"{path}: Cartesian segment {index}")
    for stage in ("approach", "retreat"):
        require(execution.get(stage, {}).get("planning", {}).get("collision_aware") is True, f"{path}: {stage} lacks collision-aware planning")
        require_scene_ids(execution.get(stage, {}).get("planning", {}).get("planning_scene_ids"), REQUIRED_MOTION_SCENE_IDS, f"{path}: {stage}")
        require(execution.get(stage, {}).get("execution", {}).get("action_status") == 4, f"{path}: {stage} execution did not succeed")
    trace = value.get("tcp_trace")
    require(isinstance(trace, list) and len(trace) >= 5, f"{path}: insufficient live TCP trace")
    return {"path": str(path), "shape": intent["shape"], "size_m": size, "position": intent.get("position"), "tcp_trace_samples": len(trace)}


def evaluate(paths: list[Path]) -> dict[str, Any]:
    records = [validate_one(path) for path in paths]
    shapes = {record["shape"] for record in records}
    require(bool(shapes & {"circle", "heart", "wave"}), "no curved generated drawing evidence")
    require(bool(shapes & {"square", "triangle"}), "no polygon generated drawing evidence")
    has_parameter_variation = len({record["size_m"] for record in records}) > 1 or any(record["position"] != "center" for record in records)
    require(has_parameter_variation, "no runtime drawing parameter variation")
    return {"schema_version": 1, "P6_DRAWING_EVALUATION": "PASS", "cases": records, "shapes": sorted(shapes), "parameter_variation": True}


def atomic_write(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def main(args: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence", required=True, action="append", type=Path, help="One drawing_execution_evidence.json; supply at least three.")
    parser.add_argument("--output", required=True, type=Path)
    options = parser.parse_args(args)
    try:
        require(len(options.evidence) >= 3, "Phase-6 acceptance needs at least three runtime cases")
        result = evaluate(options.evidence)
        atomic_write(options.output, result)
    except EvaluationError as exc:
        print(f"P6_DRAWING_EVALUATION=FAIL: {exc}")
        return 1
    print("P6_DRAWING_EVALUATION=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
