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
    for index, waypoint in enumerate(world):
        require(isinstance(waypoint, list) and len(waypoint) == 3, f"{path}: malformed world waypoint {index}")
        for axis, component in enumerate(waypoint):
            finite(component, f"{path}: world waypoint {index}/{axis}")
    board = value.get("board_frame")
    require(isinstance(board, dict) and board.get("name") == "drawing_board", f"{path}: missing board frame")
    require(board.get("center_source") == "fresh_rgbd_vision", f"{path}: board was not observed by RGB-D")
    observation = board.get("observation")
    require(isinstance(observation, dict) and observation.get("source_mode") == "VISION", f"{path}: board observation provenance invalid")
    require(board.get("board_collision_preserved") is True, f"{path}: board collision was not preserved")
    require(board.get("tool_link") == "drawing_tip", f"{path}: runtime did not use the Phase-6 drawing tip")
    require(board.get("tool_model") == "phase6_collision_bearing_drawing_pen", f"{path}: drawing tool provenance is invalid")
    scene = value.get("planning_scene_ids")
    require(isinstance(scene, list) and "p6_live_vision_drawing_board" in scene, f"{path}: board collision absent from MoveIt scene")
    execution = value.get("execution")
    require(isinstance(execution, dict), f"{path}: execution record missing")
    preflight = execution.get("pre_motion_cartesian_gate")
    require(isinstance(preflight, dict) and preflight.get("result") == "PASS", f"{path}: route was not accepted before motion")
    require(preflight.get("collision_aware") is True and preflight.get("avoid_collisions") is True, f"{path}: pre-motion route lacks collision checking")
    require(preflight.get("start_state") == "planned_approach_endpoint", f"{path}: pre-motion route used an invalid start state")
    require(preflight.get("execution_route") == "preflighted_live_start_cartesian_trajectories", f"{path}: execution did not use the accepted Cartesian route")
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
            live_preflight = segment.get("pre_motion_cartesian")
            require(isinstance(live_preflight, dict), f"{path}: Cartesian segment {index} lacks its pre-motion check")
            require(live_preflight.get("avoid_collisions") is True, f"{path}: Cartesian segment {index} pre-motion check lacks collision checking")
            require(finite(live_preflight.get("fraction"), f"{path}: Cartesian segment {index} pre-motion fraction") >= 0.999999, f"{path}: Cartesian segment {index} pre-motion route is incomplete")
            targets = segment.get("live_execution_targets")
            require(isinstance(targets, dict), f"{path}: Cartesian segment {index} lacks live-start target provenance")
            require(targets.get("strategy") == "live_state_to_forward_generated_waypoints", f"{path}: Cartesian segment {index} did not use forward generated targets")
            preflight_count = finite(targets.get("preflight_waypoint_count"), f"{path}: Cartesian segment {index} preflight target count")
            live_count = finite(targets.get("live_forward_target_count"), f"{path}: Cartesian segment {index} live target count")
            require(preflight_count >= 2.0 and live_count >= 1.0 and live_count == preflight_count - 1.0, f"{path}: Cartesian segment {index} has invalid live-start target provenance")
            require(segment.get("execution", {}).get("action_status") == 4, f"{path}: Cartesian segment {index} did not execute")
            endpoint_gate = segment.get("endpoint_gate")
            require(isinstance(endpoint_gate, dict), f"{path}: Cartesian segment {index} lacks measured endpoint gate")
            endpoint_tolerance = finite(endpoint_gate.get("tolerance_rad"), f"{path}: Cartesian segment {index} endpoint tolerance")
            require(0.0 < endpoint_tolerance <= 0.01, f"{path}: Cartesian segment {index} endpoint gate is too loose")
            require(finite(endpoint_gate.get("consecutive_samples"), f"{path}: Cartesian segment {index} endpoint sample count") >= 3.0, f"{path}: Cartesian segment {index} endpoint gate is undersampled")
            expected_endpoint = segment.get("expected_endpoint")
            observed_endpoint = segment.get("observed_endpoint")
            require(isinstance(expected_endpoint, dict) and isinstance(observed_endpoint, dict), f"{path}: Cartesian segment {index} endpoint evidence is malformed")
            require(set(expected_endpoint) == set(observed_endpoint) and len(expected_endpoint) == 6, f"{path}: Cartesian segment {index} endpoint joint set is invalid")
            endpoint_error = max(
                abs(
                    finite(observed_endpoint[name], f"{path}: Cartesian segment {index} observed {name}")
                    - finite(expected_endpoint[name], f"{path}: Cartesian segment {index} expected {name}")
                )
                for name in expected_endpoint
            )
            require(endpoint_error <= endpoint_tolerance + 1e-9, f"{path}: Cartesian segment {index} observed endpoint exceeds its gate")
            retiming = segment.get("retiming")
            require(isinstance(retiming, dict), f"{path}: Cartesian segment {index} lacks retiming evidence")
            hold = retiming.get("terminal_hold")
            require(isinstance(hold, dict), f"{path}: Cartesian segment {index} lacks terminal-hold evidence")
            hold_duration = finite(hold.get("duration_sec"), f"{path}: Cartesian segment {index} terminal-hold duration")
            require(0.0 < hold_duration <= 2.0, f"{path}: Cartesian segment {index} terminal hold is outside the bounded runtime policy")
            require(hold.get("repeats_collision_checked_endpoint") is True, f"{path}: Cartesian segment {index} terminal hold changed the collision-checked endpoint")
    for stage in ("approach", "retreat"):
        require(execution.get(stage, {}).get("planning", {}).get("collision_aware") is True, f"{path}: {stage} lacks collision-aware planning")
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
