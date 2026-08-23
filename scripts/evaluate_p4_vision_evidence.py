#!/usr/bin/env python3
"""Compare sealed vision-only outputs with separately captured live poses."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from typing import Any


THRESHOLDS_M = {
    "green_ball": 0.05,
    "red_ball": 0.05,
    "blue_cube": 0.05,
    "yellow_cube": 0.05,
    "drawing_board": 0.06,
}

# Gazebo reports the drawing-board model origin at the centre of its 30 mm
# thick collision box.  The RGB-D planar detector, however, localizes the
# robot-facing visible surface.  The local -X face is robot-facing in the lab
# world; rotate it with the captured model pose before comparing it to vision.
DRAWING_BOARD_HALF_THICKNESS_M = 0.015
MODEL_VOLUME_CENTER_REFERENCE = "model_volume_center"
ROBOT_FACING_FRONT_SURFACE_REFERENCE = "robot_facing_front_surface"


class EvaluationError(RuntimeError):
    """Raised when comparison inputs do not preserve the required boundary."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file():
        raise EvaluationError(f"{label} does not exist: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise EvaluationError(f"cannot parse {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise EvaluationError(f"{label} must be a JSON object")
    return payload


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_path = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
    finally:
        try:
            os.unlink(temporary_path)
        except FileNotFoundError:
            pass


def position(value: Any, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 3:
        raise EvaluationError(f"{label} must be a three-value list")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise EvaluationError(f"{label} has a non-finite value")
    return result


def drawing_board_front_surface(
    model_center_m: list[float], model_rpy_rad: list[float]
) -> tuple[list[float], list[float]]:
    """Return the robot-facing board surface and its world-frame normal.

    Gazebo model poses use roll-pitch-yaw rotations (Rz * Ry * Rx).  The
    board's robot-facing local normal is -X, so roll does not change that
    axis; the resulting world normal remains unit length.
    """

    _, pitch, yaw = model_rpy_rad
    normal_world = [
        -math.cos(yaw) * math.cos(pitch),
        -math.sin(yaw) * math.cos(pitch),
        math.sin(pitch),
    ]
    return (
        [
            model_center_m[index]
            + DRAWING_BOARD_HALF_THICKNESS_M * normal_world[index]
            for index in range(3)
        ],
        normal_world,
    )


def live_comparison_target(
    class_id: str, model: Any
) -> tuple[list[float], dict[str, Any]]:
    if not isinstance(model, dict):
        raise EvaluationError(f"{class_id} live model must be an object")
    model_center = position(model.get("position_m"), f"{class_id} live position")
    if class_id != "drawing_board":
        return model_center, {
            "name": MODEL_VOLUME_CENTER_REFERENCE,
            "live_model_volume_center_m": model_center,
        }
    model_rpy = position(model.get("rpy_rad"), "drawing_board live model RPY")
    front_surface, normal_world = drawing_board_front_surface(model_center, model_rpy)
    return front_surface, {
        "name": ROBOT_FACING_FRONT_SURFACE_REFERENCE,
        "live_model_volume_center_m": model_center,
        "live_model_rpy_rad": model_rpy,
        "front_surface_normal_world": normal_world,
        "front_surface_offset_from_model_center_m": DRAWING_BOARD_HALF_THICKNESS_M,
    }


def build_evaluation(arguments: argparse.Namespace) -> dict[str, Any]:
    vision = load_json(arguments.vision_evidence, "sealed vision evidence")
    if vision.get("perception_mode") != "vision":
        raise EvaluationError("vision evidence does not identify vision mode")
    if vision.get("ground_truth_input_used") is not False:
        raise EvaluationError("vision evidence does not prove the input boundary")
    sealed_at = vision.get("sealed_at_utc")
    if not isinstance(sealed_at, str) or not sealed_at:
        raise EvaluationError("vision evidence has no sealing timestamp")
    if vision.get("expected_classes") != list(THRESHOLDS_M):
        raise EvaluationError("vision evidence has unexpected class labels")
    outputs_path = arguments.vision_evidence.parent / "vision_outputs.json"
    outputs = load_json(outputs_path, "vision output record")
    expected_hash = vision.get("artifact_sha256", {}).get("evidence/vision_outputs.json")
    if expected_hash != sha256_file(outputs_path):
        raise EvaluationError("vision output record does not match its sealed hash")
    world_poses = outputs.get("world_poses")
    if not isinstance(world_poses, list):
        raise EvaluationError("vision output record has no world poses")
    vision_by_class = {entry.get("class_id"): entry for entry in world_poses if isinstance(entry, dict)}
    if set(vision_by_class) != set(THRESHOLDS_M):
        raise EvaluationError("vision output record lacks one or more required classes")
    capture = load_json(arguments.live_ground_truth, "offline live-pose evidence")
    if capture.get("access_boundary") != "OFFLINE_AFTER_SEALED_VISION_EVIDENCE":
        raise EvaluationError("offline evidence has the wrong access boundary")
    if capture.get("sealed_vision_evidence_sha256") != sha256_file(arguments.vision_evidence):
        raise EvaluationError("offline evidence is not tied to this sealed vision evidence")
    captured_at = capture.get("capture_started_at_utc")
    if not isinstance(captured_at, str) or captured_at <= sealed_at:
        raise EvaluationError("offline capture did not occur after vision evidence sealing")
    live_models = capture.get("models")
    if not isinstance(live_models, dict) or set(live_models) != set(THRESHOLDS_M):
        raise EvaluationError("offline evidence lacks required live models")
    results: dict[str, Any] = {}
    overall = True
    for class_id, threshold in THRESHOLDS_M.items():
        vision_entry = vision_by_class[class_id]
        if vision_entry.get("source_mode") != 1:
            raise EvaluationError(f"{class_id} output does not identify vision source mode")
        header = vision_entry.get("header")
        source_header = vision_entry.get("source_header")
        if not isinstance(header, dict) or header.get("frame_id") != "world":
            raise EvaluationError(f"{class_id} output is not a world-frame pose")
        if not isinstance(source_header, dict) or not source_header.get("frame_id"):
            raise EvaluationError(f"{class_id} output has no source camera frame")
        predicted = position(vision_entry.get("position_m"), f"{class_id} vision position")
        observed, reference = live_comparison_target(class_id, live_models[class_id])
        error_m = math.sqrt(sum((predicted[index] - observed[index]) ** 2 for index in range(3)))
        passed = error_m <= threshold
        overall = overall and passed
        results[class_id] = {
            "vision_position_m": predicted,
            "live_position_m": observed,
            "reference": reference,
            "position_error_m": error_m,
            "threshold_m": threshold,
            "status": "PASS" if passed else "FAIL",
        }
    return {
        "schema_version": 2,
        "scope": "SIMULATION_ONLY",
        "evaluated_at_utc": utc_now(),
        "evaluation_mode": "OFFLINE_COMPARISON_ONLY",
        "vision_evidence": str(arguments.vision_evidence.resolve()),
        "vision_evidence_sha256": sha256_file(arguments.vision_evidence),
        "live_ground_truth": str(arguments.live_ground_truth.resolve()),
        "live_ground_truth_sha256": sha256_file(arguments.live_ground_truth),
        "results": results,
        "status": {
            "p4_offline_evaluation": "PASS" if overall else "FAIL",
            "a03_green_ball_vision_localization": results["green_ball"]["status"],
            "all_five_localizations": "PASS" if overall else "FAIL",
        },
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vision-evidence", required=True, type=Path)
    parser.add_argument("--live-ground-truth", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser


def main() -> int:
    arguments = build_parser().parse_args()
    try:
        result = build_evaluation(arguments)
        atomic_write_json(arguments.output, result)
    except (EvaluationError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    status = result["status"]
    print(f"P4_OFFLINE_EVALUATION={status['p4_offline_evaluation']}")
    print(f"A03_GREEN_BALL_VISION_LOCALIZATION={status['a03_green_ball_vision_localization']}")
    print(f"OFFLINE_COMPARISON={arguments.output}")
    return 0 if status["p4_offline_evaluation"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
