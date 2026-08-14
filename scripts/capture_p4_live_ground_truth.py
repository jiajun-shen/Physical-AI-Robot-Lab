#!/usr/bin/env python3
"""Capture offline live model poses only after Phase-4 vision evidence is sealed.

This program is intentionally outside the perception package. It never
publishes a pose and is invoked by the harness only after the vision-only
evidence manifest exists. Its output is comparison material, not a pipeline
input.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time
from typing import Any


CLASS_TO_MODEL = {
    "green_ball": "green_ball",
    "red_ball": "red_ball",
    "blue_cube": "blue_cube",
    "yellow_cube": "yellow_cube",
    "drawing_board": "drawing_board",
}
FLOAT_PATTERN = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
TRIPLE_PATTERN = re.compile(
    rf"\[\s*({FLOAT_PATTERN})\s+({FLOAT_PATTERN})\s+({FLOAT_PATTERN})\s*\]"
)
ANSI_ESCAPE_PATTERN = re.compile(r"\x1b\[[0-9;]*m")


class CaptureError(RuntimeError):
    """Raised when sealed evidence or a live model-pose response is invalid."""


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
        raise CaptureError(f"{label} does not exist: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CaptureError(f"cannot parse {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise CaptureError(f"{label} must be a JSON object")
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


def parse_live_pose(path: Path, model_name: str) -> tuple[list[float], list[float]]:
    text = ANSI_ESCAPE_PATTERN.sub("", path.read_text(encoding="utf-8"))
    lowered = text.lower()
    for marker in ("no model named", "request timed out", "timed out", "failed to", "error ["):
        if marker in lowered:
            raise CaptureError(f"live response for {model_name} contains {marker!r}")
    if model_name not in text:
        raise CaptureError(f"live response does not name {model_name}")
    triples = TRIPLE_PATTERN.findall(text)
    if len(triples) < 2:
        raise CaptureError(f"cannot parse XYZ and RPY from {model_name} response")
    values = [[float(item) for item in triple] for triple in triples[:2]]
    if not all(math.isfinite(item) for vector in values for item in vector):
        raise CaptureError(f"live response for {model_name} has a non-finite value")
    return values[0], values[1]


def capture_command(command: list[str], output: Path) -> None:
    try:
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise CaptureError(f"cannot run {' '.join(command)}: {exc}") from exc
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(completed.stdout + completed.stderr, encoding="utf-8")
    if completed.returncode != 0:
        raise CaptureError(f"{' '.join(command)} exited {completed.returncode}")


def vector_distance(first: list[float], second: list[float]) -> float:
    return math.sqrt(sum((first[index] - second[index]) ** 2 for index in range(3)))


def build_capture(arguments: argparse.Namespace) -> dict[str, Any]:
    vision = load_json(arguments.vision_evidence, "sealed vision evidence")
    if vision.get("perception_mode") != "vision":
        raise CaptureError("sealed evidence does not identify vision mode")
    if vision.get("ground_truth_input_used") is not False:
        raise CaptureError("sealed evidence does not prove the input boundary")
    sealed_at = vision.get("sealed_at_utc")
    if not isinstance(sealed_at, str) or not sealed_at:
        raise CaptureError("sealed evidence has no sealing timestamp")
    if vision.get("expected_classes") != list(CLASS_TO_MODEL):
        raise CaptureError("sealed evidence has unexpected classes")
    started_at = utc_now()
    if started_at <= sealed_at:
        raise CaptureError("offline capture timestamp did not follow evidence sealing")
    raw_root = arguments.output.parent / "raw_pose_logs"
    models_log = raw_root / "models.txt"
    capture_command(["gz", "model", "--list"], models_log)
    models: dict[str, dict[str, Any]] = {}
    for class_id, model_name in CLASS_TO_MODEL.items():
        first_path = raw_root / f"{model_name}_first.txt"
        second_path = raw_root / f"{model_name}_second.txt"
        capture_command(["gz", "model", "-m", model_name, "-p"], first_path)
        first_position, first_rpy = parse_live_pose(first_path, model_name)
        time.sleep(0.15)
        capture_command(["gz", "model", "-m", model_name, "-p"], second_path)
        second_position, second_rpy = parse_live_pose(second_path, model_name)
        stability = vector_distance(first_position, second_position)
        if stability > 0.01:
            raise CaptureError(
                f"live {model_name} moved {stability:.6f} m during the offline capture"
            )
        models[class_id] = {
            "model_name": model_name,
            "position_m": [
                (first_position[index] + second_position[index]) / 2.0
                for index in range(3)
            ],
            "rpy_rad": [
                (first_rpy[index] + second_rpy[index]) / 2.0 for index in range(3)
            ],
            "stability_delta_m": stability,
            "raw_files": {
                "first": first_path.relative_to(arguments.output.parent).as_posix(),
                "second": second_path.relative_to(arguments.output.parent).as_posix(),
            },
            "raw_sha256": {
                "first": sha256_file(first_path),
                "second": sha256_file(second_path),
            },
        }
    return {
        "schema_version": 1,
        "scope": "SIMULATION_ONLY",
        "capture_started_at_utc": started_at,
        "capture_finished_at_utc": utc_now(),
        "world_name": arguments.world_name,
        "access_boundary": "OFFLINE_AFTER_SEALED_VISION_EVIDENCE",
        "sealed_vision_evidence": str(arguments.vision_evidence.resolve()),
        "sealed_vision_evidence_sha256": sha256_file(arguments.vision_evidence),
        "models_log_relative_path": models_log.relative_to(arguments.output.parent).as_posix(),
        "models_log_sha256": sha256_file(models_log),
        "models": models,
        "status": {
            "sealed_vision_evidence_precedes_capture": "PASS",
            "live_model_pose_capture": "PASS",
            "model_stability": "PASS",
        },
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vision-evidence", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--world-name", default="manipulation_lab")
    return parser


def main() -> int:
    arguments = build_parser().parse_args()
    try:
        if arguments.output.resolve() == arguments.vision_evidence.resolve():
            raise CaptureError("offline output cannot overwrite sealed vision evidence")
        capture = build_capture(arguments)
        atomic_write_json(arguments.output, capture)
    except (CaptureError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print("OFFLINE_LIVE_MODEL_POSE_CAPTURE=PASS")
    print("OFFLINE_MODEL_STABILITY=PASS")
    print(f"LIVE_GROUND_TRUTH_EVIDENCE={arguments.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
