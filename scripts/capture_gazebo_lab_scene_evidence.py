#!/usr/bin/env python3
"""Turn bounded ``gz model -p`` responses into Phase-3 live-scene evidence.

This helper deliberately reads Gazebo responses captured by the runtime gate;
it does not read the generated SDF or manifest to infer where objects ought to
be.  That separation makes the resulting JSON useful when comparing two real
Gazebo runs with different deterministic scene seeds.
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
import sys
import tempfile
from typing import Any


EXPECTED_MODELS = (
    "ground_plane",
    "work_table",
    "drawing_board",
    "obstacle_left",
    "obstacle_right",
    "green_ball",
    "red_ball",
    "blue_cube",
    "yellow_cube",
    "abb_irb1200_gripper",
)
REQUIRED_METADATA_KEYS = (
    "TESTED_COMMIT",
    "SOURCE_WORKTREE_CLEAN",
    "WORKING_SOURCE_MANIFEST_SHA256",
    "ROS_DOMAIN_ID",
    "GZ_PARTITION",
    "P31_SCENE_SEED",
)
KEY_PATTERN = re.compile(r"^[A-Z][A-Z0-9_]*$")
MODEL_LINE_PATTERN = re.compile(r"^\s*-\s+([A-Za-z0-9_]+)\s*$", re.MULTILINE)
FLOAT_PATTERN = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
TRIPLE_PATTERN = re.compile(
    rf"\[\s*({FLOAT_PATTERN})\s+({FLOAT_PATTERN})\s+({FLOAT_PATTERN})\s*\]"
)
ANSI_ESCAPE_PATTERN = re.compile(r"\x1b\[[0-9;]*m")


class EvidenceError(ValueError):
    """Raised when claimed live evidence cannot be verified from its inputs."""


def read_text(path: Path, label: str) -> str:
    if not path.is_file():
        raise EvidenceError(f"{label} does not exist or is not a regular file: {path}")
    try:
        text = path.read_text(encoding="utf-8")
    except UnicodeDecodeError as exc:
        raise EvidenceError(f"{label} is not UTF-8 text: {path}") from exc
    if not text.strip():
        raise EvidenceError(f"{label} is empty: {path}")
    return text


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_key_values(path: Path, label: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for line_number, line in enumerate(read_text(path, label).splitlines(), start=1):
        if not line.strip():
            continue
        if "=" not in line:
            raise EvidenceError(f"{label} line {line_number} is not KEY=VALUE")
        key, value = line.split("=", maxsplit=1)
        if not KEY_PATTERN.fullmatch(key):
            raise EvidenceError(f"{label} line {line_number} has an invalid key: {key!r}")
        if not value:
            raise EvidenceError(f"{label} line {line_number} has an empty value for {key}")
        if key in values:
            raise EvidenceError(f"{label} contains duplicate key {key}")
        values[key] = value
    return values


def parse_seed(value: str, label: str) -> int:
    if not re.fullmatch(r"(?:0|[1-9][0-9]*)", value):
        raise EvidenceError(f"{label} must be a non-negative decimal integer")
    parsed = int(value)
    if parsed > 2_147_483_646:
        raise EvidenceError(f"{label} exceeds the supported Phase-3 seed range")
    return parsed


def require_finite_vector(value: tuple[str, str, str], label: str) -> list[float]:
    parsed = [float(item) for item in value]
    if not all(math.isfinite(item) for item in parsed):
        raise EvidenceError(f"{label} contains a non-finite value")
    if any(abs(item) > 1_000_000.0 for item in parsed):
        raise EvidenceError(f"{label} has an implausibly large value")
    return parsed


def parse_pose(path: Path, model_name: str) -> dict[str, Any]:
    text = ANSI_ESCAPE_PATTERN.sub("", read_text(path, f"Gazebo pose response for {model_name}"))
    lowered = text.lower()
    for failure_marker in (
        "no model named",
        "request timed out",
        "timed out",
        "failed to",
        "error [",
    ):
        if failure_marker in lowered:
            raise EvidenceError(
                f"Gazebo pose response for {model_name} contains failure marker "
                f"{failure_marker!r}"
            )
    if model_name not in text:
        raise EvidenceError(
            f"Gazebo pose response does not name the requested model {model_name}: {path}"
        )
    triples = TRIPLE_PATTERN.findall(text)
    if len(triples) < 2:
        raise EvidenceError(
            f"could not parse XYZ and RPY triples from Gazebo pose response for "
            f"{model_name}: {path}"
        )
    position = require_finite_vector(triples[0], f"{model_name} live position")
    rpy = require_finite_vector(triples[1], f"{model_name} live RPY")
    return {
        "position_m": position,
        "rpy_rad": rpy,
        "raw_pose_sha256": sha256_file(path),
        "raw_pose_triple_count": len(triples),
    }


def relative_pose_path(path: Path, evidence_output: Path) -> str:
    """Record a portable, confined provenance path beside the evidence JSON."""

    try:
        relative = path.resolve(strict=True).relative_to(
            evidence_output.parent.resolve(strict=True)
        )
    except ValueError as exc:
        raise EvidenceError(
            "each raw Gazebo pose response must be beneath the evidence output "
            f"directory: {path}"
        ) from exc
    if not relative.parts or any(part in {"", ".", ".."} for part in relative.parts):
        raise EvidenceError(f"raw Gazebo pose response has an unsafe relative path: {path}")
    return relative.as_posix()


def parse_model_list(path: Path, world_name: str) -> list[str]:
    text = ANSI_ESCAPE_PATTERN.sub("", read_text(path, "Gazebo model-list response"))
    if f"Requesting state for world [{world_name}]" not in text:
        raise EvidenceError(
            "Gazebo model-list response does not identify the expected world "
            f"{world_name!r}"
        )
    names = MODEL_LINE_PATTERN.findall(text)
    if len(names) != len(EXPECTED_MODELS) or set(names) != set(EXPECTED_MODELS):
        raise EvidenceError(
            "Gazebo model-list response does not contain exactly the required "
            f"Phase-3 models; observed {names}"
        )
    if len(set(names)) != len(names):
        raise EvidenceError("Gazebo model-list response contains duplicate model names")
    return names


def parse_pose_arguments(arguments: list[str]) -> dict[str, Path]:
    poses: dict[str, Path] = {}
    for argument in arguments:
        if "=" not in argument:
            raise EvidenceError("--pose must use MODEL=PATH")
        model_name, raw_path = argument.split("=", maxsplit=1)
        if model_name not in EXPECTED_MODELS:
            raise EvidenceError(f"--pose names an unexpected model: {model_name}")
        if not raw_path:
            raise EvidenceError(f"--pose has no response path for {model_name}")
        if model_name in poses:
            raise EvidenceError(f"--pose was supplied more than once for {model_name}")
        poses[model_name] = Path(raw_path)
    if set(poses) != set(EXPECTED_MODELS):
        missing = sorted(set(EXPECTED_MODELS) - set(poses))
        extra = sorted(set(poses) - set(EXPECTED_MODELS))
        raise EvidenceError(f"--pose inputs are incomplete; missing={missing}, extra={extra}")
    return poses


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


def build_evidence(arguments: argparse.Namespace) -> dict[str, Any]:
    seed = parse_seed(str(arguments.seed), "--seed")
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", arguments.world_name):
        raise EvidenceError("--world-name must be a Gazebo identifier")
    metadata = parse_key_values(arguments.run_metadata, "runtime metadata")
    for key in REQUIRED_METADATA_KEYS:
        if key not in metadata:
            raise EvidenceError(f"runtime metadata lacks required key {key}")
    metadata_seed = parse_seed(metadata["P31_SCENE_SEED"], "runtime metadata P31_SCENE_SEED")
    if metadata_seed != seed:
        raise EvidenceError(
            "runtime metadata P31_SCENE_SEED does not match the selected live-scene seed"
        )
    if not re.fullmatch(r"[0-9a-f]{40,64}", metadata["TESTED_COMMIT"]):
        raise EvidenceError("runtime metadata TESTED_COMMIT is not a Git object ID")
    if metadata["SOURCE_WORKTREE_CLEAN"] not in {"true", "false"}:
        raise EvidenceError("runtime metadata SOURCE_WORKTREE_CLEAN must be true or false")
    if not re.fullmatch(r"[0-9a-f]{64}", metadata["WORKING_SOURCE_MANIFEST_SHA256"]):
        raise EvidenceError("runtime metadata WORKING_SOURCE_MANIFEST_SHA256 is invalid")
    if not re.fullmatch(r"[1-9][0-9]*", metadata["ROS_DOMAIN_ID"]):
        raise EvidenceError("runtime metadata ROS_DOMAIN_ID is invalid")
    if not metadata["GZ_PARTITION"]:
        raise EvidenceError("runtime metadata GZ_PARTITION is empty")

    listed_names = parse_model_list(arguments.models_log, arguments.world_name)
    pose_paths = parse_pose_arguments(arguments.pose)
    models: dict[str, dict[str, Any]] = {}
    for name in EXPECTED_MODELS:
        record = parse_pose(pose_paths[name], name)
        record["raw_pose_relative_path"] = relative_pose_path(
            pose_paths[name], arguments.output
        )
        models[name] = record
    return {
        "schema_version": 1,
        "evidence_kind": "gazebo_live_scene_pose",
        "scope": "SIMULATION_ONLY",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "world_name": arguments.world_name,
        "selected_seed": seed,
        "expected_models": list(EXPECTED_MODELS),
        "observed_model_order": listed_names,
        "run_identity": {
            key: metadata[key] for key in REQUIRED_METADATA_KEYS
        },
        "source_files": {
            "models_log_sha256": sha256_file(arguments.models_log),
        },
        "models": models,
        "status": {
            "gazebo_model_list": "PASS",
            "all_expected_models_present": "PASS",
            "gazebo_live_model_pose_capture": "PASS",
        },
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", required=True)
    parser.add_argument("--world-name", default="manipulation_lab")
    parser.add_argument("--run-metadata", required=True, type=Path)
    parser.add_argument("--models-log", required=True, type=Path)
    parser.add_argument("--pose", action="append", default=[], metavar="MODEL=PATH")
    parser.add_argument("--output", required=True, type=Path)
    return parser


def main() -> int:
    arguments = build_parser().parse_args()
    try:
        evidence = build_evidence(arguments)
        atomic_write_json(arguments.output, evidence)
    except (EvidenceError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print("GAZEBO_LIVE_SCENE_POSES=PASS")
    print("LAB_OBJECTS_PRESENT=PASS")
    print(f"LIVE_SCENE_SELECTED_SEED={evidence['selected_seed']}")
    print(f"LIVE_SCENE_EVIDENCE={arguments.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
