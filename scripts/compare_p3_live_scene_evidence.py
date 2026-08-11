#!/usr/bin/env python3
"""Compare two completed P3 runtime outputs using live Gazebo pose evidence.

The comparison intentionally refuses to inspect generated SDF or scene
manifest files.  It accepts only two completed runtime output roots and their
captured ``gz model -p`` evidence, so a PASS demonstrates real seeded Gazebo
state variation rather than merely different source-generated artifacts.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
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
SEEDED_MODELS = (
    "obstacle_left",
    "obstacle_right",
    "green_ball",
    "red_ball",
    "blue_cube",
    "yellow_cube",
)
PRIMARY_POSITION_WITNESSES = ("green_ball", "blue_cube", "yellow_cube")
DIRECTIONAL_CUBES = ("blue_cube", "yellow_cube")
REQUIRED_RESULT_STATUS = {
    "P3_1": "PASS",
    "PROJECT_ABB_GRIPPER_GAZEBO_SPAWN": "PASS",
    "GRIPPER_HARDWARE_RESOURCES": "PASS",
    "CONTROLLERS_ACTIVE": "PASS",
    "LAB_OBJECTS_PRESENT": "PASS",
    "LAB_SCENE_SEED_REPRODUCIBLE": "PASS",
    "GRIPPER_OPEN_CLOSE_OPEN": "PASS",
    "GRIPPER_MIRROR_STATE": "PASS",
    "ABB_GRIPPER_LAB_HEADLESS_RUNTIME": "PASS",
    "CLEANUP_VERIFIED": "PASS",
}
IDENTITY_KEYS = (
    "TESTED_COMMIT",
    "SOURCE_WORKTREE_CLEAN",
    "WORKING_SOURCE_MANIFEST_SHA256",
    "P31_SCENE_SEED",
)
KEY_PATTERN = re.compile(r"^[A-Z][A-Z0-9_]*$")
HEX_SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
GIT_OBJECT_PATTERN = re.compile(r"^[0-9a-f]{40,64}$")
ANSI_ESCAPE_PATTERN = re.compile(r"\x1b\[[0-9;]*m")
MODEL_LINE_PATTERN = re.compile(r"^\s*-\s+([A-Za-z0-9_]+)\s*$", re.MULTILINE)
FLOAT_PATTERN = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
TRIPLE_PATTERN = re.compile(
    rf"\[\s*({FLOAT_PATTERN})\s+({FLOAT_PATTERN})\s+({FLOAT_PATTERN})\s*\]"
)


class ComparisonError(ValueError):
    """Raised when either alleged runtime output is incomplete or inconsistent."""


@dataclass(frozen=True)
class CompletedRun:
    root: Path
    seed: int
    tested_commit: str
    source_manifest_sha256: str
    evidence: dict[str, Any]
    models: dict[str, dict[str, Any]]


def read_text(path: Path, label: str) -> str:
    if not path.is_file():
        raise ComparisonError(f"{label} does not exist or is not a regular file: {path}")
    try:
        text = path.read_text(encoding="utf-8")
    except UnicodeDecodeError as exc:
        raise ComparisonError(f"{label} is not UTF-8 text: {path}") from exc
    if not text.strip():
        raise ComparisonError(f"{label} is empty: {path}")
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
            raise ComparisonError(f"{label} line {line_number} is not KEY=VALUE")
        key, value = line.split("=", maxsplit=1)
        if not KEY_PATTERN.fullmatch(key):
            raise ComparisonError(f"{label} line {line_number} has invalid key {key!r}")
        if not value:
            raise ComparisonError(f"{label} line {line_number} has empty value for {key}")
        if key in values:
            raise ComparisonError(f"{label} contains duplicate key {key}")
        values[key] = value
    return values


def parse_nonnegative_seed(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, str)):
        raise ComparisonError(f"{label} must be a non-negative integer")
    text = str(value)
    if not re.fullmatch(r"(?:0|[1-9][0-9]*)", text):
        raise ComparisonError(f"{label} must be a non-negative decimal integer")
    parsed = int(text)
    if parsed > 2_147_483_646:
        raise ComparisonError(f"{label} exceeds the supported Phase-3 seed range")
    return parsed


def require_mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ComparisonError(f"{label} must be a JSON object")
    return value


def finite_vector(value: Any, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ComparisonError(f"{label} must be a three-value JSON array")
    parsed: list[float] = []
    for index, item in enumerate(value):
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ComparisonError(f"{label}[{index}] must be numeric")
        number = float(item)
        if not math.isfinite(number) or abs(number) > 1_000_000.0:
            raise ComparisonError(f"{label}[{index}] is not a plausible finite value")
        parsed.append(number)
    return parsed


def finite_raw_vector(value: tuple[str, str, str], label: str) -> list[float]:
    parsed = [float(item) for item in value]
    if not all(math.isfinite(item) and abs(item) <= 1_000_000.0 for item in parsed):
        raise ComparisonError(f"{label} is not a plausible finite vector")
    return parsed


def parse_live_model_list(path: Path) -> None:
    text = ANSI_ESCAPE_PATTERN.sub("", read_text(path, "post-action Gazebo model list"))
    if "Requesting state for world [manipulation_lab]" not in text:
        raise ComparisonError("post-action Gazebo model list has the wrong or absent world")
    names = MODEL_LINE_PATTERN.findall(text)
    if len(names) != len(EXPECTED_MODELS) or set(names) != set(EXPECTED_MODELS):
        raise ComparisonError(
            "post-action Gazebo model list does not contain exactly the expected models"
        )
    if len(set(names)) != len(names):
        raise ComparisonError("post-action Gazebo model list contains duplicate model names")


def parse_live_model_pose(path: Path, model_name: str) -> tuple[list[float], list[float], int]:
    text = ANSI_ESCAPE_PATTERN.sub("", read_text(path, f"raw pose response for {model_name}"))
    lowered = text.lower()
    for failure_marker in (
        "no model named",
        "request timed out",
        "timed out",
        "failed to",
        "error [",
    ):
        if failure_marker in lowered:
            raise ComparisonError(
                f"raw pose response for {model_name} contains failure marker {failure_marker!r}"
            )
    if model_name not in text:
        raise ComparisonError(f"raw pose response does not name requested model {model_name}")
    triples = TRIPLE_PATTERN.findall(text)
    if len(triples) < 2:
        raise ComparisonError(
            f"raw pose response for {model_name} has no complete XYZ/RPY observation"
        )
    return (
        finite_raw_vector(triples[0], f"raw {model_name} position"),
        finite_raw_vector(triples[1], f"raw {model_name} RPY"),
        len(triples),
    )


def require_close_vectors(
    left: list[float], right: list[float], label: str, *, tolerance: float = 1e-12
) -> None:
    if len(left) != len(right) or any(
        not math.isclose(left[index], right[index], rel_tol=0.0, abs_tol=tolerance)
        for index in range(len(left))
    ):
        raise ComparisonError(f"{label} does not match the re-parsed raw Gazebo response")


def load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        document = json.loads(read_text(path, label))
    except json.JSONDecodeError as exc:
        raise ComparisonError(f"{label} is not valid JSON: {path}") from exc
    return require_mapping(document, label)


def require_equal(left: Any, right: Any, label: str) -> None:
    if left != right:
        raise ComparisonError(f"{label} does not agree: {left!r} != {right!r}")


def load_completed_run(output_root: Path) -> CompletedRun:
    root = output_root.expanduser().resolve(strict=True)
    if not root.is_dir():
        raise ComparisonError(f"runtime output root is not a directory: {root}")
    result = parse_key_values(root / "result.txt", "runtime result")
    metadata = parse_key_values(root / "run_metadata.txt", "runtime metadata")
    for key, expected_value in REQUIRED_RESULT_STATUS.items():
        if result.get(key) != expected_value:
            raise ComparisonError(
                f"runtime result {root} has {key}={result.get(key)!r}; "
                f"requires {expected_value!r}"
            )
    for key in IDENTITY_KEYS:
        if key not in result or key not in metadata:
            raise ComparisonError(f"runtime output {root} lacks identity key {key}")
        require_equal(result[key], metadata[key], f"result and metadata {key}")
    if result["SOURCE_WORKTREE_CLEAN"] != "true":
        raise ComparisonError(f"runtime output {root} was not run from a clean source worktree")
    if not GIT_OBJECT_PATTERN.fullmatch(result["TESTED_COMMIT"]):
        raise ComparisonError(f"runtime output {root} has an invalid TESTED_COMMIT")
    if not HEX_SHA256_PATTERN.fullmatch(result["WORKING_SOURCE_MANIFEST_SHA256"]):
        raise ComparisonError(
            f"runtime output {root} has an invalid WORKING_SOURCE_MANIFEST_SHA256"
        )
    seed = parse_nonnegative_seed(result["P31_SCENE_SEED"], "P31_SCENE_SEED")

    evidence_path = root / "runtime" / "live_scene_evidence.json"
    evidence = load_json(evidence_path, "live Gazebo scene evidence")
    if evidence.get("schema_version") != 1:
        raise ComparisonError("live Gazebo scene evidence has an unsupported schema_version")
    if evidence.get("evidence_kind") != "gazebo_live_scene_pose":
        raise ComparisonError("live Gazebo scene evidence has the wrong evidence_kind")
    if evidence.get("scope") != "SIMULATION_ONLY":
        raise ComparisonError("live Gazebo scene evidence has the wrong scope")
    if evidence.get("world_name") != "manipulation_lab":
        raise ComparisonError("live Gazebo scene evidence has the wrong world_name")
    require_equal(
        parse_nonnegative_seed(evidence.get("selected_seed"), "live evidence selected_seed"),
        seed,
        "live evidence selected_seed and result P31_SCENE_SEED",
    )
    expected_models = evidence.get("expected_models")
    if expected_models != list(EXPECTED_MODELS):
        raise ComparisonError("live Gazebo scene evidence has unexpected expected_models")
    observed_order = evidence.get("observed_model_order")
    if not isinstance(observed_order, list) or set(observed_order) != set(EXPECTED_MODELS):
        raise ComparisonError("live Gazebo scene evidence has an incomplete model-list observation")
    status = require_mapping(evidence.get("status"), "live Gazebo scene evidence status")
    for key in (
        "gazebo_model_list",
        "all_expected_models_present",
        "gazebo_live_model_pose_capture",
    ):
        if status.get(key) != "PASS":
            raise ComparisonError(f"live Gazebo scene evidence status {key} is not PASS")
    run_identity = require_mapping(evidence.get("run_identity"), "live evidence run_identity")
    for key in IDENTITY_KEYS:
        if key not in run_identity:
            raise ComparisonError(f"live evidence run_identity lacks {key}")
        require_equal(run_identity[key], result[key], f"live evidence run_identity {key}")
    source_files = require_mapping(evidence.get("source_files"), "live evidence source_files")
    model_list_digest = source_files.get("models_log_sha256")
    if not isinstance(model_list_digest, str) or not HEX_SHA256_PATTERN.fullmatch(
        model_list_digest
    ):
        raise ComparisonError("live evidence has invalid models_log_sha256")
    raw_model_list = root / "runtime" / "models_after_actions.txt"
    if sha256_file(raw_model_list) != model_list_digest:
        raise ComparisonError("post-action Gazebo model-list hash does not match live evidence")
    parse_live_model_list(raw_model_list)
    models = require_mapping(evidence.get("models"), "live Gazebo scene evidence models")
    if set(models) != set(EXPECTED_MODELS):
        raise ComparisonError("live Gazebo scene evidence does not contain exactly the expected models")
    for name in EXPECTED_MODELS:
        model = require_mapping(models[name], f"live evidence model {name}")
        model["position_m"] = finite_vector(model.get("position_m"), f"{name}.position_m")
        model["rpy_rad"] = finite_vector(model.get("rpy_rad"), f"{name}.rpy_rad")
        digest = model.get("raw_pose_sha256")
        if not isinstance(digest, str) or not HEX_SHA256_PATTERN.fullmatch(digest):
            raise ComparisonError(f"live evidence model {name} has invalid raw_pose_sha256")
        expected_relative_path = f"model_poses/{name}.txt"
        if model.get("raw_pose_relative_path") != expected_relative_path:
            raise ComparisonError(
                f"live evidence model {name} has an unexpected raw pose provenance path"
            )
        raw_pose_path = root / "runtime" / expected_relative_path
        try:
            raw_pose_path.resolve(strict=True).relative_to(root)
        except ValueError as exc:
            raise ComparisonError(f"raw pose response for {name} escapes its output root") from exc
        if sha256_file(raw_pose_path) != digest:
            raise ComparisonError(f"raw pose hash for {name} does not match live evidence")
        raw_position, raw_rpy, raw_triple_count = parse_live_model_pose(
            raw_pose_path, name
        )
        require_close_vectors(
            model["position_m"], raw_position, f"live evidence {name}.position_m"
        )
        require_close_vectors(model["rpy_rad"], raw_rpy, f"live evidence {name}.rpy_rad")
        if model.get("raw_pose_triple_count") != raw_triple_count:
            raise ComparisonError(
                f"live evidence model {name} raw_pose_triple_count does not match raw response"
            )
    return CompletedRun(
        root=root,
        seed=seed,
        tested_commit=result["TESTED_COMMIT"],
        source_manifest_sha256=result["WORKING_SOURCE_MANIFEST_SHA256"],
        evidence=evidence,
        models=models,
    )


def euclidean_distance(left: list[float], right: list[float]) -> float:
    return math.sqrt(sum((right[index] - left[index]) ** 2 for index in range(3)))


def angular_distance(left: float, right: float) -> float:
    return abs((left - right + math.pi) % (2.0 * math.pi) - math.pi)


def finite_nonnegative(value: float, label: str) -> float:
    if not math.isfinite(value) or value < 0.0:
        raise ComparisonError(f"{label} must be a finite non-negative number")
    return value


def compare_runs(
    first: CompletedRun,
    second: CompletedRun,
    minimum_position_delta_m: float,
    minimum_yaw_delta_rad: float,
) -> dict[str, Any]:
    if first.root == second.root:
        raise ComparisonError("the two runtime output roots must be different")
    if first.seed == second.seed:
        raise ComparisonError("the two runtime outputs use the same selected seed")
    require_equal(first.tested_commit, second.tested_commit, "TESTED_COMMIT")
    require_equal(
        first.source_manifest_sha256,
        second.source_manifest_sha256,
        "WORKING_SOURCE_MANIFEST_SHA256",
    )
    minimum_position_delta_m = finite_nonnegative(
        minimum_position_delta_m, "--minimum-position-delta-m"
    )
    minimum_yaw_delta_rad = finite_nonnegative(
        minimum_yaw_delta_rad, "--minimum-yaw-delta-rad"
    )
    if minimum_position_delta_m <= 0.0 or minimum_yaw_delta_rad <= 0.0:
        raise ComparisonError("comparison thresholds must be greater than zero")

    position_deltas = {
        name: euclidean_distance(
            first.models[name]["position_m"], second.models[name]["position_m"]
        )
        for name in SEEDED_MODELS
    }
    moved_seeded_models = sorted(
        name
        for name, delta in position_deltas.items()
        if delta >= minimum_position_delta_m
    )
    primary_position_witnesses = sorted(
        name for name in PRIMARY_POSITION_WITNESSES if name in moved_seeded_models
    )
    if len(moved_seeded_models) < 2:
        raise ComparisonError(
            "live Gazebo evidence shows fewer than two materially moved seeded models; "
            f"deltas={position_deltas}"
        )
    if not primary_position_witnesses:
        raise ComparisonError(
            "live Gazebo evidence has no materially moved green ball or cube; "
            f"deltas={position_deltas}"
        )

    cube_yaw_deltas = {
        name: angular_distance(
            first.models[name]["rpy_rad"][2], second.models[name]["rpy_rad"][2]
        )
        for name in DIRECTIONAL_CUBES
    }
    oriented_cubes = sorted(
        name for name, delta in cube_yaw_deltas.items() if delta >= minimum_yaw_delta_rad
    )
    if not oriented_cubes:
        raise ComparisonError(
            "live Gazebo evidence has no meaningful cube yaw variation; "
            f"deltas={cube_yaw_deltas}"
        )

    return {
        "schema_version": 1,
        "evidence_kind": "p3_live_seed_comparison",
        "scope": "SIMULATION_ONLY",
        "compared_at_utc": datetime.now(timezone.utc).isoformat(),
        "world_name": "manipulation_lab",
        "tested_commit": first.tested_commit,
        "working_source_manifest_sha256": first.source_manifest_sha256,
        "runs": [
            {
                "output_root": str(first.root),
                "selected_seed": first.seed,
                "live_scene_evidence": str(first.root / "runtime" / "live_scene_evidence.json"),
            },
            {
                "output_root": str(second.root),
                "selected_seed": second.seed,
                "live_scene_evidence": str(second.root / "runtime" / "live_scene_evidence.json"),
            },
        ],
        "thresholds": {
            "minimum_position_delta_m": minimum_position_delta_m,
            "minimum_yaw_delta_rad": minimum_yaw_delta_rad,
        },
        "live_position_delta_m": position_deltas,
        "live_cube_yaw_delta_rad": cube_yaw_deltas,
        "witnesses": {
            "materially_moved_seeded_models": moved_seeded_models,
            "primary_position_witnesses": primary_position_witnesses,
            "meaningfully_reoriented_cubes": oriented_cubes,
        },
        "status": {
            "same_clean_commit": "PASS",
            "distinct_selected_seeds": "PASS",
            "live_seeded_object_position_variation": "PASS",
            "live_directional_cube_orientation_variation": "PASS",
        },
    }


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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--first-output-root", required=True, type=Path)
    parser.add_argument("--second-output-root", required=True, type=Path)
    parser.add_argument("--minimum-position-delta-m", type=float, default=0.02)
    parser.add_argument("--minimum-yaw-delta-rad", type=float, default=0.15)
    parser.add_argument(
        "--evidence",
        type=Path,
        help="optional output JSON path; without it, JSON is written to stdout",
    )
    return parser


def main() -> int:
    arguments = build_parser().parse_args()
    try:
        first = load_completed_run(arguments.first_output_root)
        second = load_completed_run(arguments.second_output_root)
        comparison = compare_runs(
            first,
            second,
            arguments.minimum_position_delta_m,
            arguments.minimum_yaw_delta_rad,
        )
        if arguments.evidence is None:
            print(json.dumps(comparison, indent=2, sort_keys=True))
        else:
            atomic_write_json(arguments.evidence, comparison)
            print("P3_LIVE_SCENE_SEED_COMPARISON=PASS")
            print("P3_LIVE_OBJECT_POSITION_VARIATION=PASS")
            print("P3_LIVE_CUBE_ORIENTATION_VARIATION=PASS")
            print(f"P3_LIVE_SCENE_COMPARISON_EVIDENCE={arguments.evidence.resolve()}")
    except (ComparisonError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
