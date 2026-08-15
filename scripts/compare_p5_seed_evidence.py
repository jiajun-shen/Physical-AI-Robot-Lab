#!/usr/bin/env python3
"""Fail-closed comparison of two completed Phase-5 seed-run evidence bundles.

Each --*-run directory must have the following harness-owned layout:

    <run>/
      run_metadata.txt                       # P5_SCENE_SEED and GZ_PARTITION
      pre_pick/evidence/vision_evidence.json
      execution/execution_evidence.json
      post_place/evidence/vision_evidence.json
      evidence/p5_evaluation.json            # Phase-5 evaluator output

This tool is deliberately offline: it never starts or queries ROS, Gazebo,
MoveIt, a world file, or a ground-truth service.  It re-hashes each referenced
input, verifies that the Phase-5 evaluator passed, extracts the green-ball
position directly from the sealed pre-pick Phase-4 vision artifact, and only
then accepts the two seed runs as distinct evidence.
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
import sys
import tempfile
from typing import Any


P4_CLASSES = (
    "green_ball",
    "red_ball",
    "blue_cube",
    "yellow_cube",
    "drawing_board",
)
P4_ARTIFACTS = frozenset(
    {
        "evidence/rgb.png",
        "evidence/annotated_rgb.png",
        "evidence/depth.npy",
        "evidence/camera_info.json",
        "evidence/tf_chain.json",
        "evidence/vision_outputs.json",
    }
)
P5_EVALUATOR_MARKERS = (
    "P5_PICK_PLACE_EVALUATION",
    "P5_STAGE_SEQUENCE",
    "P5_PHASE4_VISION_PROVENANCE",
    "P5_MOVEIT_MOTION",
    "P5_GRASP_RETENTION",
    "P5_RELEASE",
    "P5_POST_PLACE",
)
P4_VISION_MARKERS = (
    "p4_rgb_transport",
    "p4_depth_transport",
    "p4_camera_info",
    "p4_tf_chain",
    "p4_all_five_typed_outputs",
    "p4_vision_input_boundary",
)


class ComparisonError(RuntimeError):
    """Raised when a required two-seed evidence boundary is absent."""


@dataclass(frozen=True)
class RunSummary:
    root: Path
    seed: int
    gazebo_partition: str
    pre_pick_evidence_sha256: str
    execution_evidence_sha256: str
    post_place_evidence_sha256: str
    pre_pick_green_vision_position_m: tuple[float, float, float]
    pre_pick_green_source_stamp_ns: int

    def as_dict(self) -> dict[str, Any]:
        return {
            "root": str(self.root),
            "seed": self.seed,
            "gazebo_partition": self.gazebo_partition,
            "pre_pick_evidence_sha256": self.pre_pick_evidence_sha256,
            "execution_evidence_sha256": self.execution_evidence_sha256,
            "post_place_evidence_sha256": self.post_place_evidence_sha256,
            "pre_pick_green_vision_position_m": list(self.pre_pick_green_vision_position_m),
            "pre_pick_green_source_stamp_ns": self.pre_pick_green_source_stamp_ns,
        }


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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


def load_json(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file():
        raise ComparisonError(f"{label} is missing: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ComparisonError(f"cannot parse {label}: {path}") from exc
    return require_mapping(payload, label)


def require_mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ComparisonError(f"{label} must be a JSON object")
    return value


def require_list(value: Any, label: str) -> list[Any]:
    if not isinstance(value, list):
        raise ComparisonError(f"{label} must be a JSON list")
    return value


def require_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ComparisonError(f"{label} must be a non-empty string")
    return value


def require_int(value: Any, label: str, *, minimum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ComparisonError(f"{label} must be an integer")
    if minimum is not None and value < minimum:
        raise ComparisonError(f"{label} must be at least {minimum}")
    return value


def require_bool(value: Any, label: str) -> bool:
    if not isinstance(value, bool):
        raise ComparisonError(f"{label} must be a boolean")
    return value


def require_vector(value: Any, label: str) -> tuple[float, float, float]:
    entries = require_list(value, label)
    if len(entries) != 3:
        raise ComparisonError(f"{label} must have exactly three values")
    result: list[float] = []
    for index, item in enumerate(entries):
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ComparisonError(f"{label}[{index}] must be a number")
        number = float(item)
        if not math.isfinite(number):
            raise ComparisonError(f"{label}[{index}] must be finite")
        result.append(number)
    return (result[0], result[1], result[2])


def require_sha256(value: Any, label: str) -> str:
    digest = require_string(value, label)
    if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
        raise ComparisonError(f"{label} must be a lowercase SHA-256 digest")
    return digest


def resolve_run_root(root: Path, label: str) -> Path:
    try:
        resolved = root.resolve(strict=True)
    except OSError as exc:
        raise ComparisonError(f"{label} cannot be resolved: {root}") from exc
    if not resolved.is_dir():
        raise ComparisonError(f"{label} is not a directory: {resolved}")
    return resolved


def parse_metadata(path: Path) -> dict[str, str]:
    if not path.is_file():
        raise ComparisonError(f"run metadata is missing: {path}")
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise ComparisonError(f"cannot read run metadata: {path}") from exc
    metadata: dict[str, str] = {}
    for line_number, line in enumerate(lines, start=1):
        if not line or "=" not in line:
            raise ComparisonError(f"invalid metadata line {line_number} in {path}: {line!r}")
        key, value = line.split("=", 1)
        if not key or not value or key in metadata:
            raise ComparisonError(f"invalid metadata line {line_number} in {path}: {line!r}")
        metadata[key] = value
    return metadata


def expected_paths(root: Path) -> tuple[Path, Path, Path, Path]:
    return (
        root / "pre_pick" / "evidence" / "vision_evidence.json",
        root / "execution" / "execution_evidence.json",
        root / "post_place" / "evidence" / "vision_evidence.json",
        root / "evidence" / "p5_evaluation.json",
    )


def validate_p5_evaluator(
    evaluation_path: Path,
    *,
    seed: int,
    pre_pick_path: Path,
    execution_path: Path,
    post_place_path: Path,
) -> tuple[str, str, str]:
    """Validate the sealed evaluator output against its concrete input files."""

    evaluation = load_json(evaluation_path, "P5 evaluator evidence")
    if require_int(evaluation.get("schema_version"), "P5 evaluator schema_version") != 1:
        raise ComparisonError("P5 evaluator has an unsupported schema")
    if evaluation.get("scope") != "SIMULATION_ONLY":
        raise ComparisonError("P5 evaluator is outside the simulation-only scope")
    if evaluation.get("evaluation_mode") != "OFFLINE_EVIDENCE_VALIDATION_ONLY":
        raise ComparisonError("P5 evaluator has an unexpected evaluation mode")
    if require_int(evaluation.get("seed"), "P5 evaluator seed", minimum=0) != seed:
        raise ComparisonError("P5 evaluator seed disagrees with P5_SCENE_SEED")
    if evaluation.get("status") != "PASS":
        raise ComparisonError("P5 evaluator did not pass")
    markers = require_mapping(evaluation.get("markers"), "P5 evaluator markers")
    for marker in P5_EVALUATOR_MARKERS:
        if markers.get(marker) != "PASS":
            raise ComparisonError(f"P5 evaluator has not passed {marker}")

    inputs = require_mapping(evaluation.get("inputs"), "P5 evaluator inputs")
    validated_hashes: list[str] = []
    for input_name, expected_path in (
        ("pre_pick_evidence", pre_pick_path),
        ("execution_evidence", execution_path),
        ("post_place_evidence", post_place_path),
    ):
        record = require_mapping(inputs.get(input_name), f"P5 evaluator inputs.{input_name}")
        recorded_path = Path(require_string(record.get("path"), f"{input_name}.path"))
        try:
            resolved_recorded = recorded_path.resolve(strict=True)
        except OSError as exc:
            raise ComparisonError(f"{input_name}.path cannot be resolved") from exc
        if resolved_recorded != expected_path.resolve():
            raise ComparisonError(f"{input_name}.path is outside its run directory contract")
        expected_hash = require_sha256(record.get("sha256"), f"{input_name}.sha256")
        if not expected_path.is_file():
            raise ComparisonError(f"{input_name} is missing: {expected_path}")
        actual_hash = sha256_file(expected_path)
        if actual_hash != expected_hash:
            raise ComparisonError(f"{input_name} hash does not match the sealed evaluator record")
        validated_hashes.append(actual_hash)
    return (validated_hashes[0], validated_hashes[1], validated_hashes[2])


def validate_phase4_vision_seal(
    evidence_path: Path, label: str
) -> tuple[dict[str, Any], int]:
    """Validate one sealed Phase-4 vision bundle without querying any runtime."""

    evidence = load_json(evidence_path, label)
    if require_int(evidence.get("schema_version"), f"{label} schema_version") != 1:
        raise ComparisonError(f"{label} has an unsupported schema")
    if evidence.get("scope") != "SIMULATION_ONLY":
        raise ComparisonError(f"{label} is outside the simulation-only scope")
    if evidence.get("perception_mode") != "vision":
        raise ComparisonError(f"{label} is not vision-only")
    if require_bool(evidence.get("ground_truth_input_used"), f"{label} ground_truth_input_used"):
        raise ComparisonError(f"{label} reports ground-truth input use")
    if require_list(evidence.get("expected_classes"), f"{label} expected_classes") != list(P4_CLASSES):
        raise ComparisonError(f"{label} has unexpected expected_classes")
    sealed_source_header = require_mapping(
        evidence.get("source_header"), f"{label} source_header"
    )
    sealed_source_stamp = require_int(
        sealed_source_header.get("stamp_ns"), f"{label} source_header.stamp_ns", minimum=1
    )
    if sealed_source_header.get("frame_id") != "lab_camera_optical_frame":
        raise ComparisonError(f"{label} has an unexpected source camera frame")
    statuses = require_mapping(evidence.get("status"), f"{label} status")
    for marker in P4_VISION_MARKERS:
        if statuses.get(marker) != "PASS":
            raise ComparisonError(f"{label} has not passed {marker}")

    artifact_hashes = require_mapping(
        evidence.get("artifact_sha256"), f"{label} artifact_sha256"
    )
    if set(artifact_hashes) != P4_ARTIFACTS:
        raise ComparisonError(f"{label} has incomplete or unexpected Phase-4 artifact hashes")
    phase4_root = evidence_path.parent.parent
    for relative in P4_ARTIFACTS:
        artifact = phase4_root / Path(*relative.split("/"))
        expected_hash = require_sha256(
            artifact_hashes.get(relative), f"{label} artifact_sha256[{relative!r}]"
        )
        if not artifact.is_file() or sha256_file(artifact) != expected_hash:
            raise ComparisonError(f"{label} artifact hash does not match: {relative}")
    return evidence, sealed_source_stamp


def extract_pre_pick_green(pre_pick_path: Path) -> tuple[tuple[float, float, float], int]:
    """Extract one explicitly vision-sourced world-frame green-ball pose."""

    _, sealed_source_stamp = validate_phase4_vision_seal(
        pre_pick_path, "pre-pick Phase-4 evidence"
    )
    outputs_path = pre_pick_path.parent / "vision_outputs.json"
    outputs = load_json(outputs_path, "pre-pick vision outputs")
    green_entries: list[dict[str, Any]] = []
    for index, raw_pose in enumerate(require_list(outputs.get("world_poses"), "pre-pick world_poses")):
        pose = require_mapping(raw_pose, f"pre-pick world_poses[{index}]")
        if pose.get("class_id") == "green_ball":
            green_entries.append(pose)
    if len(green_entries) != 1:
        raise ComparisonError("pre-pick world_poses must contain exactly one green_ball")
    green = green_entries[0]
    if require_int(green.get("source_mode"), "pre-pick green source_mode") != 1:
        raise ComparisonError("pre-pick green pose is not vision-sourced")
    header = require_mapping(green.get("header"), "pre-pick green header")
    if header.get("frame_id") != "world":
        raise ComparisonError("pre-pick green pose is not expressed in the world frame")
    if require_int(header.get("stamp_ns"), "pre-pick green header.stamp_ns") != sealed_source_stamp:
        raise ComparisonError("pre-pick green pose header does not match the sealed vision epoch")
    source_header = require_mapping(green.get("source_header"), "pre-pick green source_header")
    source_stamp = require_int(
        source_header.get("stamp_ns"), "pre-pick green source_header.stamp_ns", minimum=1
    )
    if source_header.get("frame_id") != "lab_camera_optical_frame":
        raise ComparisonError("pre-pick green source frame is not the lab camera")
    if source_stamp != sealed_source_stamp:
        raise ComparisonError("pre-pick green source stamp does not match the sealed vision epoch")
    return (
        require_vector(green.get("position_m"), "pre-pick green position_m"),
        source_stamp,
    )


def validate_execution_provenance(execution_path: Path, seed: int) -> None:
    execution = load_json(execution_path, "execution evidence")
    if require_int(execution.get("schema_version"), "execution schema_version") != 1:
        raise ComparisonError("execution evidence has an unsupported schema")
    if execution.get("scope") != "SIMULATION_ONLY":
        raise ComparisonError("execution evidence is outside the simulation-only scope")
    if require_int(execution.get("seed"), "execution seed", minimum=0) != seed:
        raise ComparisonError("execution seed disagrees with P5_SCENE_SEED")
    if require_bool(
        execution.get("online_ground_truth_input_used"),
        "execution online_ground_truth_input_used",
    ):
        raise ComparisonError("execution evidence reports online ground-truth input use")
    if execution.get("status") != "PENDING_POST_PLACE_VISION_EVALUATION":
        raise ComparisonError("execution evidence does not have the expected pending post-place status")


def run_summary(root: Path) -> RunSummary:
    resolved_root = resolve_run_root(root, "run directory")
    metadata = parse_metadata(resolved_root / "run_metadata.txt")
    seed_text = metadata.get("P5_SCENE_SEED")
    if seed_text is None or not seed_text.isdecimal():
        raise ComparisonError(
            f"run metadata has no valid P5_SCENE_SEED: {resolved_root / 'run_metadata.txt'}"
        )
    seed = int(seed_text)
    gazebo_partition = require_string(
        metadata.get("GZ_PARTITION"), "run metadata GZ_PARTITION"
    )
    pre_pick_path, execution_path, post_place_path, evaluation_path = expected_paths(resolved_root)
    resolved_inputs = tuple(path.resolve() for path in (pre_pick_path, execution_path, post_place_path))
    if len(set(resolved_inputs)) != 3:
        raise ComparisonError("pre-pick, execution, and post-place artifacts must be distinct")
    pre_hash, execution_hash, post_hash = validate_p5_evaluator(
        evaluation_path,
        seed=seed,
        pre_pick_path=pre_pick_path,
        execution_path=execution_path,
        post_place_path=post_place_path,
    )
    if len({pre_hash, execution_hash, post_hash}) != 3:
        raise ComparisonError("pre-pick, execution, and post-place hashes must be distinct")
    validate_execution_provenance(execution_path, seed)
    validate_phase4_vision_seal(post_place_path, "post-place Phase-4 evidence")
    green_position, source_stamp = extract_pre_pick_green(pre_pick_path)
    return RunSummary(
        root=resolved_root,
        seed=seed,
        gazebo_partition=gazebo_partition,
        pre_pick_evidence_sha256=pre_hash,
        execution_evidence_sha256=execution_hash,
        post_place_evidence_sha256=post_hash,
        pre_pick_green_vision_position_m=green_position,
        pre_pick_green_source_stamp_ns=source_stamp,
    )


def build_comparison(first_root: Path, second_root: Path) -> dict[str, Any]:
    first_resolved = resolve_run_root(first_root, "first run directory")
    second_resolved = resolve_run_root(second_root, "second run directory")
    if first_resolved == second_resolved:
        raise ComparisonError("the two inputs resolve to the same run directory")

    first = run_summary(first_resolved)
    second = run_summary(second_resolved)
    if first.seed == second.seed:
        raise ComparisonError("the two completed runs use the same selected seed")
    if first.gazebo_partition == second.gazebo_partition:
        raise ComparisonError("the two completed runs reuse the same GZ_PARTITION")

    for field in (
        "pre_pick_evidence_sha256",
        "execution_evidence_sha256",
        "post_place_evidence_sha256",
    ):
        if getattr(first, field) == getattr(second, field):
            readable = field.removesuffix("_sha256").replace("_", "-")
            raise ComparisonError(f"the two runs reuse the same {readable} hash")
    all_hashes = (
        first.pre_pick_evidence_sha256,
        first.execution_evidence_sha256,
        first.post_place_evidence_sha256,
        second.pre_pick_evidence_sha256,
        second.execution_evidence_sha256,
        second.post_place_evidence_sha256,
    )
    if len(set(all_hashes)) != len(all_hashes):
        raise ComparisonError("the two runs reuse an input artifact hash")

    green_delta = math.sqrt(
        sum(
            (first.pre_pick_green_vision_position_m[index]
             - second.pre_pick_green_vision_position_m[index])
            ** 2
            for index in range(3)
        )
    )
    if green_delta + 1e-12 < 0.05:
        raise ComparisonError(
            "pre-pick vision-derived green positions vary only "
            f"{green_delta:.6f} m; requires at least 0.05 m"
        )
    reported_green_delta = round(green_delta, 12)

    return {
        "schema_version": 1,
        "scope": "SIMULATION_ONLY",
        "comparison_mode": "OFFLINE_TWO_SEED_EVIDENCE_COMPARISON_ONLY",
        "compared_at_utc": utc_now(),
        "first_run": first.as_dict(),
        "second_run": second.as_dict(),
        "pre_pick_green_vision_position_delta_m": reported_green_delta,
        "minimum_required_green_position_delta_m": 0.05,
        "markers": {
            "P5_TWO_DISTINCT_RUN_DIRECTORIES": "PASS",
            "P5_TWO_DISTINCT_SEEDS": "PASS",
            "P5_TWO_DISTINCT_GZ_PARTITIONS": "PASS",
            "P5_TWO_SEED_RUN_PROVENANCE": "PASS",
            "P5_TWO_SEED_EVALUATOR_PASS": "PASS",
            "P5_TWO_SEED_ARTIFACT_HASHES_DISTINCT": "PASS",
            "P5_TWO_SEED_VISION_VARIATION": "PASS",
            "P5_TWO_SEED_PICK_PLACE_EVIDENCE": "PASS",
        },
        "status": "PASS",
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--first-run", required=True, type=Path)
    parser.add_argument("--second-run", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser


def output_overlaps_run(output: Path, run: Path) -> bool:
    resolved_output = output.resolve()
    return resolved_output == run or run in resolved_output.parents


def main(argv: list[str] | None = None) -> int:
    arguments = build_parser().parse_args(argv)
    try:
        first_root = resolve_run_root(arguments.first_run, "first run directory")
        second_root = resolve_run_root(arguments.second_run, "second run directory")
        if output_overlaps_run(arguments.output, first_root) or output_overlaps_run(
            arguments.output, second_root
        ):
            raise ComparisonError("--output must be outside both immutable run directories")
        result = build_comparison(first_root, second_root)
        atomic_write_json(arguments.output, result)
    except (ComparisonError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print("P5_TWO_SEED_PICK_PLACE_EVIDENCE=PASS")
    print("P5_TWO_SEED_VISION_VARIATION=PASS")
    print(f"P5_SEED_COMPARISON={arguments.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
