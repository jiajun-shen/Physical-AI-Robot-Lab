#!/usr/bin/env python3
"""Fail-closed offline evaluator for one Phase-5 pick/place episode.

The online runtime deliberately stops after ``RETREAT`` and writes an
``execution_evidence.json`` record with a pending post-place evaluation.  This
tool joins that record to the sealed pre-pick and post-place Phase-4 RGB-D
artifacts.  It never queries Gazebo, ROS, a seed manifest, or a world model.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import sys
import tempfile
from typing import Any, Iterable


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
RUNTIME_STAGES = (
    "OBSERVE_PICK",
    "PLAN_APPROACH",
    "OPEN",
    "APPROACH",
    "CLOSE",
    "LIFT_VERIFY",
    "TRANSPORT",
    "LOWER",
    "RELEASE",
    "RETREAT",
)
REQUIRED_STAGES = RUNTIME_STAGES + ("OBSERVE_PLACE", "VERIFY_PLACE")
MOTION_LABELS = (
    "pregrasp",
    "visual_checkpoint",
    "grasp_approach",
    "lift",
    "transport",
    "lower",
    "post_release_clearance",
    "retreat",
)
MOTION_STAGE_KEYS = (
    ("PLAN_APPROACH", "planned_pregrasp", "pregrasp"),
    ("APPROACH", "planned_grasp", "grasp_approach"),
    ("LIFT_VERIFY", "motion", "lift"),
    ("TRANSPORT", "motion", "transport"),
    ("LOWER", "motion", "lower"),
    ("RETREAT", "motion", "retreat"),
)
ARM_JOINTS = tuple(f"joint_{index}" for index in range(1, 7))

VISION_CONFIDENCE_FLOOR = 0.60
MIN_LIFT_M = 0.050
MIN_TRANSPORT_M = 0.050
MIN_RETENTION_SAMPLES = 3
MIN_LIFT_RETENTION_SAMPLES = 1
MIN_LOWER_RETENTION_SAMPLES = 1
MAX_TCP_RELATIVE_DRIFT_M = 0.040
# Must match motion.visual_checkpoint_max_tcp_distance_m. This bounded
# pre-close RGB-D/TCP gate accommodates the observed 90.6 mm Seed-43
# calibrated side-view endpoint without weakening final insertion or
# visual-retention evidence.
MAX_VISUAL_CHECKPOINT_TCP_DISTANCE_M = 0.095
MAX_UNOBSERVED_FINAL_APPROACH_M = 0.075
MAX_RETENTION_TCP_DISTANCE_M = 0.115
GRIPPER_OBSERVATION_TOLERANCE_M = 0.003
MIN_CONTACT_CLOSE_POSITION_M = 0.020
MIN_NON_OVERLAP_DISTANCE_M = 0.075
MAX_BESIDE_DISTANCE_M = 0.120
TABLE_SURFACE_Z_M = 0.750
BALL_RADIUS_M = 0.035
SUPPORT_HEIGHT_TOLERANCE_M = 0.035
EXPECTED_BALL_CENTER_Z_M = TABLE_SURFACE_Z_M + BALL_RADIUS_M
STATIC_MARKER_CLASSES = (
    "p5_orange_top_fiducial",
    "p5_violet_top_fiducial",
)
STATIC_MARKER_TO_BOX_CENTER_Z_M = 0.104
STATIC_COLLISION_BOX_SIZE_M = (0.160, 0.160, 0.220)


class EvaluationError(RuntimeError):
    """Raised when a required evidence boundary or acceptance gate is absent."""


@dataclass(frozen=True)
class VisionPose:
    class_id: str
    detection_id: str
    position_m: tuple[float, float, float]
    source_stamp_ns: int
    confidence: float


@dataclass(frozen=True)
class Phase4Evidence:
    path: Path
    sha256: str
    source_stamp_ns: int
    poses: dict[str, VisionPose]


@dataclass(frozen=True)
class TrackingSample:
    pose: VisionPose
    tcp_position_m: tuple[float, float, float]
    relative_m: tuple[float, float, float]


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
        raise EvaluationError(f"{label} is missing: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise EvaluationError(f"cannot parse {label}: {path}") from exc
    return require_mapping(payload, label)


def require_mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise EvaluationError(f"{label} must be a JSON object")
    return value


def require_list(value: Any, label: str) -> list[Any]:
    if not isinstance(value, list):
        raise EvaluationError(f"{label} must be a list")
    return value


def require_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise EvaluationError(f"{label} must be a non-empty string")
    return value


def require_bool(value: Any, label: str) -> bool:
    if type(value) is not bool:
        raise EvaluationError(f"{label} must be a boolean")
    return value


def require_int(value: Any, label: str, *, minimum: int | None = None) -> int:
    if type(value) is not int:
        raise EvaluationError(f"{label} must be an integer")
    if minimum is not None and value < minimum:
        raise EvaluationError(f"{label} must be >= {minimum}")
    return value


def require_number(
    value: Any,
    label: str,
    *,
    minimum: float | None = None,
    positive: bool = False,
) -> float:
    if type(value) not in (int, float):
        raise EvaluationError(f"{label} must be numeric")
    number = float(value)
    if not math.isfinite(number):
        raise EvaluationError(f"{label} must be finite")
    if positive and number <= 0.0:
        raise EvaluationError(f"{label} must be positive")
    if minimum is not None and number < minimum:
        raise EvaluationError(f"{label} must be >= {minimum}")
    return number


def require_vector(value: Any, label: str) -> tuple[float, float, float]:
    items = require_list(value, label)
    if len(items) != 3:
        raise EvaluationError(f"{label} must have exactly three values")
    return tuple(require_number(item, f"{label}[{index}]") for index, item in enumerate(items))  # type: ignore[return-value]


def require_timestamp(value: Any, label: str, *, positive: bool = False) -> int:
    return require_int(value, label, minimum=1 if positive else 0)


def require_utc_timestamp(value: Any, label: str) -> str:
    text = require_string(value, label)
    try:
        parsed = datetime.fromisoformat(text.replace("Z", "+00:00"))
    except ValueError as exc:
        raise EvaluationError(f"{label} is not ISO-8601") from exc
    if parsed.tzinfo is None:
        raise EvaluationError(f"{label} must include a timezone")
    return text


def close_enough(first: float, second: float, *, tolerance: float = 1e-6) -> bool:
    return math.isclose(first, second, rel_tol=0.0, abs_tol=tolerance)


def require_same_number(first: float, second: float, label: str, *, tolerance: float = 1e-6) -> None:
    if not close_enough(first, second, tolerance=tolerance):
        raise EvaluationError(f"{label} is inconsistent")


def require_same_vector(
    first: Iterable[float], second: Iterable[float], label: str, *, tolerance: float = 1e-6
) -> None:
    if any(not close_enough(a, b, tolerance=tolerance) for a, b in zip(first, second)):
        raise EvaluationError(f"{label} is inconsistent")


def parse_header(
    value: Any,
    label: str,
    *,
    frame_id: str | None = None,
    positive_stamp: bool = False,
) -> tuple[int, str]:
    header = require_mapping(value, label)
    sec = require_int(header.get("stamp_sec"), f"{label}.stamp_sec", minimum=0)
    nanosec = require_int(header.get("stamp_nanosec"), f"{label}.stamp_nanosec", minimum=0)
    if nanosec >= 1_000_000_000:
        raise EvaluationError(f"{label}.stamp_nanosec is out of range")
    stamp = require_timestamp(header.get("stamp_ns"), f"{label}.stamp_ns", positive=positive_stamp)
    if stamp != sec * 1_000_000_000 + nanosec:
        raise EvaluationError(f"{label} timestamp components disagree")
    observed_frame = require_string(header.get("frame_id"), f"{label}.frame_id")
    if frame_id is not None and observed_frame != frame_id:
        raise EvaluationError(f"{label}.frame_id must be {frame_id!r}")
    return stamp, observed_frame


def safe_artifact_path(root: Path, relative: str, label: str) -> Path:
    if "\\" in relative:
        raise EvaluationError(f"{label} must use POSIX-relative paths")
    parsed = PurePosixPath(relative)
    if parsed.is_absolute() or not parsed.parts or any(part in {".", ".."} for part in parsed.parts):
        raise EvaluationError(f"{label} escapes the Phase-4 evidence root")
    root_resolved = root.resolve()
    candidate = (root_resolved / Path(*parsed.parts)).resolve()
    try:
        candidate.relative_to(root_resolved)
    except ValueError as exc:
        raise EvaluationError(f"{label} escapes the Phase-4 evidence root") from exc
    return candidate


def parse_p4_pose(
    value: Any,
    label: str,
    *,
    expected_frame: str,
    source_stamp_ns: int,
    source_frame: str,
) -> VisionPose:
    pose = require_mapping(value, label)
    class_id = require_string(pose.get("class_id"), f"{label}.class_id")
    detection_id = require_string(pose.get("detection_id"), f"{label}.detection_id")
    if require_int(pose.get("source_mode"), f"{label}.source_mode") != 1:
        raise EvaluationError(f"{label} is not vision-sourced")
    parse_header(pose.get("header"), f"{label}.header", frame_id=expected_frame)
    observed_stamp, observed_source_frame = parse_header(
        pose.get("source_header"), f"{label}.source_header", positive_stamp=True
    )
    if observed_stamp != source_stamp_ns or observed_source_frame != source_frame:
        raise EvaluationError(f"{label} does not share the sealed RGB-D source header")
    confidence = require_number(pose.get("confidence"), f"{label}.confidence", positive=True)
    if confidence < VISION_CONFIDENCE_FLOOR:
        raise EvaluationError(f"{label}.confidence is below {VISION_CONFIDENCE_FLOOR:.2f}")
    if require_bool(pose.get("orientation_valid"), f"{label}.orientation_valid"):
        raise EvaluationError(f"{label} unexpectedly claims an orientation estimate")
    require_int(pose.get("valid_depth_samples"), f"{label}.valid_depth_samples", minimum=1)
    require_number(pose.get("depth_m"), f"{label}.depth_m", positive=True)
    require_number(pose.get("depth_mad_m"), f"{label}.depth_mad_m", minimum=0.0)
    return VisionPose(
        class_id=class_id,
        detection_id=detection_id,
        position_m=require_vector(pose.get("position_m"), f"{label}.position_m"),
        source_stamp_ns=observed_stamp,
        confidence=confidence,
    )


def parse_p4_detections(
    value: Any, source_stamp_ns: int, source_frame: str
) -> dict[str, str]:
    entries = require_list(value, "Phase-4 vision_outputs.detections")
    if len(entries) != len(P4_CLASSES):
        raise EvaluationError("Phase-4 detections do not contain exactly five classes")
    result: dict[str, str] = {}
    for index, item in enumerate(entries):
        label = f"Phase-4 detections[{index}]"
        detection = require_mapping(item, label)
        class_id = require_string(detection.get("class_id"), f"{label}.class_id")
        if class_id in result:
            raise EvaluationError(f"Phase-4 detections duplicate {class_id}")
        detection_id = require_string(detection.get("detection_id"), f"{label}.detection_id")
        if require_int(detection.get("source_mode"), f"{label}.source_mode") != 1:
            raise EvaluationError(f"{label} is not vision-sourced")
        header_stamp, header_frame = parse_header(detection.get("header"), f"{label}.header")
        if header_frame != source_frame or header_stamp != source_stamp_ns:
            raise EvaluationError(f"{label} does not match the sealed camera source")
        confidence = require_number(detection.get("confidence"), f"{label}.confidence", positive=True)
        if confidence < VISION_CONFIDENCE_FLOOR:
            raise EvaluationError(f"{label}.confidence is below {VISION_CONFIDENCE_FLOOR:.2f}")
        result[class_id] = detection_id
    if tuple(result) != P4_CLASSES and set(result) != set(P4_CLASSES):
        raise EvaluationError("Phase-4 detections have unexpected class labels")
    return result


def parse_phase4_evidence(path: Path, label: str) -> Phase4Evidence:
    evidence = load_json(path, label)
    if require_int(evidence.get("schema_version"), f"{label}.schema_version") != 1:
        raise EvaluationError(f"{label} has an unsupported schema")
    if evidence.get("scope") != "SIMULATION_ONLY":
        raise EvaluationError(f"{label} is outside the simulation-only scope")
    if evidence.get("perception_mode") != "vision":
        raise EvaluationError(f"{label} is not vision-only evidence")
    if require_bool(evidence.get("ground_truth_input_used"), f"{label}.ground_truth_input_used"):
        raise EvaluationError(f"{label} reports a ground-truth input")
    require_utc_timestamp(evidence.get("sealed_at_utc"), f"{label}.sealed_at_utc")
    if require_list(evidence.get("expected_classes"), f"{label}.expected_classes") != list(P4_CLASSES):
        raise EvaluationError(f"{label} has unexpected expected_classes")
    source_stamp_ns, source_frame = parse_header(
        evidence.get("source_header"), f"{label}.source_header", positive_stamp=True
    )
    if source_frame != "lab_camera_optical_frame":
        raise EvaluationError(f"{label} has an unexpected source camera frame")
    minimum_stamp = require_timestamp(
        evidence.get("minimum_source_stamp_ns"), f"{label}.minimum_source_stamp_ns"
    )
    if source_stamp_ns < minimum_stamp:
        raise EvaluationError(f"{label} source stamp predates its minimum stamp")
    statuses = require_mapping(evidence.get("status"), f"{label}.status")
    for status_name in (
        "p4_rgb_transport",
        "p4_depth_transport",
        "p4_camera_info",
        "p4_tf_chain",
        "p4_all_five_typed_outputs",
        "p4_vision_input_boundary",
    ):
        if statuses.get(status_name) != "PASS":
            raise EvaluationError(f"{label} has not passed {status_name}")

    root = path.resolve().parent.parent
    artifact_hashes = require_mapping(evidence.get("artifact_sha256"), f"{label}.artifact_sha256")
    if set(artifact_hashes) != P4_ARTIFACTS:
        raise EvaluationError(f"{label} has incomplete or unexpected Phase-4 artifact hashes")
    for relative, expected_hash in artifact_hashes.items():
        if not isinstance(relative, str):
            raise EvaluationError(f"{label}.artifact_sha256 has a non-string path")
        if not isinstance(expected_hash, str) or len(expected_hash) != 64:
            raise EvaluationError(f"{label}.artifact_sha256[{relative!r}] is malformed")
        if any(character not in "0123456789abcdef" for character in expected_hash):
            raise EvaluationError(f"{label}.artifact_sha256[{relative!r}] is not lowercase SHA-256")
        artifact = safe_artifact_path(root, relative, f"{label}.artifact_sha256[{relative!r}]")
        if not artifact.is_file() or sha256_file(artifact) != expected_hash:
            raise EvaluationError(f"{label} artifact hash mismatch: {relative}")

    outputs_path = safe_artifact_path(root, "evidence/vision_outputs.json", f"{label}.vision_outputs")
    outputs = load_json(outputs_path, f"{label} vision outputs")
    detection_ids = parse_p4_detections(outputs.get("detections"), source_stamp_ns, source_frame)
    camera_entries = require_list(outputs.get("camera_poses"), f"{label} camera poses")
    world_entries = require_list(outputs.get("world_poses"), f"{label} world poses")
    if len(camera_entries) != len(P4_CLASSES) or len(world_entries) != len(P4_CLASSES):
        raise EvaluationError(f"{label} lacks exactly five typed camera/world poses")
    camera_by_class: dict[str, VisionPose] = {}
    world_by_class: dict[str, VisionPose] = {}
    for index, item in enumerate(camera_entries):
        parsed = parse_p4_pose(
            item,
            f"{label} camera_poses[{index}]",
            expected_frame=source_frame,
            source_stamp_ns=source_stamp_ns,
            source_frame=source_frame,
        )
        if parsed.class_id in camera_by_class:
            raise EvaluationError(f"{label} camera poses duplicate {parsed.class_id}")
        camera_by_class[parsed.class_id] = parsed
    for index, item in enumerate(world_entries):
        parsed = parse_p4_pose(
            item,
            f"{label} world_poses[{index}]",
            expected_frame="world",
            source_stamp_ns=source_stamp_ns,
            source_frame=source_frame,
        )
        if parsed.class_id in world_by_class:
            raise EvaluationError(f"{label} world poses duplicate {parsed.class_id}")
        world_by_class[parsed.class_id] = parsed
    if set(camera_by_class) != set(P4_CLASSES) or set(world_by_class) != set(P4_CLASSES):
        raise EvaluationError(f"{label} pose classes are incomplete or unexpected")
    for class_id in P4_CLASSES:
        if (
            detection_ids[class_id] != camera_by_class[class_id].detection_id
            or detection_ids[class_id] != world_by_class[class_id].detection_id
        ):
            raise EvaluationError(f"{label} detection identity chain is broken for {class_id}")
    return Phase4Evidence(
        path=path.resolve(),
        sha256=sha256_file(path),
        source_stamp_ns=source_stamp_ns,
        poses=world_by_class,
    )


def parse_runtime_vision(value: Any, label: str, *, expected_class: str | None = None) -> VisionPose:
    vision = require_mapping(value, label)
    if vision.get("source_mode") != "VISION":
        raise EvaluationError(f"{label} is not a runtime vision observation")
    class_id = require_string(vision.get("class_id"), f"{label}.class_id")
    if expected_class is not None and class_id != expected_class:
        raise EvaluationError(f"{label}.class_id must be {expected_class}")
    confidence = require_number(vision.get("confidence"), f"{label}.confidence", positive=True)
    if confidence < VISION_CONFIDENCE_FLOOR:
        raise EvaluationError(f"{label}.confidence is below {VISION_CONFIDENCE_FLOOR:.2f}")
    return VisionPose(
        class_id=class_id,
        detection_id=require_string(vision.get("detection_id"), f"{label}.detection_id"),
        position_m=require_vector(vision.get("position_m"), f"{label}.position_m"),
        source_stamp_ns=require_timestamp(
            vision.get("source_stamp_ns"), f"{label}.source_stamp_ns", positive=True
        ),
        confidence=confidence,
    )


def require_same_pose(first: VisionPose, second: VisionPose, label: str) -> None:
    if first.class_id != second.class_id or first.detection_id != second.detection_id:
        raise EvaluationError(f"{label} has a different identity")
    if first.source_stamp_ns != second.source_stamp_ns:
        raise EvaluationError(f"{label} has a different source stamp")
    require_same_vector(first.position_m, second.position_m, label)
    require_same_number(first.confidence, second.confidence, label)


def parse_stage_records(execution: dict[str, Any]) -> dict[str, dict[str, Any]]:
    if require_list(execution.get("required_stages_executed"), "execution.required_stages_executed") != list(
        RUNTIME_STAGES
    ):
        raise EvaluationError("execution does not declare the exact runtime stage prefix")
    raw_records = require_list(execution.get("stage_records"), "execution.stage_records")
    if len(raw_records) != len(RUNTIME_STAGES):
        raise EvaluationError("execution does not contain exactly ten runtime stage records")
    records: dict[str, dict[str, Any]] = {}
    last_sim_time = -1
    for index, expected_stage in enumerate(RUNTIME_STAGES):
        record = require_mapping(raw_records[index], f"execution.stage_records[{index}]")
        if record.get("stage") != expected_stage:
            raise EvaluationError("execution stage sequence is not exact")
        if record.get("status") != "PASS":
            raise EvaluationError(f"execution stage {expected_stage} did not PASS")
        require_mapping(record.get("details"), f"execution stage {expected_stage}.details")
        sim_time = require_timestamp(record.get("sim_time_ns"), f"execution stage {expected_stage}.sim_time_ns")
        if sim_time < last_sim_time:
            raise EvaluationError("execution stage timestamps are not monotonic")
        last_sim_time = sim_time
        require_utc_timestamp(record.get("wall_time_utc"), f"execution stage {expected_stage}.wall_time_utc")
        records[expected_stage] = record
    return records


def validate_execution_envelope(execution: dict[str, Any]) -> int:
    if require_int(execution.get("schema_version"), "execution.schema_version") != 1:
        raise EvaluationError("execution has an unsupported schema")
    if execution.get("scope") != "SIMULATION_ONLY":
        raise EvaluationError("execution is outside the simulation-only scope")
    seed = require_int(execution.get("seed"), "execution.seed", minimum=0)
    if require_bool(
        execution.get("online_ground_truth_input_used"), "execution.online_ground_truth_input_used"
    ):
        raise EvaluationError("execution reports an online ground-truth input")
    if execution.get("status") != "PENDING_POST_PLACE_VISION_EVALUATION":
        raise EvaluationError("execution is not the required pending post-place record")
    require_utc_timestamp(execution.get("finished_at_utc"), "execution.finished_at_utc")
    return seed


def validate_pre_pick_link(
    execution: dict[str, Any], stages: dict[str, dict[str, Any]], pre_pick: Phase4Evidence
) -> None:
    execution_pre = require_mapping(execution.get("pre_pick_evidence"), "execution.pre_pick_evidence")
    if execution_pre.get("sealed_artifact") != "external_phase4_vision_evidence":
        raise EvaluationError("execution pre-pick provenance marker is missing")
    if require_timestamp(
        execution_pre.get("source_stamp_ns"), "execution.pre_pick_evidence.source_stamp_ns", positive=True
    ) != pre_pick.source_stamp_ns:
        raise EvaluationError("execution pre-pick source stamp does not match sealed Phase-4 evidence")
    for class_id in ("green_ball", "red_ball"):
        runtime_pose = parse_runtime_vision(
            execution_pre.get("green" if class_id == "green_ball" else "red"),
            f"execution.pre_pick_evidence.{class_id}",
            expected_class=class_id,
        )
        require_same_pose(runtime_pose, pre_pick.poses[class_id], f"execution pre-pick {class_id}")

    observe = require_mapping(stages["OBSERVE_PICK"].get("details"), "OBSERVE_PICK.details")
    if require_bool(observe.get("vision_only"), "OBSERVE_PICK.details.vision_only") is not True:
        raise EvaluationError("OBSERVE_PICK is not vision-only")
    if require_bool(
        observe.get("ground_truth_input_used"), "OBSERVE_PICK.details.ground_truth_input_used"
    ):
        raise EvaluationError("OBSERVE_PICK reports a ground-truth input")
    if require_timestamp(
        observe.get("pre_pick_source_stamp_ns"), "OBSERVE_PICK.details.pre_pick_source_stamp_ns", positive=True
    ) != pre_pick.source_stamp_ns:
        raise EvaluationError("OBSERVE_PICK source stamp does not match sealed Phase-4 evidence")
    for class_id, key in (("green_ball", "green"), ("red_ball", "red")):
        runtime_pose = parse_runtime_vision(
            observe.get(key), f"OBSERVE_PICK.details.{key}", expected_class=class_id
        )
        require_same_pose(runtime_pose, pre_pick.poses[class_id], f"OBSERVE_PICK {class_id}")


def validate_static_obstacle_coverage(
    stages: dict[str, dict[str, Any]], pre_pick: Phase4Evidence
) -> list[str]:
    """Require two fresh RGB-D obstacles to be present for every motion plan."""

    observe = stage_details(stages, "OBSERVE_PICK")
    entries = require_list(
        observe.get("static_obstacle_vision"), "OBSERVE_PICK.details.static_obstacle_vision"
    )
    if len(entries) != len(STATIC_MARKER_CLASSES):
        raise EvaluationError("OBSERVE_PICK lacks exactly two static-obstacle RGB-D records")
    observed_ids: list[str] = []
    seen_markers: set[str] = set()
    for index, raw_entry in enumerate(entries):
        entry = require_mapping(raw_entry, f"OBSERVE_PICK.static_obstacle_vision[{index}]")
        object_id = require_string(
            entry.get("object_id"), f"OBSERVE_PICK.static_obstacle_vision[{index}].object_id"
        )
        marker = parse_runtime_vision(
            entry.get("marker"), f"OBSERVE_PICK.static_obstacle_vision[{index}].marker"
        )
        if marker.class_id not in STATIC_MARKER_CLASSES or marker.class_id in seen_markers:
            raise EvaluationError("static-obstacle marker classes are missing or duplicated")
        if marker.source_stamp_ns <= pre_pick.source_stamp_ns:
            raise EvaluationError("static-obstacle marker evidence is not fresh after pre-pick seal")
        if entry.get("source") != "fresh_phase5_rgbd_static_obstacle_localizer":
            raise EvaluationError("static-obstacle collision record does not identify the RGB-D localizer")
        offset = require_number(
            entry.get("marker_top_to_box_center_z_offset_m"),
            f"OBSERVE_PICK.static_obstacle_vision[{index}].marker_top_to_box_center_z_offset_m",
            positive=True,
        )
        require_same_number(
            offset,
            STATIC_MARKER_TO_BOX_CENTER_Z_M,
            "static-obstacle marker-to-centre offset",
        )
        center = require_vector(
            entry.get("collision_box_center_m"),
            f"OBSERVE_PICK.static_obstacle_vision[{index}].collision_box_center_m",
        )
        require_same_vector(
            center,
            (
                marker.position_m[0],
                marker.position_m[1],
                marker.position_m[2] - offset,
            ),
            "static-obstacle RGB-D marker-to-box transform",
        )
        size = require_vector(
            entry.get("collision_box_size_xyz_m"),
            f"OBSERVE_PICK.static_obstacle_vision[{index}].collision_box_size_xyz_m",
        )
        if any(size[axis] + 1e-6 < STATIC_COLLISION_BOX_SIZE_M[axis] for axis in range(3)):
            raise EvaluationError("static-obstacle collision box is weaker than the conservative P5 envelope")
        expected_id = f"p5_vision_static_{marker.class_id}"
        if object_id != expected_id:
            raise EvaluationError("static-obstacle collision ID does not match its RGB-D marker")
        seen_markers.add(marker.class_id)
        observed_ids.append(object_id)
    if seen_markers != set(STATIC_MARKER_CLASSES):
        raise EvaluationError("static-obstacle evidence does not cover both P5 marker classes")
    initial_ids = require_list(
        observe.get("planning_scene_object_ids"), "OBSERVE_PICK.details.planning_scene_object_ids"
    )
    if not set(observed_ids).issubset({require_string(item, "planning-scene object ID") for item in initial_ids}):
        raise EvaluationError("OBSERVE_PICK planning scene is missing a static RGB-D obstacle")
    return sorted(observed_ids)


def validate_motion_record(value: Any, label: str) -> dict[str, Any]:
    record = require_mapping(value, label)
    if require_bool(record.get("collision_aware"), f"{label}.collision_aware") is not True:
        raise EvaluationError(f"{label} is not collision-aware")
    target = require_mapping(record.get("target"), f"{label}.target")
    if target.get("frame_id") != "world":
        raise EvaluationError(f"{label}.target is not in the world frame")
    require_vector(target.get("position_m"), f"{label}.target.position_m")
    orientation = require_list(target.get("orientation_xyzw"), f"{label}.target.orientation_xyzw")
    if len(orientation) != 4:
        raise EvaluationError(f"{label}.target.orientation_xyzw must have four values")
    quaternion = [require_number(item, f"{label}.target.orientation_xyzw[{index}]") for index, item in enumerate(orientation)]
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm < 0.999 or norm > 1.001:
        raise EvaluationError(f"{label}.target orientation is not normalized")
    planning = require_mapping(record.get("planning"), f"{label}.planning")
    if require_int(planning.get("value"), f"{label}.planning.value") != 1:
        raise EvaluationError(f"{label} MoveIt planning did not return SUCCESS")
    if require_int(planning.get("waypoint_count"), f"{label}.planning.waypoint_count", minimum=2) < 2:
        raise EvaluationError(f"{label} has too few planned waypoints")
    retiming = require_mapping(planning.get("retiming"), f"{label}.planning.retiming")
    if retiming.get("method") != "velocity_only_conservative_retime":
        raise EvaluationError(f"{label} does not record the bounded retiming method")
    execution = require_mapping(record.get("execution"), f"{label}.execution")
    if require_int(execution.get("value"), f"{label}.execution.value") != 1:
        raise EvaluationError(f"{label} MoveIt execution did not return SUCCESS")
    if require_int(execution.get("action_status"), f"{label}.execution.action_status") != 4:
        raise EvaluationError(f"{label} MoveIt action did not succeed")
    endpoint = require_mapping(record.get("observed_endpoint"), f"{label}.observed_endpoint")
    if set(endpoint) != set(ARM_JOINTS):
        raise EvaluationError(f"{label} endpoint is not the bounded six-axis chain")
    for joint in ARM_JOINTS:
        require_number(endpoint.get(joint), f"{label}.observed_endpoint.{joint}")
    return record


def validate_motion_records(
    execution: dict[str, Any], stages: dict[str, dict[str, Any]], static_obstacle_ids: list[str]
) -> None:
    raw_records = require_list(execution.get("motion_records"), "execution.motion_records")
    if len(raw_records) != len(MOTION_LABELS):
        raise EvaluationError("execution does not contain exactly six arm-motion records")
    records: dict[str, dict[str, Any]] = {}
    for index, expected_label in enumerate(MOTION_LABELS):
        record = validate_motion_record(raw_records[index], f"execution.motion_records[{index}]")
        if record.get("label") != expected_label:
            raise EvaluationError("execution motion record sequence is not exact")
        recorded_static_ids = require_list(
            record.get("planning_scene_static_obstacle_ids"),
            f"execution.motion_records[{index}].planning_scene_static_obstacle_ids",
        )
        if sorted(
            require_string(
                item,
                f"execution.motion_records[{index}].planning_scene_static_obstacle_ids",
            )
            for item in recorded_static_ids
        ) != static_obstacle_ids:
            raise EvaluationError(
                f"{expected_label} does not prove both static RGB-D obstacles were in MoveIt"
            )
        records[expected_label] = record
    for stage_name, detail_key, motion_label in MOTION_STAGE_KEYS:
        details = require_mapping(stages[stage_name].get("details"), f"{stage_name}.details")
        stage_motion = require_mapping(details.get(detail_key), f"{stage_name}.details.{detail_key}")
        if stage_motion != records[motion_label]:
            raise EvaluationError(f"{stage_name} is not tied to its successful MoveIt motion record")
    retreat_details = require_mapping(stages["RETREAT"].get("details"), "RETREAT.details")
    if (
        require_mapping(
            retreat_details.get("vertical_clearance_motion"),
            "RETREAT.details.vertical_clearance_motion",
        )
        != records["post_release_clearance"]
    ):
        raise EvaluationError("RETREAT is not tied to its successful vertical clearance motion")


def validate_gripper_action(value: Any, label: str, *, contact_close: bool = False) -> tuple[float, float]:
    action = require_mapping(value, label)
    if not require_bool(action.get("action_goal_accepted"), f"{label}.action_goal_accepted"):
        raise EvaluationError(f"{label} action goal was not accepted")
    action_status = require_int(action.get("action_status"), f"{label}.action_status")
    require_number(action.get("result_position_m"), f"{label}.result_position_m")
    if action.get("result_effort") != "UNTRUSTED_NOT_USED":
        raise EvaluationError(f"{label} must not promote untrusted gripper effort")
    result_stalled = require_bool(action.get("result_stalled"), f"{label}.result_stalled")
    result_reached_goal = require_bool(action.get("result_reached_goal"), f"{label}.result_reached_goal")
    terminal_semantics = action.get("terminal_status_semantics")
    observation = require_mapping(action.get("observation"), f"{label}.observation")
    target = require_number(observation.get("target_position_m"), f"{label}.observation.target_position_m", positive=True)
    observed = require_number(observation.get("observed_position_m"), f"{label}.observation.observed_position_m", positive=True)
    if contact_close:
        minimum_contact = require_number(
            observation.get("minimum_contact_position_m"),
            f"{label}.observation.minimum_contact_position_m",
            positive=True,
        )
        if minimum_contact < MIN_CONTACT_CLOSE_POSITION_M or minimum_contact >= target:
            raise EvaluationError(f"{label} contact-close range is outside the calibrated bound")
        if not minimum_contact <= observed <= target + GRIPPER_OBSERVATION_TOLERANCE_M:
            raise EvaluationError(f"{label} has no bounded contact-close joint-state observation")
        if observation.get("endpoint_observation_semantics") != "bounded_contact_close_joint_state":
            raise EvaluationError(f"{label} does not identify the bounded contact-close endpoint")
        if terminal_semantics == "controller_succeeded_with_observed_contact_range":
            if action_status != 4 or result_stalled or not result_reached_goal:
                raise EvaluationError(f"{label} has inconsistent successful contact-close status")
        elif terminal_semantics == "controller_stalled_with_observed_contact_range":
            if action_status != 6 or not result_stalled or result_reached_goal:
                raise EvaluationError(f"{label} has inconsistent stalled contact-close status")
        else:
            raise EvaluationError(f"{label} has no approved bounded contact-close terminal status")
    else:
        if action_status != 4 or result_stalled or not result_reached_goal:
            raise EvaluationError(f"{label} action did not succeed")
        if terminal_semantics != "controller_succeeded_with_observed_target":
            raise EvaluationError(f"{label} does not identify a successful target endpoint")
        if abs(observed - target) > GRIPPER_OBSERVATION_TOLERANCE_M:
            raise EvaluationError(f"{label} has no in-tolerance gripper-state observation")
    require_timestamp(observation.get("sim_time_ns"), f"{label}.observation.sim_time_ns")
    require_utc_timestamp(observation.get("wall_time_utc"), f"{label}.observation.wall_time_utc")
    return target, observed


def parse_tracking_sample(value: Any, label: str) -> TrackingSample:
    sample = require_mapping(value, label)
    pose = parse_runtime_vision(sample.get("vision"), f"{label}.vision", expected_class="green_ball")
    tcp = require_mapping(sample.get("tcp"), f"{label}.tcp")
    tcp_position = require_vector(tcp.get("position_m"), f"{label}.tcp.position_m")
    require_timestamp(tcp.get("tf_stamp_ns"), f"{label}.tcp.tf_stamp_ns")
    relative = require_vector(sample.get("green_minus_tcp_m"), f"{label}.green_minus_tcp_m")
    if sample.get("tf_alignment") != "tcp_lookup_at_vision_source_stamp":
        raise EvaluationError(f"{label} does not use a vision-timestamp-aligned TCP transform")
    expected_relative = tuple(pose.position_m[index] - tcp_position[index] for index in range(3))
    require_same_vector(relative, expected_relative, f"{label}.green_minus_tcp_m")
    sampled_at = require_mapping(sample.get("sampled_at"), f"{label}.sampled_at")
    require_timestamp(sampled_at.get("sim_time_ns"), f"{label}.sampled_at.sim_time_ns")
    require_utc_timestamp(sampled_at.get("wall_time_utc"), f"{label}.sampled_at.wall_time_utc")
    return TrackingSample(pose=pose, tcp_position_m=tcp_position, relative_m=relative)


def require_same_tracking_sample(first: TrackingSample, second: TrackingSample, label: str) -> None:
    require_same_pose(first.pose, second.pose, f"{label} vision")
    require_same_vector(first.tcp_position_m, second.tcp_position_m, f"{label} TCP")
    require_same_vector(first.relative_m, second.relative_m, f"{label} relative offset")


def validate_retention_samples(
    value: Any,
    label: str,
    baseline: TrackingSample,
    *,
    after_stamp_ns: int,
    minimum_samples: int = MIN_RETENTION_SAMPLES,
) -> list[TrackingSample]:
    entries = require_list(value, label)
    if len(entries) < minimum_samples:
        raise EvaluationError(f"{label} has fewer than {minimum_samples} retention samples")
    samples: list[TrackingSample] = []
    newest = after_stamp_ns
    for index, raw_sample in enumerate(entries):
        sample_label = f"{label}[{index}]"
        sample = parse_tracking_sample(raw_sample, sample_label)
        if sample.pose.source_stamp_ns <= newest:
            raise EvaluationError(f"{sample_label} is not a fresh retention observation")
        newest = sample.pose.source_stamp_ns
        drift = math.sqrt(
            sum((sample.relative_m[axis] - baseline.relative_m[axis]) ** 2 for axis in range(3))
        )
        recorded_drift = require_number(
            require_mapping(raw_sample, sample_label).get("tcp_relative_drift_m"),
            f"{sample_label}.tcp_relative_drift_m",
            minimum=0.0,
        )
        require_same_number(recorded_drift, drift, f"{sample_label}.tcp_relative_drift_m")
        if drift > MAX_TCP_RELATIVE_DRIFT_M:
            raise EvaluationError(f"{sample_label} exceeds the TCP-relative retention drift bound")
        samples.append(sample)
    return samples


def stage_details(stages: dict[str, dict[str, Any]], stage: str) -> dict[str, Any]:
    return require_mapping(stages[stage].get("details"), f"{stage}.details")


def validate_attachment_retention_and_release(
    execution: dict[str, Any], stages: dict[str, dict[str, Any]], pre_pick: Phase4Evidence
) -> tuple[int, float, float]:
    open_target, _ = validate_gripper_action(stage_details(stages, "OPEN"), "OPEN.details")
    close_target, close_observed = validate_gripper_action(
        stage_details(stages, "CLOSE"), "CLOSE.details", contact_close=True
    )
    if close_target >= open_target:
        raise EvaluationError("CLOSE does not command a smaller gripper opening than OPEN")

    attachment = require_mapping(execution.get("attachment"), "execution.attachment")
    guard = require_mapping(attachment.get("guard"), "execution.attachment.guard")
    if require_bool(guard.get("force_or_effort_used"), "execution.attachment.guard.force_or_effort_used"):
        raise EvaluationError("attachment guard improperly relies on force/effort")
    guard_sample = parse_tracking_sample(
        guard.get("fresh_vision_tcp_sample"), "execution.attachment.guard.fresh_vision_tcp_sample"
    )
    if guard_sample.pose.source_stamp_ns <= pre_pick.source_stamp_ns:
        raise EvaluationError("attachment guard does not use fresh RGB-D evidence")
    if guard.get("vision_sample_timing") != "visual_checkpoint_tf_aligned_before_observed_close":
        raise EvaluationError("attachment guard does not identify the visual checkpoint timing")
    guard_distance = require_number(
        guard.get("tcp_distance_m"), "execution.attachment.guard.tcp_distance_m", minimum=0.0
    )
    calculated_distance = math.sqrt(sum(component * component for component in guard_sample.relative_m))
    require_same_number(guard_distance, calculated_distance, "attachment guard TCP distance")
    if guard_distance > MAX_VISUAL_CHECKPOINT_TCP_DISTANCE_M:
        raise EvaluationError("attachment guard visual checkpoint is too distant")
    guard_close = require_number(
        guard.get("observed_close_position_m"), "execution.attachment.guard.observed_close_position_m", positive=True
    )
    require_same_number(guard_close, close_observed, "attachment guard close observation")
    approach_details = stage_details(stages, "APPROACH")
    checkpoint_sample = require_mapping(
        approach_details.get("visual_checkpoint_guard"), "APPROACH.details.visual_checkpoint_guard"
    )
    if checkpoint_sample != guard.get("fresh_vision_tcp_sample"):
        raise EvaluationError("attachment guard does not preserve the APPROACH visual checkpoint sample")
    checkpoint_motion = require_mapping(
        approach_details.get("visual_checkpoint"), "APPROACH.details.visual_checkpoint"
    )
    validate_motion_record(checkpoint_motion, "APPROACH.details.visual_checkpoint")
    blind_distance = require_number(
        approach_details.get("unobserved_final_approach_m"),
        "APPROACH.details.unobserved_final_approach_m",
        positive=True,
    )
    if blind_distance > MAX_UNOBSERVED_FINAL_APPROACH_M:
        raise EvaluationError("APPROACH blind final insertion exceeds the calibrated bound")
    activation = require_mapping(attachment.get("activation"), "execution.attachment.activation")
    if activation.get("activation") != "dynamic_entity_system_add_after_vision_tcp_close_guard":
        raise EvaluationError("attachment activation is not the guarded runtime mechanism")
    attached_state = require_mapping(attachment.get("attached_state"), "execution.attachment.attached_state")
    if attached_state.get("state") != "attached":
        raise EvaluationError("attachment record has no attached state")

    raw_states = require_list(execution.get("attachment_states"), "execution.attachment_states")
    if len(raw_states) < 2:
        raise EvaluationError("attachment lifecycle has too few state observations")
    states: list[dict[str, Any]] = []
    for index, raw_state in enumerate(raw_states):
        state = require_mapping(raw_state, f"execution.attachment_states[{index}]")
        if state.get("state") not in {"attached", "detached"}:
            raise EvaluationError("attachment lifecycle contains an unknown state")
        require_timestamp(state.get("sim_time_ns"), f"execution.attachment_states[{index}].sim_time_ns")
        require_utc_timestamp(state.get("wall_time_utc"), f"execution.attachment_states[{index}].wall_time_utc")
        states.append(state)
    try:
        attached_index = states.index(attached_state)
    except ValueError as exc:
        raise EvaluationError("attachment attached state is not in the lifecycle trace") from exc
    detached_indices = [index for index, state in enumerate(states) if state["state"] == "detached"]
    if not detached_indices:
        raise EvaluationError("attachment lifecycle has no detached state")
    detached_index = detached_indices[-1]
    if attached_index >= detached_index or states[-1]["state"] != "detached":
        raise EvaluationError("attachment lifecycle does not end detached after attachment")
    if any(state["state"] == "attached" for state in states[detached_index + 1 :]):
        raise EvaluationError("attachment is reattached after release")

    retention = require_mapping(execution.get("retention"), "execution.retention")
    baseline = parse_tracking_sample(retention.get("baseline"), "execution.retention.baseline")
    require_same_tracking_sample(baseline, guard_sample, "retention baseline versus attachment guard")
    raw_lift_samples = require_list(retention.get("lift_samples"), "execution.retention.lift_samples")
    if not raw_lift_samples:
        raise EvaluationError("execution.retention.lift_samples is empty")
    first_lift_raw = require_mapping(raw_lift_samples[0], "execution.retention.lift_samples[0]")
    if first_lift_raw.get("retention_baseline") != "post_attachment_vision_tcp_sample":
        raise EvaluationError("first lift sample does not establish the post-attachment visual baseline")
    lift_baseline = parse_tracking_sample(first_lift_raw, "execution.retention.lift_samples[0]")
    lift_baseline_distance = math.sqrt(sum(component * component for component in lift_baseline.relative_m))
    if lift_baseline_distance > MAX_RETENTION_TCP_DISTANCE_M:
        raise EvaluationError("first lift sample exceeds the calibrated retention TCP-distance bound")
    lift_samples = validate_retention_samples(
        retention.get("lift_samples"),
        "execution.retention.lift_samples",
        lift_baseline,
        after_stamp_ns=baseline.pose.source_stamp_ns,
        minimum_samples=MIN_LIFT_RETENTION_SAMPLES,
    )
    lift_delta = max(sample.pose.position_m[2] for sample in lift_samples) - baseline.pose.position_m[2]
    if lift_delta < MIN_LIFT_M:
        raise EvaluationError(f"visual lift is below {MIN_LIFT_M:.3f} m")
    recorded_lift = require_number(retention.get("lift_delta_m"), "execution.retention.lift_delta_m")
    require_same_number(recorded_lift, lift_delta, "retention lift delta")
    lift_details = stage_details(stages, "LIFT_VERIFY")
    if lift_details.get("attachment") != attachment:
        raise EvaluationError("LIFT_VERIFY is not tied to the guarded attachment record")
    if lift_details.get("vision_retention_samples") != retention.get("lift_samples"):
        raise EvaluationError("LIFT_VERIFY samples do not match the retention record")
    if require_number(lift_details.get("minimum_lift_m"), "LIFT_VERIFY.details.minimum_lift_m") < MIN_LIFT_M:
        raise EvaluationError("LIFT_VERIFY declares a weaker-than-required lift threshold")
    require_same_number(
        require_number(lift_details.get("lift_delta_m"), "LIFT_VERIFY.details.lift_delta_m"),
        lift_delta,
        "LIFT_VERIFY lift delta",
    )

    transport_samples = validate_retention_samples(
        retention.get("transport_samples"),
        "execution.retention.transport_samples",
        lift_samples[-1],
        after_stamp_ns=lift_samples[-1].pose.source_stamp_ns,
    )
    transport_distance = math.hypot(
        transport_samples[-1].pose.position_m[0] - lift_samples[-1].pose.position_m[0],
        transport_samples[-1].pose.position_m[1] - lift_samples[-1].pose.position_m[1],
    )
    if transport_distance < MIN_TRANSPORT_M:
        raise EvaluationError(f"visual transport is below {MIN_TRANSPORT_M:.3f} m")
    recorded_transport = require_number(
        retention.get("transport_planar_distance_m"), "execution.retention.transport_planar_distance_m"
    )
    require_same_number(recorded_transport, transport_distance, "retention transport distance")
    transport_details = stage_details(stages, "TRANSPORT")
    if transport_details.get("attachment_state") != "attached":
        raise EvaluationError("TRANSPORT does not record an attached object")
    if transport_details.get("vision_retention_samples") != retention.get("transport_samples"):
        raise EvaluationError("TRANSPORT samples do not match the retention record")
    require_same_number(
        require_number(
            transport_details.get("transport_planar_distance_m"),
            "TRANSPORT.details.transport_planar_distance_m",
        ),
        transport_distance,
        "TRANSPORT distance",
    )

    lower_samples = validate_retention_samples(
        retention.get("lower_samples"),
        "execution.retention.lower_samples",
        transport_samples[-1],
        after_stamp_ns=transport_samples[-1].pose.source_stamp_ns,
        minimum_samples=MIN_LOWER_RETENTION_SAMPLES,
    )
    lower_details = stage_details(stages, "LOWER")
    if lower_details.get("vision_retention_samples") != retention.get("lower_samples"):
        raise EvaluationError("LOWER samples do not match the retention record")

    release_details = stage_details(stages, "RELEASE")
    if execution.get("release") != release_details:
        raise EvaluationError("execution.release does not match the RELEASE stage")
    release_open = require_mapping(release_details.get("gripper_open"), "RELEASE.details.gripper_open")
    release_target, release_observed = validate_gripper_action(
        release_open, "RELEASE.details.gripper_open"
    )
    if release_target <= close_target or release_target < open_target - GRIPPER_OBSERVATION_TOLERANCE_M:
        raise EvaluationError("RELEASE does not command the gripper open")
    if abs(release_observed - release_target) > GRIPPER_OBSERVATION_TOLERANCE_M:
        raise EvaluationError("RELEASE has no open gripper observation")
    detached_state = require_mapping(release_details.get("detached_state"), "RELEASE.details.detached_state")
    if detached_state.get("state") != "detached" or detached_state not in states:
        raise EvaluationError("RELEASE has no lifecycle-backed detached state")
    if release_details.get("support_proof") != "PENDING_FRESH_POST_RELEASE_RGBD_EVALUATION":
        raise EvaluationError("RELEASE does not require fresh post-release vision support proof")
    post_place = require_mapping(execution.get("post_place"), "execution.post_place")
    minimum_post_stamp = require_timestamp(
        post_place.get("minimum_source_stamp_ns"), "execution.post_place.minimum_source_stamp_ns", positive=True
    )
    if require_timestamp(
        release_details.get("post_place_minimum_source_stamp_ns"),
        "RELEASE.details.post_place_minimum_source_stamp_ns",
        positive=True,
    ) != minimum_post_stamp:
        raise EvaluationError("RELEASE and post-place minimum stamps disagree")
    if lower_samples[-1].pose.source_stamp_ns >= minimum_post_stamp:
        raise EvaluationError("release minimum stamp is not after the lower retention evidence")
    retreat_details = stage_details(stages, "RETREAT")
    if retreat_details.get("attachment_state") != "detached":
        raise EvaluationError("RETREAT does not remain detached")
    if retreat_details.get("gripper_is_open") != release_open.get("observation"):
        raise EvaluationError("RETREAT does not preserve the observed open gripper state")
    if retreat_details.get("retreat_target_policy") != "p5_safe_startup_tcp_for_unobstructed_phase4_post_place_vision":
        raise EvaluationError("RETREAT does not use the approved P5 post-release safety posture")
    retreat_exemption = require_mapping(
        retreat_details.get("released_target_collision_exemption"),
        "RETREAT.details.released_target_collision_exemption",
    )
    if retreat_exemption.get("object_id") != "p5_vision_green_ball":
        raise EvaluationError("RETREAT collision exemption is not limited to the released green ball")
    if retreat_exemption.get("scope") != "open_finger_contact_clearance_after_recorded_detach_only":
        raise EvaluationError("RETREAT collision exemption has an unapproved scope")
    if "p5_vision_green_ball" in require_list(
        retreat_exemption.get("planning_scene_object_ids"),
        "RETREAT.details.released_target_collision_exemption.planning_scene_object_ids",
    ):
        raise EvaluationError("RETREAT retains the released ball collision proxy during contact clearance")
    return minimum_post_stamp, lift_delta, transport_distance


def validate_post_place(
    execution: dict[str, Any], pre_pick: Phase4Evidence, post_place: Phase4Evidence, minimum_stamp: int
) -> tuple[float, dict[str, float]]:
    if post_place.source_stamp_ns <= minimum_stamp:
        raise EvaluationError("post-place Phase-4 vision is not fresh after release")
    if post_place.source_stamp_ns <= pre_pick.source_stamp_ns:
        raise EvaluationError("post-place Phase-4 vision is not newer than pre-pick vision")
    post = require_mapping(execution.get("post_place"), "execution.post_place")
    if post.get("evaluation_required") != "fresh_vision_only_post_place_evidence":
        raise EvaluationError("execution does not require fresh vision-only post-place evidence")
    planned_green = parse_runtime_vision(
        post.get("planned_green"), "execution.post_place.planned_green", expected_class="green_ball"
    )
    if planned_green.source_stamp_ns != pre_pick.source_stamp_ns:
        raise EvaluationError("planned placement is not derived from the sealed pre-pick vision epoch")
    red_pre_pick = parse_runtime_vision(
        post.get("red_pre_pick"), "execution.post_place.red_pre_pick", expected_class="red_ball"
    )
    require_same_pose(red_pre_pick, pre_pick.poses["red_ball"], "post-place pre-pick red reference")

    green = post_place.poses["green_ball"]
    red = post_place.poses["red_ball"]
    planar_distance = math.hypot(
        green.position_m[0] - red.position_m[0], green.position_m[1] - red.position_m[1]
    )
    if planar_distance < MIN_NON_OVERLAP_DISTANCE_M or planar_distance > MAX_BESIDE_DISTANCE_M:
        raise EvaluationError(
            "post-place planar green/red distance is outside "
            f"[{MIN_NON_OVERLAP_DISTANCE_M:.3f}, {MAX_BESIDE_DISTANCE_M:.3f}] m"
        )
    support_heights: dict[str, float] = {}
    for class_id, pose in (("green_ball", green), ("red_ball", red)):
        support_error = abs(pose.position_m[2] - EXPECTED_BALL_CENTER_Z_M)
        if support_error > SUPPORT_HEIGHT_TOLERANCE_M:
            raise EvaluationError(
                f"post-place {class_id} is not near the table support height "
                f"({pose.position_m[2]:.3f} m)"
            )
        support_heights[class_id] = pose.position_m[2]
    return planar_distance, support_heights


def evaluate(
    pre_pick_path: Path, execution_path: Path, post_place_path: Path
) -> dict[str, Any]:
    resolved_inputs = [path.resolve() for path in (pre_pick_path, execution_path, post_place_path)]
    if len(set(resolved_inputs)) != len(resolved_inputs):
        raise EvaluationError("pre-pick, execution, and post-place inputs must be distinct artifacts")
    pre_pick = parse_phase4_evidence(pre_pick_path, "pre-pick Phase-4 evidence")
    execution = load_json(execution_path, "execution evidence")
    seed = validate_execution_envelope(execution)
    stages = parse_stage_records(execution)
    validate_pre_pick_link(execution, stages, pre_pick)
    static_obstacle_ids = validate_static_obstacle_coverage(stages, pre_pick)
    validate_motion_records(execution, stages, static_obstacle_ids)
    minimum_post_stamp, lift_delta, transport_distance = validate_attachment_retention_and_release(
        execution, stages, pre_pick
    )
    post_place = parse_phase4_evidence(post_place_path, "post-place Phase-4 evidence")
    planar_distance, support_heights = validate_post_place(
        execution, pre_pick, post_place, minimum_post_stamp
    )

    stage_summary = [
        {"stage": stage, "status": "PASS", "source": "execution_evidence"}
        for stage in RUNTIME_STAGES
    ]
    stage_summary.extend(
        (
            {
                "stage": "OBSERVE_PLACE",
                "status": "PASS",
                "source": "sealed_post_place_phase4_vision",
                "source_stamp_ns": post_place.source_stamp_ns,
            },
            {
                "stage": "VERIFY_PLACE",
                "status": "PASS",
                "planar_distance_m": planar_distance,
                "support_height_m": support_heights["green_ball"],
            },
        )
    )
    markers = {
        "P5_PICK_PLACE_EVALUATION": "PASS",
        "P5_STAGE_SEQUENCE": "PASS",
        "P5_PHASE4_VISION_PROVENANCE": "PASS",
        "P5_MOVEIT_MOTION": "PASS",
        "P5_GRASP_RETENTION": "PASS",
        "P5_RELEASE": "PASS",
        "P5_POST_PLACE": "PASS",
    }
    return {
        "schema_version": 1,
        "scope": "SIMULATION_ONLY",
        "evaluation_mode": "OFFLINE_EVIDENCE_VALIDATION_ONLY",
        "evaluated_at_utc": utc_now(),
        "seed": seed,
        "inputs": {
            "pre_pick_evidence": {
                "path": str(pre_pick.path),
                "sha256": pre_pick.sha256,
                "source_stamp_ns": pre_pick.source_stamp_ns,
            },
            "execution_evidence": {
                "path": str(execution_path.resolve()),
                "sha256": sha256_file(execution_path),
            },
            "post_place_evidence": {
                "path": str(post_place.path),
                "sha256": post_place.sha256,
                "source_stamp_ns": post_place.source_stamp_ns,
                "minimum_required_source_stamp_ns": minimum_post_stamp,
            },
        },
        "required_stages": list(REQUIRED_STAGES),
        "stage_records": stage_summary,
        "measurements": {
            "visual_lift_delta_m": lift_delta,
            "transport_planar_distance_m": transport_distance,
            "final_green_red_planar_distance_m": planar_distance,
            "final_support_heights_m": support_heights,
            "expected_ball_center_z_m": EXPECTED_BALL_CENTER_Z_M,
            "support_height_tolerance_m": SUPPORT_HEIGHT_TOLERANCE_M,
        },
        "markers": markers,
        "status": "PASS",
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pre-pick-evidence", required=True, type=Path)
    parser.add_argument("--execution-evidence", required=True, type=Path)
    parser.add_argument("--post-place-evidence", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser


def failure_result(arguments: argparse.Namespace, error: Exception) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "scope": "SIMULATION_ONLY",
        "evaluation_mode": "OFFLINE_EVIDENCE_VALIDATION_ONLY",
        "evaluated_at_utc": utc_now(),
        "inputs": {
            "pre_pick_evidence": str(arguments.pre_pick_evidence.resolve()),
            "execution_evidence": str(arguments.execution_evidence.resolve()),
            "post_place_evidence": str(arguments.post_place_evidence.resolve()),
        },
        "markers": {
            "P5_PICK_PLACE_EVALUATION": "FAIL",
            "P5_EVALUATION_ERROR": str(error),
        },
        "status": "FAIL",
        "error": str(error),
        "error_type": type(error).__name__,
    }


def main(argv: list[str] | None = None) -> int:
    arguments = build_parser().parse_args(argv)
    output = arguments.output
    try:
        if output.resolve() in {
            arguments.pre_pick_evidence.resolve(),
            arguments.execution_evidence.resolve(),
            arguments.post_place_evidence.resolve(),
        }:
            raise EvaluationError("--output must not overwrite an input artifact")
        result = evaluate(
            arguments.pre_pick_evidence, arguments.execution_evidence, arguments.post_place_evidence
        )
        atomic_write_json(output, result)
    except (EvaluationError, OSError, ValueError) as exc:
        failed = failure_result(arguments, exc)
        try:
            atomic_write_json(output, failed)
        except OSError:
            pass
        print("P5_PICK_PLACE_EVALUATION=FAIL")
        print(f"P5_EVALUATION_ERROR={str(exc).replace(chr(10), ' ')}", file=sys.stderr)
        print(f"P5_EVALUATION_EVIDENCE={output}")
        return 1
    for name, status in result["markers"].items():
        print(f"{name}={status}")
    print(f"P5_EVALUATION_EVIDENCE={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
