#!/usr/bin/env python3
"""Seal a real vision-only Phase-4 RGB-D observation before offline comparison.

This verifier subscribes only to ROS image, depth, CameraInfo, TF, and typed
vision outputs. It deliberately contains no simulator query and must finish
before the separate offline evaluator is invoked by the runtime harness.
"""

from __future__ import annotations

import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from typing import Any, Iterable

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from physical_ai_interfaces.msg import (
    ObjectDetection,
    ObjectDetectionArray,
    ObjectPose,
    ObjectPoseArray,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


EXPECTED_CLASSES = (
    "green_ball",
    "red_ball",
    "blue_cube",
    "yellow_cube",
    "drawing_board",
)
MAX_SENSOR_DELTA_NS = 50_000_000


class VerifierError(RuntimeError):
    """Raised when a required real vision observation is missing or invalid."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def stamp_ns(header) -> int:
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def header_dict(header) -> dict[str, Any]:
    return {
        "stamp_sec": int(header.stamp.sec),
        "stamp_nanosec": int(header.stamp.nanosec),
        "stamp_ns": stamp_ns(header),
        "frame_id": header.frame_id,
    }


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


def require_finite(value: float, label: str, *, positive: bool = False) -> float:
    number = float(value)
    if not math.isfinite(number) or (positive and number <= 0.0):
        raise VerifierError(f"{label} is not finite{' and positive' if positive else ''}")
    return number


def detection_dict(message: ObjectDetection) -> dict[str, Any]:
    return {
        "header": header_dict(message.header),
        "detection_id": message.detection_id,
        "class_id": message.class_id,
        "roi": {
            "x_offset": int(message.roi.x_offset),
            "y_offset": int(message.roi.y_offset),
            "width": int(message.roi.width),
            "height": int(message.roi.height),
        },
        "image_width": int(message.image_width),
        "image_height": int(message.image_height),
        "center_u_px": require_finite(message.center_u_px, "detection center_u_px"),
        "center_v_px": require_finite(message.center_v_px, "detection center_v_px"),
        "mask_pixel_count": int(message.mask_pixel_count),
        "confidence": require_finite(message.confidence, "detection confidence"),
        "source_mode": int(message.source_mode),
    }


def pose_dict(message: ObjectPose) -> dict[str, Any]:
    point = message.pose.pose.position
    return {
        "header": header_dict(message.header),
        "source_header": header_dict(message.source_header),
        "detection_id": message.detection_id,
        "class_id": message.class_id,
        "position_m": [
            require_finite(point.x, "pose x"),
            require_finite(point.y, "pose y"),
            require_finite(point.z, "pose z"),
        ],
        "confidence": require_finite(message.confidence, "pose confidence"),
        "depth_m": require_finite(message.depth_m, "pose depth", positive=True),
        "depth_mad_m": require_finite(message.depth_mad_m, "pose depth MAD"),
        "valid_depth_samples": int(message.valid_depth_samples),
        "orientation_valid": bool(message.orientation_valid),
        "source_mode": int(message.source_mode),
    }


def transform_dict(transform: TransformStamped) -> dict[str, Any]:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return {
        "header": header_dict(transform.header),
        "child_frame_id": transform.child_frame_id,
        "translation_m": [translation.x, translation.y, translation.z],
        "rotation_xyzw": [rotation.x, rotation.y, rotation.z, rotation.w],
    }


class VisionEvidenceVerifier(Node):
    """Wait for one coherent five-object vision observation and seal it."""

    def __init__(self, timeout_sec: float, minimum_source_stamp_sec: float) -> None:
        super().__init__(
            "phase4_vision_evidence_verifier",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.deadline = time.monotonic() + timeout_sec
        self.minimum_source_stamp_ns = int(minimum_source_stamp_sec * 1_000_000_000)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.rgb_messages: list[Image] = []
        self.depth_messages: list[Image] = []
        self.info_messages: list[CameraInfo] = []
        self.detection_arrays: list[ObjectDetectionArray] = []
        self.camera_pose_arrays: list[ObjectPoseArray] = []
        self.world_pose_arrays: list[ObjectPoseArray] = []
        self.completed: tuple[
            Image,
            Image,
            CameraInfo,
            ObjectDetectionArray,
            ObjectPoseArray,
            ObjectPoseArray,
        ] | None = None
        self.last_class_set: set[str] = set()
        self.create_subscription(
            Image, "/lab_camera/color/image_raw", self._on_rgb, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/lab_camera/depth/image_raw", self._on_depth, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            "/lab_camera/color/camera_info",
            self._on_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ObjectDetectionArray,
            "/perception/detections",
            self._on_detections,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ObjectPoseArray,
            "/perception/camera_object_poses",
            self._on_camera_poses,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ObjectPoseArray,
            "/perception/world_object_poses",
            self._on_world_poses,
            qos_profile_sensor_data,
        )

    @staticmethod
    def _trim(messages: list[Any]) -> None:
        if len(messages) > 120:
            del messages[:-80]

    def _on_rgb(self, message: Image) -> None:
        self.rgb_messages.append(copy.deepcopy(message))
        self._trim(self.rgb_messages)
        self._attempt_complete()

    def _on_depth(self, message: Image) -> None:
        self.depth_messages.append(copy.deepcopy(message))
        self._trim(self.depth_messages)
        self._attempt_complete()

    def _on_info(self, message: CameraInfo) -> None:
        self.info_messages.append(copy.deepcopy(message))
        self._trim(self.info_messages)
        self._attempt_complete()

    def _on_detections(self, message: ObjectDetectionArray) -> None:
        self.detection_arrays.append(copy.deepcopy(message))
        self._trim(self.detection_arrays)
        self._attempt_complete()

    def _on_camera_poses(self, message: ObjectPoseArray) -> None:
        self.camera_pose_arrays.append(copy.deepcopy(message))
        self._trim(self.camera_pose_arrays)
        self._attempt_complete()

    def _on_world_poses(self, message: ObjectPoseArray) -> None:
        self.world_pose_arrays.append(copy.deepcopy(message))
        self._trim(self.world_pose_arrays)
        self.last_class_set = {pose.class_id for pose in message.poses}
        self._attempt_complete()

    @staticmethod
    def _nearest(messages: Iterable[Any], target_ns: int) -> Any | None:
        candidates = [(abs(stamp_ns(message.header) - target_ns), message) for message in messages]
        if not candidates:
            return None
        delta, selected = min(candidates, key=lambda item: item[0])
        return selected if delta <= MAX_SENSOR_DELTA_NS else None

    @staticmethod
    def _pose_map(message: ObjectPoseArray) -> dict[str, ObjectPose] | None:
        labels = [pose.class_id for pose in message.poses]
        if len(labels) != len(EXPECTED_CLASSES) or set(labels) != set(EXPECTED_CLASSES):
            return None
        if len(set(labels)) != len(labels):
            return None
        if any(pose.source_mode != ObjectPose.SOURCE_VISION for pose in message.poses):
            return None
        if any(pose.orientation_valid for pose in message.poses):
            return None
        if any(pose.valid_depth_samples <= 0 for pose in message.poses):
            return None
        return {pose.class_id: pose for pose in message.poses}

    def _attempt_complete(self) -> None:
        if self.completed is not None:
            return
        for world_array in reversed(self.world_pose_arrays):
            world_map = self._pose_map(world_array)
            if world_map is None:
                continue
            source_header = next(iter(world_map.values())).source_header
            source_stamp = stamp_ns(source_header)
            if source_stamp < self.minimum_source_stamp_ns:
                continue
            if any(
                stamp_ns(pose.source_header) != source_stamp
                or pose.source_header.frame_id != source_header.frame_id
                or pose.header.frame_id != "world"
                for pose in world_map.values()
            ):
                continue
            detections = self._nearest(self.detection_arrays, source_stamp)
            camera_poses = self._nearest(self.camera_pose_arrays, source_stamp)
            rgb = self._nearest(self.rgb_messages, source_stamp)
            depth = self._nearest(self.depth_messages, source_stamp)
            info = self._nearest(self.info_messages, source_stamp)
            if any(item is None for item in (detections, camera_poses, rgb, depth, info)):
                continue
            assert isinstance(detections, ObjectDetectionArray)
            assert isinstance(camera_poses, ObjectPoseArray)
            assert isinstance(rgb, Image)
            assert isinstance(depth, Image)
            assert isinstance(info, CameraInfo)
            camera_map = self._pose_map(camera_poses)
            detection_map = {item.class_id: item for item in detections.detections}
            if (
                camera_map is None
                or len(detection_map) != len(EXPECTED_CLASSES)
                or set(detection_map) != set(EXPECTED_CLASSES)
            ):
                continue
            if any(item.source_mode != ObjectDetection.SOURCE_VISION for item in detection_map.values()):
                continue
            if any(
                pose.detection_id != detection_map[label].detection_id
                or camera_map[label].detection_id != detection_map[label].detection_id
                or camera_map[label].header.frame_id != source_header.frame_id
                for label, pose in world_map.items()
            ):
                continue
            if (
                rgb.header.frame_id != source_header.frame_id
                or depth.header.frame_id != source_header.frame_id
                or info.header.frame_id != source_header.frame_id
                or not source_header.frame_id
            ):
                continue
            if rgb.width != depth.width or rgb.height != depth.height:
                continue
            if rgb.width != info.width or rgb.height != info.height:
                continue
            self.completed = (rgb, depth, info, detections, camera_poses, world_array)
            return

    def wait_for_completed(self) -> tuple[Image, Image, CameraInfo, ObjectDetectionArray, ObjectPoseArray, ObjectPoseArray]:
        while time.monotonic() < self.deadline:
            if self.completed is not None:
                return self.completed
            rclpy.spin_once(self, timeout_sec=0.1)
        observed = ",".join(sorted(self.last_class_set)) or "none"
        raise VerifierError(
            f"timed out waiting for one coherent five-object vision result; last classes={observed}"
        )


def decode_depth(bridge: CvBridge, message: Image) -> np.ndarray:
    try:
        raw = bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
    except CvBridgeError as exc:
        raise VerifierError(f"cannot decode depth: {exc}") from exc
    if message.encoding == "32FC1":
        depth = np.asarray(raw, dtype=np.float32)
    elif message.encoding == "16UC1":
        depth = np.asarray(raw, dtype=np.float32) / 1000.0
    else:
        raise VerifierError(f"unsupported observed depth encoding {message.encoding!r}")
    if depth.ndim != 2:
        raise VerifierError("depth image is not single-channel")
    return depth


def wait_for_world_to_camera_transform(
    verifier: VisionEvidenceVerifier, source_header: Header
):
    """Return the observed static/dynamic TF without racing DDS discovery.

    The RGB-D stream may produce a coherent first observation before a fresh
    verifier's ``TransformListener`` has received the latched ``/tf_static``
    camera chain.  This is a readiness race, not evidence of a valid TF path.
    Keep the original source timestamp and wait only until the verifier's
    existing deadline; a missing path remains an explicit failure.
    """

    last_error: TransformException | None = None
    while time.monotonic() < verifier.deadline:
        try:
            return verifier.tf_buffer.lookup_transform(
                "world",
                source_header.frame_id,
                Time.from_msg(source_header.stamp),
                timeout=Duration(seconds=0.25),
            )
        except TransformException as exc:
            last_error = exc
            rclpy.spin_once(verifier, timeout_sec=0.1)
    detail = str(last_error) if last_error is not None else "no TF lookup attempted"
    raise VerifierError(f"cannot record world-to-camera TF before deadline: {detail}")


def save_evidence(
    output_root: Path,
    verifier: VisionEvidenceVerifier,
    completed: tuple[Image, Image, CameraInfo, ObjectDetectionArray, ObjectPoseArray, ObjectPoseArray],
) -> Path:
    rgb, depth, info, detections, camera_poses, world_poses = completed
    evidence_root = output_root / "evidence"
    evidence_root.mkdir(parents=True, exist_ok=True)
    try:
        bgr = verifier.bridge.imgmsg_to_cv2(rgb, desired_encoding="bgr8")
    except CvBridgeError as exc:
        raise VerifierError(f"cannot decode RGB: {exc}") from exc
    success, encoded = cv2.imencode(".png", bgr)
    if not success:
        raise VerifierError("cannot encode RGB PNG")
    rgb_path = evidence_root / "rgb.png"
    rgb_path.write_bytes(encoded.tobytes())
    annotated = bgr.copy()
    for detection in detections.detections:
        top_left = (int(detection.roi.x_offset), int(detection.roi.y_offset))
        bottom_right = (
            int(detection.roi.x_offset + detection.roi.width),
            int(detection.roi.y_offset + detection.roi.height),
        )
        cv2.rectangle(annotated, top_left, bottom_right, (255, 255, 255), 1)
        cv2.putText(
            annotated,
            detection.class_id,
            (top_left[0], max(12, top_left[1] - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    success, encoded = cv2.imencode(".png", annotated)
    if not success:
        raise VerifierError("cannot encode annotated RGB PNG")
    annotated_path = evidence_root / "annotated_rgb.png"
    annotated_path.write_bytes(encoded.tobytes())
    depth_array = decode_depth(verifier.bridge, depth)
    depth_path = evidence_root / "depth.npy"
    with depth_path.open("wb") as stream:
        np.save(stream, depth_array, allow_pickle=False)
    finite_positive = depth_array[np.isfinite(depth_array) & (depth_array > 0.0)]
    if finite_positive.size == 0:
        raise VerifierError("observed depth contains no finite positive sample")
    camera_info_path = evidence_root / "camera_info.json"
    atomic_write_json(
        camera_info_path,
        {
            "header": header_dict(info.header),
            "width": int(info.width),
            "height": int(info.height),
            "distortion_model": info.distortion_model,
            "d": [float(value) for value in info.d],
            "k": [float(value) for value in info.k],
            "r": [float(value) for value in info.r],
            "p": [float(value) for value in info.p],
        },
    )
    source_header = world_poses.poses[0].source_header
    transform = wait_for_world_to_camera_transform(verifier, source_header)
    tf_path = evidence_root / "tf_chain.json"
    atomic_write_json(tf_path, transform_dict(transform))
    output_path = evidence_root / "vision_outputs.json"
    atomic_write_json(
        output_path,
        {
            "detections": [detection_dict(item) for item in detections.detections],
            "camera_poses": [pose_dict(item) for item in camera_poses.poses],
            "world_poses": [pose_dict(item) for item in world_poses.poses],
        },
    )
    files = (rgb_path, annotated_path, depth_path, camera_info_path, tf_path, output_path)
    artifact_hashes = {
        path.relative_to(output_root).as_posix(): sha256_file(path) for path in files
    }
    evidence_path = evidence_root / "vision_evidence.json"
    atomic_write_json(
        evidence_path,
        {
            "schema_version": 1,
            "scope": "SIMULATION_ONLY",
            "sealed_at_utc": utc_now(),
            "perception_mode": "vision",
            "ground_truth_input_used": False,
            "expected_classes": list(EXPECTED_CLASSES),
            "source_header": header_dict(source_header),
            "minimum_source_stamp_ns": verifier.minimum_source_stamp_ns,
            "rgb": {
                "header": header_dict(rgb.header),
                "encoding": rgb.encoding,
                "width": int(rgb.width),
                "height": int(rgb.height),
            },
            "depth": {
                "header": header_dict(depth.header),
                "encoding": depth.encoding,
                "width": int(depth.width),
                "height": int(depth.height),
                "unit": "m",
                "finite_positive_samples": int(finite_positive.size),
                "minimum_m": float(np.min(finite_positive)),
                "median_m": float(np.median(finite_positive)),
                "maximum_m": float(np.max(finite_positive)),
            },
            "artifact_sha256": artifact_hashes,
            "status": {
                "p4_rgb_transport": "PASS",
                "p4_depth_transport": "PASS",
                "p4_camera_info": "PASS",
                "p4_tf_chain": "PASS",
                "p4_all_five_typed_outputs": "PASS",
                "p4_vision_input_boundary": "PASS",
            },
        },
    )
    return evidence_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--minimum-source-stamp-sec", type=float, default=3.0)
    return parser


def main() -> int:
    arguments = build_parser().parse_args()
    verifier: VisionEvidenceVerifier | None = None
    evidence_path: Path | None = None
    try:
        if not math.isfinite(arguments.timeout) or arguments.timeout <= 0.0:
            raise VerifierError("--timeout must be a finite positive number")
        if (
            not math.isfinite(arguments.minimum_source_stamp_sec)
            or arguments.minimum_source_stamp_sec < 0.0
        ):
            raise VerifierError("--minimum-source-stamp-sec must be finite and nonnegative")
        rclpy.init(args=None)
        verifier = VisionEvidenceVerifier(
            arguments.timeout, arguments.minimum_source_stamp_sec
        )
        completed = verifier.wait_for_completed()
        evidence_path = save_evidence(arguments.output_root, verifier, completed)
    except (VerifierError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if verifier is not None:
            verifier.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    assert evidence_path is not None
    print("P4_RGB_TRANSPORT=PASS")
    print("P4_DEPTH_TRANSPORT=PASS")
    print("P4_CAMERA_INFO=PASS")
    print("P4_TF_CHAIN=PASS")
    print("P4_ALL_FIVE_TYPED_OUTPUTS=PASS")
    print("P4_VISION_INPUT_BOUNDARY=PASS")
    print(f"SEALED_VISION_EVIDENCE={evidence_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
