#!/usr/bin/env python3
"""Execute one fail-closed, vision-driven Phase-5 pick/place episode.

The node deliberately has no Gazebo pose, model-state, seed-manifest, or
world-ground-truth input.  It consumes the sealed pre-pick RGB-D evidence plus
live typed vision, TF, MoveIt and controller state.  Gazebo is contacted only
to dynamically add the simulation-only retention mechanism *after* the fresh
vision/TCP and observed-close guard has passed; it is never queried for an
object pose.
"""

from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any, Callable, Iterable

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import ParallelGripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    AttachedCollisionObject,
    BoundingVolume,
    CollisionObject,
    Constraints,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPlanningScene
from physical_ai_interfaces.msg import ObjectPoseArray
import rclpy
from rclpy.action import ActionClient
from rclpy.client import Client
from rclpy.duration import Duration as RosDuration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


STAGES = (
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
SUCCESS = MoveItErrorCodes.SUCCESS


class PickPlaceError(RuntimeError):
    """A safety/evidence gate failed and the episode must stop."""


@dataclass(frozen=True)
class VisionPose:
    """A finite, vision-only world-frame object observation."""

    class_id: str
    detection_id: str
    x: float
    y: float
    z: float
    stamp_ns: int
    confidence: float
    source_stream: str = "primary_rgbd"

    def as_dict(self) -> dict[str, Any]:
        return {
            "class_id": self.class_id,
            "detection_id": self.detection_id,
            "position_m": [self.x, self.y, self.z],
            "source_stamp_ns": self.stamp_ns,
            "confidence": self.confidence,
            "source_mode": "VISION",
            "source_stream": self.source_stream,
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
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def finite(value: Any, label: str, *, positive: bool = False) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise PickPlaceError(f"{label} must be numeric") from exc
    if not math.isfinite(number) or (positive and number <= 0.0):
        raise PickPlaceError(f"{label} must be finite{' and positive' if positive else ''}")
    return number


def finite_int(value: Any, label: str, *, minimum: int = 0) -> int:
    if isinstance(value, bool):
        raise PickPlaceError(f"{label} must be an integer")
    try:
        number = int(value)
    except (TypeError, ValueError) as exc:
        raise PickPlaceError(f"{label} must be an integer") from exc
    if number < minimum or number != value:
        raise PickPlaceError(f"{label} must be an integer >= {minimum}")
    return number


def mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise PickPlaceError(f"{label} must be a mapping")
    return value


def list_of_numbers(value: Any, label: str, length: int) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise PickPlaceError(f"{label} must contain exactly {length} values")
    return [finite(item, f"{label}[{index}]") for index, item in enumerate(value)]


def stamp_ns(header: Any) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def quaternion_x_axis(quaternion: Iterable[float]) -> tuple[float, float, float]:
    x, y, z, w = (finite(item, "orientation component") for item in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 0.999 or norm > 1.001:
        raise PickPlaceError("orientation candidate must be a unit quaternion")
    return (
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + z * w),
        2.0 * (x * z - y * w),
    )


def planar_distance(first: VisionPose, second: VisionPose) -> float:
    return math.hypot(first.x - second.x, first.y - second.y)


def distance3(left: dict[str, float], right: dict[str, float]) -> float:
    return math.sqrt(
        (left["x"] - right["x"]) ** 2
        + (left["y"] - right["y"]) ** 2
        + (left["z"] - right["z"]) ** 2
    )


def pose_from_output(entry: Any, label: str) -> VisionPose:
    item = mapping(entry, label)
    if item.get("source_mode") != 1:
        raise PickPlaceError(f"{label} is not vision-sourced")
    header = mapping(item.get("header"), f"{label}.header")
    source_header = mapping(item.get("source_header"), f"{label}.source_header")
    if header.get("frame_id") != "world" or source_header.get("frame_id") != "lab_camera_optical_frame":
        raise PickPlaceError(f"{label} does not carry the expected world/camera frame chain")
    source_stamp = finite_int(source_header.get("stamp_ns"), f"{label}.source_stamp_ns", minimum=1)
    position = list_of_numbers(item.get("position_m"), f"{label}.position_m", 3)
    confidence = finite(item.get("confidence"), f"{label}.confidence", positive=True)
    if confidence < 0.60:
        raise PickPlaceError(f"{label} confidence is below the 0.60 Phase-5 floor")
    class_id = item.get("class_id")
    detection_id = item.get("detection_id")
    if not isinstance(class_id, str) or not class_id or not isinstance(detection_id, str) or not detection_id:
        raise PickPlaceError(f"{label} lacks a class or detection id")
    return VisionPose(
        class_id=class_id,
        detection_id=detection_id,
        x=position[0],
        y=position[1],
        z=position[2],
        stamp_ns=source_stamp,
        confidence=confidence,
        source_stream="sealed_primary_rgbd",
    )


def load_pre_pick_evidence(path: Path) -> tuple[dict[str, VisionPose], dict[str, Any]]:
    """Load a sealed Phase-4 artifact and reject stale/mutated provenance."""

    if not path.is_file():
        raise PickPlaceError(f"pre-pick evidence is missing: {path}")
    root = path.parent.parent
    payload = json.loads(path.read_text(encoding="utf-8"))
    evidence = mapping(payload, "pre-pick evidence")
    if evidence.get("scope") != "SIMULATION_ONLY" or evidence.get("perception_mode") != "vision":
        raise PickPlaceError("pre-pick evidence is not the required simulation vision mode")
    if evidence.get("ground_truth_input_used") is not False:
        raise PickPlaceError("pre-pick evidence reports a ground-truth input")
    status = mapping(evidence.get("status"), "pre-pick status")
    required_statuses = {
        "p4_rgb_transport",
        "p4_depth_transport",
        "p4_camera_info",
        "p4_tf_chain",
        "p4_all_five_typed_outputs",
        "p4_vision_input_boundary",
    }
    if any(status.get(name) != "PASS" for name in required_statuses):
        raise PickPlaceError("pre-pick Phase-4 transport/vision evidence is incomplete")
    artifacts = mapping(evidence.get("artifact_sha256"), "pre-pick artifact hashes")
    for relative, expected_hash in artifacts.items():
        if not isinstance(relative, str) or not isinstance(expected_hash, str):
            raise PickPlaceError("pre-pick artifact hash entry is malformed")
        artifact = root / relative
        if not artifact.is_file() or sha256_file(artifact) != expected_hash:
            raise PickPlaceError(f"pre-pick artifact does not match sealed hash: {relative}")
    outputs_path = root / "evidence" / "vision_outputs.json"
    if not outputs_path.is_file():
        raise PickPlaceError("pre-pick vision outputs are missing")
    outputs = mapping(json.loads(outputs_path.read_text(encoding="utf-8")), "vision outputs")
    pose_entries = outputs.get("world_poses")
    if not isinstance(pose_entries, list):
        raise PickPlaceError("pre-pick world poses are missing")
    by_class: dict[str, VisionPose] = {}
    for index, entry in enumerate(pose_entries):
        observed = pose_from_output(entry, f"pre-pick world_poses[{index}]")
        if observed.class_id in by_class:
            raise PickPlaceError(f"pre-pick has duplicate {observed.class_id} observations")
        by_class[observed.class_id] = observed
    required_classes = {"green_ball", "red_ball", "blue_cube", "yellow_cube", "drawing_board"}
    missing = required_classes.difference(by_class)
    if missing:
        raise PickPlaceError(f"pre-pick vision output is missing: {sorted(missing)}")
    source_header = mapping(evidence.get("source_header"), "pre-pick source header")
    sealed_stamp = finite_int(source_header.get("stamp_ns"), "pre-pick source stamp", minimum=1)
    if any(item.stamp_ns != sealed_stamp for item in by_class.values()):
        raise PickPlaceError("pre-pick objects do not share one sealed RGB-D source stamp")
    return by_class, evidence


def load_static_obstacle_geometry() -> dict[str, Any]:
    """Load only fixed P5 marker/collision geometry, never a scene layout.

    The separate P5 localizer publishes the two marker poses from RGB-D.  This
    helper reads its public class/geometry contract so the runtime cannot
    silently disagree about marker labels, the top-to-centre transform, or
    conservative collision-box dimensions.  It contains no seed, manifest,
    or Gazebo model state.
    """

    config_path = (
        Path(get_package_share_directory("abb_lab_moveit_config"))
        / "config"
        / "phase5_static_obstacle_vision.yaml"
    )
    if not config_path.is_file():
        raise PickPlaceError(f"P5 static-obstacle geometry config is missing: {config_path}")
    try:
        document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise PickPlaceError(f"cannot parse P5 static-obstacle geometry config: {exc}") from exc
    geometry = mapping(
        mapping(document, "P5 static-obstacle config").get("phase5_static_obstacle_geometry"),
        "P5 static-obstacle geometry",
    )
    topic = geometry.get("output_topic")
    if topic != "/phase5/perception/static_obstacle_poses":
        raise PickPlaceError("P5 static-obstacle geometry declares an unexpected output topic")
    marker_ids = geometry.get("expected_marker_class_ids")
    if (
        not isinstance(marker_ids, list)
        or len(marker_ids) != 2
        or any(not isinstance(item, str) or not item for item in marker_ids)
        or len(set(marker_ids)) != 2
    ):
        raise PickPlaceError("P5 static-obstacle geometry must declare exactly two marker classes")
    return {
        "output_topic": topic,
        "expected_marker_class_ids": tuple(marker_ids),
        "marker_top_to_box_center_z_offset_m": finite(
            geometry.get("marker_top_to_box_center_z_offset_m"),
            "P5 static-obstacle top-to-centre offset",
            positive=True,
        ),
        "collision_box_size_xyz_m": list_of_numbers(
            geometry.get("collision_box_size_xyz_m"),
            "P5 static-obstacle collision box size",
            3,
        ),
        "confidence_floor": finite(
            geometry.get("confidence_floor"),
            "P5 static-obstacle confidence floor",
            positive=True,
        ),
        "config_path": str(config_path),
    }


class PickPlaceRuntime(Node):
    """A bounded ROS client for one deterministic, evidence-bearing episode."""

    def __init__(
        self,
        config: dict[str, Any],
        pre_pick: dict[str, VisionPose],
        pre_evidence: dict[str, Any],
        output_root: Path,
        seed: int,
        timeout_sec: float,
    ) -> None:
        super().__init__(
            "verified_pick_place",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.config = config
        self.pre_pick = pre_pick
        self.pre_evidence = pre_evidence
        self.output_root = output_root
        self.seed = seed
        self.deadline = time.monotonic() + timeout_sec
        self.robot = mapping(config.get("robot"), "robot config")
        self.planner = mapping(config.get("planner"), "planner config")
        self.gripper = mapping(config.get("gripper"), "gripper config")
        self.geometry = mapping(config.get("geometry"), "geometry config")
        self.motion = mapping(config.get("motion"), "motion config")
        self.placement = mapping(config.get("placement"), "placement config")
        self.attachment = mapping(config.get("attachment"), "attachment config")
        self.static_obstacle_geometry = load_static_obstacle_geometry()
        self.static_obstacle_topic = str(self.static_obstacle_geometry["output_topic"])
        self.static_marker_classes = tuple(
            self.static_obstacle_geometry["expected_marker_class_ids"]
        )
        self.arm_joints = tuple(self.robot.get("arm_joint_names", ()))
        if self.arm_joints != tuple(f"joint_{index}" for index in range(1, 7)):
            raise PickPlaceError("Phase-5 runtime requires exactly the bounded six-axis chain")
        self.gripper_joint = str(self.robot.get("gripper_joint"))
        self.planning_frame = str(self.robot.get("planning_frame"))
        self.tcp_link = str(self.robot.get("tcp_link"))
        if self.planning_frame != "world" or self.tcp_link != "gripper_tcp":
            raise PickPlaceError("unexpected Phase-5 planning frame or TCP link")
        self.velocity_limits = {
            name: finite(
                mapping(self.robot.get("velocity_limits_rad_per_sec"), "velocity limits").get(name),
                f"velocity limit {name}",
                positive=True,
            )
            for name in self.arm_joints
        }
        self.plan_client: Client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.scene_client: Client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.scene_query_client: Client = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.gripper_client = ActionClient(
            self, ParallelGripperCommand, "/two_finger_gripper_controller/gripper_cmd"
        )
        self.detach_publisher = self.create_publisher(
            Empty, str(self.attachment.get("detach_topic")), 10
        )
        self.attach_publisher = self.create_publisher(Empty, "/phase5/grasp/attach", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_joint_positions: dict[str, float] | None = None
        self.latest_joint_sequence = 0
        self.latest_vision: dict[str, VisionPose] = {}
        self.latest_guard_vision: dict[str, VisionPose] = {}
        self.latest_transport_vision: dict[str, VisionPose] = {}
        self.vision_sequence = 0
        self.latest_static_obstacles: dict[str, VisionPose] = {}
        self.static_obstacle_sequence = 0
        self.static_obstacle_records: list[dict[str, Any]] = []
        self.attachment_states: list[dict[str, Any]] = []
        self.current_stage = "INITIALIZE"
        self.stage_records: list[dict[str, Any]] = []
        self.motion_records: list[dict[str, Any]] = []
        self._attachment_state = "UNKNOWN"
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, qos_profile_sensor_data
        )
        self.create_subscription(
            ObjectPoseArray,
            "/perception/world_object_poses",
            self._on_vision,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ObjectPoseArray,
            "/phase5/perception/guard_world_object_poses",
            self._on_guard_vision,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ObjectPoseArray,
            "/phase5/perception/transport_world_object_poses",
            self._on_transport_vision,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ObjectPoseArray,
            self.static_obstacle_topic,
            self._on_static_obstacle_vision,
            qos_profile_sensor_data,
        )
        self.create_subscription(String, str(self.attachment.get("state_topic")), self._on_attachment, 10)

    def remaining(self) -> float:
        return self.deadline - time.monotonic()

    def sim_time_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def stage_time(self) -> dict[str, Any]:
        return {"wall_time_utc": utc_now(), "sim_time_ns": self.sim_time_ns()}

    def _bounded_deadline(self, requested_sec: float, label: str) -> float:
        duration = finite(requested_sec, f"{label} timeout", positive=True)
        remaining = self.remaining()
        if remaining <= 0.0:
            raise PickPlaceError(f"global episode timeout expired before {label}")
        return time.monotonic() + min(duration, remaining)

    def _spin_until(self, predicate: Callable[[], bool], label: str, requested_sec: float) -> None:
        end = self._bounded_deadline(requested_sec, label)
        while time.monotonic() < end:
            if predicate():
                return
            rclpy.spin_once(self, timeout_sec=min(0.10, max(0.0, end - time.monotonic())))
        if not predicate():
            raise PickPlaceError(f"timed out waiting for {label}")

    def _on_joint_state(self, message: JointState) -> None:
        positions = {str(name): float(value) for name, value in zip(message.name, message.position)}
        required = (*self.arm_joints, self.gripper_joint)
        if any(name not in positions or not math.isfinite(positions[name]) for name in required):
            return
        self.latest_joint_positions = {name: positions[name] for name in required}
        self.latest_joint_sequence += 1

    def _record_vision(
        self,
        message: ObjectPoseArray,
        destination: dict[str, VisionPose],
        source_stream: str,
    ) -> None:
        observed: dict[str, VisionPose] = {}
        for item in message.poses:
            if item.source_mode != item.SOURCE_VISION or item.header.frame_id != "world":
                continue
            source_stamp = stamp_ns(item.source_header)
            values = (item.pose.pose.position.x, item.pose.pose.position.y, item.pose.pose.position.z)
            if source_stamp <= 0 or not all(math.isfinite(float(value)) for value in values):
                continue
            if not item.class_id or not item.detection_id or float(item.confidence) < 0.60:
                continue
            observed[str(item.class_id)] = VisionPose(
                class_id=str(item.class_id),
                detection_id=str(item.detection_id),
                x=float(values[0]),
                y=float(values[1]),
                z=float(values[2]),
                stamp_ns=source_stamp,
                confidence=float(item.confidence),
                source_stream=source_stream,
            )
        if observed:
            # An end-effector can occlude the green ball in later RGB-D frames
            # while it completes a collision-checked approach. Retain the most
            # recent typed observation of each class instead of deleting that
            # class because another valid frame omits it. `wait_fresh_vision`
            # still requires the retained source stamp to be newer than the
            # current action start, so this never revives a pre-action pose.
            destination.update(observed)
            self.vision_sequence += 1

    def _on_vision(self, message: ObjectPoseArray) -> None:
        self._record_vision(message, self.latest_vision, "primary_rgbd")

    def _on_guard_vision(self, message: ObjectPoseArray) -> None:
        self._record_vision(message, self.latest_guard_vision, "phase5_guard_rgbd")

    def _on_transport_vision(self, message: ObjectPoseArray) -> None:
        self._record_vision(message, self.latest_transport_vision, "phase5_transport_rgbd")

    def _on_static_obstacle_vision(self, message: ObjectPoseArray) -> None:
        """Accept one coherent RGB-D epoch for both P5 obstacle fiducials."""

        observed: dict[str, VisionPose] = {}
        expected = set(self.static_marker_classes)
        confidence_floor = float(self.static_obstacle_geometry["confidence_floor"])
        for item in message.poses:
            class_id = str(item.class_id)
            if class_id not in expected:
                continue
            if (
                item.source_mode != item.SOURCE_VISION
                or item.header.frame_id != self.planning_frame
                or class_id in observed
            ):
                return
            source_stamp = stamp_ns(item.source_header)
            values = (
                item.pose.pose.position.x,
                item.pose.pose.position.y,
                item.pose.pose.position.z,
            )
            if (
                source_stamp <= 0
                or not all(math.isfinite(float(value)) for value in values)
                or not item.detection_id
                or float(item.confidence) < confidence_floor
            ):
                return
            observed[class_id] = VisionPose(
                class_id=class_id,
                detection_id=str(item.detection_id),
                x=float(values[0]),
                y=float(values[1]),
                z=float(values[2]),
                stamp_ns=source_stamp,
                confidence=float(item.confidence),
            )
        if set(observed) != expected:
            return
        if len({item.stamp_ns for item in observed.values()}) != 1:
            return
        self.latest_static_obstacles = observed
        self.static_obstacle_sequence += 1

    def _on_attachment(self, message: String) -> None:
        state = str(message.data).strip().lower()
        if state not in {"attached", "detached"}:
            return
        self._attachment_state = state
        self.attachment_states.append({"state": state, **self.stage_time()})

    def wait_for_endpoints(self) -> None:
        service_timeout = finite(
            self.planner.get("service_wait_timeout_sec"), "service wait timeout", positive=True
        )
        for client, name in (
            (self.plan_client, "/plan_kinematic_path"),
            (self.scene_client, "/apply_planning_scene"),
            (self.scene_query_client, "/get_planning_scene"),
        ):
            end = self._bounded_deadline(service_timeout, f"service {name}")
            while time.monotonic() < end:
                if client.wait_for_service(timeout_sec=min(0.2, max(0.0, end - time.monotonic()))):
                    break
            else:
                raise PickPlaceError(f"MoveIt service is unavailable: {name}")
        for client, name in (
            (self.execute_client, "/execute_trajectory"),
            (self.gripper_client, "/two_finger_gripper_controller/gripper_cmd"),
        ):
            end = self._bounded_deadline(service_timeout, f"action {name}")
            while time.monotonic() < end:
                if client.wait_for_server(timeout_sec=min(0.2, max(0.0, end - time.monotonic()))):
                    break
            else:
                raise PickPlaceError(f"action server is unavailable: {name}")
        self._spin_until(
            lambda: self.latest_joint_positions is not None,
            "a bounded-chain joint-state observation",
            service_timeout,
        )

    def call_service(self, client: Client, request: Any, label: str) -> Any:
        future = client.call_async(request)
        self._spin_until(
            future.done,
            f"service result for {label}",
            finite(self.planner.get("service_call_timeout_sec"), "service call timeout", positive=True),
        )
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            raise PickPlaceError(f"service failure for {label}: {exc}") from exc
        if result is None:
            raise PickPlaceError(f"service returned no result for {label}")
        return result

    def robot_state(self, arm_positions: dict[str, float]) -> RobotState:
        state = RobotState()
        state.is_diff = False
        names = [*self.arm_joints, self.gripper_joint]
        state.joint_state.name = names
        gripper_position = self.current_positions()[self.gripper_joint]
        state.joint_state.position = [
            *[float(arm_positions[name]) for name in self.arm_joints], gripper_position
        ]
        return state

    def current_positions(self) -> dict[str, float]:
        self._spin_until(
            lambda: self.latest_joint_positions is not None,
            "current joint state",
            finite(self.planner.get("endpoint_observation_timeout_sec"), "endpoint timeout", positive=True),
        )
        assert self.latest_joint_positions is not None
        return dict(self.latest_joint_positions)

    def pose_stamped(self, position: tuple[float, float, float], quaternion: list[float]) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.planning_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion
        return pose

    def build_pose_request(self, arm_positions: dict[str, float], target: PoseStamped) -> MotionPlanRequest:
        request = MotionPlanRequest()
        request.group_name = str(self.robot.get("group_name"))
        request.pipeline_id = str(self.planner.get("pipeline_id"))
        request.planner_id = str(self.planner.get("planner_id"))
        request.start_state = self.robot_state(arm_positions)
        request.num_planning_attempts = finite_int(
            self.planner.get("planning_attempts"), "planning attempts", minimum=1
        )
        request.allowed_planning_time = finite(
            self.planner.get("allowed_planning_time_sec"), "allowed planning time", positive=True
        )
        request.max_velocity_scaling_factor = finite(
            self.planner.get("velocity_scaling"), "velocity scaling", positive=True
        )
        constraints = Constraints()
        region = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [finite(self.planner.get("position_tolerance_m"), "position tolerance", positive=True)]
        region.primitives = [primitive]
        region.primitive_poses = [copy.deepcopy(target.pose)]
        position = PositionConstraint()
        position.header.frame_id = self.planning_frame
        position.link_name = self.tcp_link
        position.constraint_region = region
        position.weight = 1.0
        orientation = OrientationConstraint()
        orientation.header.frame_id = self.planning_frame
        orientation.link_name = self.tcp_link
        orientation.orientation = copy.deepcopy(target.pose.orientation)
        tolerance = finite(self.planner.get("orientation_tolerance_rad"), "orientation tolerance", positive=True)
        orientation.absolute_x_axis_tolerance = tolerance
        orientation.absolute_y_axis_tolerance = tolerance
        orientation.absolute_z_axis_tolerance = tolerance
        orientation.parameterization = OrientationConstraint.ROTATION_VECTOR
        orientation.weight = 1.0
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        request.goal_constraints = [constraints]
        return request

    @staticmethod
    def error_details(code: MoveItErrorCodes) -> dict[str, Any]:
        return {"value": int(code.val), "message": str(code.message), "source": str(code.source)}

    @staticmethod
    def set_time(point: Any, seconds: float) -> None:
        if not math.isfinite(seconds) or seconds < 0.0:
            raise PickPlaceError("planned waypoint received invalid duration")
        sec = int(math.floor(seconds))
        nanosec = int(round((seconds - sec) * 1_000_000_000.0))
        if nanosec == 1_000_000_000:
            sec += 1
            nanosec = 0
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

    def retime(self, trajectory: Any, label: str) -> tuple[Any, dict[str, Any]]:
        result = copy.deepcopy(trajectory)
        joint_trajectory = result.joint_trajectory
        if tuple(joint_trajectory.joint_names) != self.arm_joints:
            raise PickPlaceError(
                f"{label} trajectory did not use the bounded six-axis controller order"
            )
        source = list(joint_trajectory.points)
        if len(source) < 2:
            raise PickPlaceError(f"{label} plan has fewer than two waypoints")
        cleaned: list[Any] = []
        previous: list[float] | None = None
        for index, point in enumerate(source):
            positions = [finite(value, f"{label} point {index}") for value in point.positions]
            if len(positions) != len(self.arm_joints):
                raise PickPlaceError(f"{label} point {index} does not match arm joint count")
            if previous is not None and max(abs(a - b) for a, b in zip(positions, previous)) <= 1e-12:
                continue
            copied = copy.deepcopy(point)
            copied.positions = positions
            copied.velocities = []
            copied.accelerations = []
            copied.effort = []
            cleaned.append(copied)
            previous = positions
        if len(cleaned) < 2:
            raise PickPlaceError(f"{label} plan contains no measurable motion")
        velocity_scale = finite(self.planner.get("velocity_scaling"), "velocity scaling", positive=True)
        safety = finite(self.planner.get("retime_safety_factor"), "retime safety factor", positive=True)
        minimum = finite(self.planner.get("minimum_segment_duration_sec"), "minimum duration", positive=True)
        cumulative = 0.0
        segment_durations: list[float] = []
        self.set_time(cleaned[0], 0.0)
        for index in range(1, len(cleaned)):
            prior = cleaned[index - 1]
            current = cleaned[index]
            required = max(
                abs(float(current.positions[joint]) - float(prior.positions[joint]))
                / (self.velocity_limits[name] * velocity_scale)
                for joint, name in enumerate(self.arm_joints)
            )
            duration = max(minimum, required * safety)
            cumulative += duration
            self.set_time(current, cumulative)
            segment_durations.append(duration)
        joint_trajectory.points = cleaned
        return result, {
            "method": "velocity_only_conservative_retime",
            "velocity_scaling": velocity_scale,
            "safety_factor": safety,
            "minimum_segment_duration_sec": minimum,
            "segment_durations_sec": segment_durations,
            "duration_sec": cumulative,
            "acceleration_limits_used": False,
        }

    def execute(self, trajectory: Any, label: str) -> dict[str, Any]:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        goal.controller_names = ["joint_trajectory_controller"]
        sent = self.execute_client.send_goal_async(goal)
        self._spin_until(
            sent.done,
            f"MoveIt accepts {label}",
            finite(self.planner.get("service_call_timeout_sec"), "service call timeout", positive=True),
        )
        try:
            handle = sent.result()
        except Exception as exc:  # noqa: BLE001
            raise PickPlaceError(f"could not send {label} to MoveIt: {exc}") from exc
        if handle is None or not handle.accepted:
            raise PickPlaceError(f"MoveIt rejected execution of {label}")
        completed = handle.get_result_async()
        self._spin_until(
            completed.done,
            f"MoveIt executes {label}",
            finite(self.planner.get("action_result_timeout_sec"), "action timeout", positive=True),
        )
        result = completed.result()
        if result is None:
            raise PickPlaceError(f"MoveIt produced no execution result for {label}")
        details = self.error_details(result.result.error_code)
        details["action_status"] = int(result.status)
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code.val != SUCCESS:
            raise PickPlaceError(f"MoveIt execution failed for {label}: {details}")
        return details

    def observe_endpoint(self, expected: dict[str, float], label: str) -> dict[str, float]:
        samples_needed = finite_int(self.planner.get("endpoint_samples"), "endpoint samples", minimum=1)
        tolerance = finite(self.planner.get("observed_joint_tolerance_rad"), "endpoint tolerance", positive=True)
        initial = self.latest_joint_sequence
        matching = 0
        observed: dict[str, float] | None = None
        end = self._bounded_deadline(
            finite(self.planner.get("endpoint_observation_timeout_sec"), "endpoint timeout", positive=True),
            f"observed endpoint for {label}",
        )
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.10, max(0.0, end - time.monotonic())))
            if self.latest_joint_sequence <= initial or self.latest_joint_positions is None:
                continue
            initial = self.latest_joint_sequence
            observed = dict(self.latest_joint_positions)
            if max(abs(observed[name] - expected[name]) for name in self.arm_joints) <= tolerance:
                matching += 1
                if matching >= samples_needed:
                    return observed
            else:
                matching = 0
        raise PickPlaceError(f"{label} endpoint was not observed through joint states: {observed}")

    def plan_pose(
        self,
        label: str,
        target: PoseStamped,
        start_positions: dict[str, float] | None = None,
    ) -> dict[str, Any]:
        """Create one collision-aware, bounded-chain arm plan without executing it."""

        static_scene_ids = self.require_static_obstacles_in_scene(label)
        if start_positions is None:
            start = self.current_positions()
        else:
            requested_start = mapping(start_positions, f"{label} probe start")
            start = {
                name: finite(
                    requested_start.get(name), f"{label} probe start {name}"
                )
                for name in self.arm_joints
            }
        request = self.build_pose_request({name: start[name] for name in self.arm_joints}, target)
        response = self.call_service(self.plan_client, self._motion_plan_service(request), f"plan {label}")
        plan = response.motion_plan_response
        plan_details = self.error_details(plan.error_code)
        if plan.error_code.val != SUCCESS:
            raise PickPlaceError(f"collision-aware MoveIt plan failed for {label}: {plan_details}")
        retimed, retiming = self.retime(plan.trajectory, label)
        points = retimed.joint_trajectory.points
        endpoint = {
            name: float(points[-1].positions[index]) for index, name in enumerate(self.arm_joints)
        }
        return {
            "trajectory": retimed,
            "endpoint": endpoint,
            "target": {
                "frame_id": target.header.frame_id,
                "position_m": [target.pose.position.x, target.pose.position.y, target.pose.position.z],
                "orientation_xyzw": [
                    target.pose.orientation.x,
                    target.pose.orientation.y,
                    target.pose.orientation.z,
                    target.pose.orientation.w,
                ],
            },
            "planning": {**plan_details, "waypoint_count": len(points), "retiming": retiming},
            "planning_scene_static_obstacle_ids": static_scene_ids,
        }

    def probe_pose(
        self,
        label: str,
        target: PoseStamped,
        start_positions: dict[str, float] | None = None,
    ) -> dict[str, Any]:
        """Record a non-executing MoveIt acceptance probe for a side-grasp candidate."""

        planned = self.plan_pose(label, target, start_positions)
        return {
            "collision_aware": True,
            "target": planned["target"],
            "planning": planned["planning"],
            "plan_endpoint": planned["endpoint"],
            "planning_scene_static_obstacle_ids": planned["planning_scene_static_obstacle_ids"],
        }

    def move_pose(self, label: str, target: PoseStamped) -> dict[str, Any]:
        planned = self.plan_pose(label, target)
        execution = self.execute(planned["trajectory"], label)
        endpoint = mapping(planned["endpoint"], f"{label} plan endpoint")
        observed = self.observe_endpoint(endpoint, label)
        record = {
            "label": label,
            "collision_aware": True,
            "target": planned["target"],
            "planning": planned["planning"],
            "execution": execution,
            "observed_endpoint": {name: observed[name] for name in self.arm_joints},
            "planning_scene_static_obstacle_ids": planned["planning_scene_static_obstacle_ids"],
            **self.stage_time(),
        }
        self.motion_records.append(record)
        return record

    @staticmethod
    def _motion_plan_service(request: MotionPlanRequest) -> GetMotionPlan.Request:
        service = GetMotionPlan.Request()
        service.motion_plan_request = request
        return service

    def current_tcp(self, at_ns: int | None = None) -> dict[str, float]:
        lookup_time = Time() if at_ns is None else Time(nanoseconds=at_ns)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame,
                self.tcp_link,
                lookup_time,
                timeout=RosDuration(seconds=0.4),
            )
        except TransformException as exc:
            raise PickPlaceError(f"cannot observe {self.tcp_link} TF: {exc}") from exc
        translation = transform.transform.translation
        values = {"x": float(translation.x), "y": float(translation.y), "z": float(translation.z)}
        if not all(math.isfinite(value) for value in values.values()):
            raise PickPlaceError("TCP TF contains a non-finite translation")
        values["stamp_ns"] = stamp_ns(transform.header)
        return values

    def wait_fresh_vision(
        self,
        class_id: str,
        after_ns: int,
        label: str,
        *,
        guard_only: bool = False,
        transport_only: bool = False,
    ) -> VisionPose:
        result: VisionPose | None = None
        if guard_only and transport_only:
            raise PickPlaceError("vision stream selection cannot request both guard and transport cameras")
        if guard_only:
            observations = self.latest_guard_vision
            stream_label = "guard RGB-D"
        elif transport_only:
            observations = self.latest_transport_vision
            stream_label = "transport RGB-D"
        else:
            observations = self.latest_vision
            stream_label = "primary RGB-D"

        def available() -> bool:
            nonlocal result
            candidate = observations.get(class_id)
            if candidate is None or candidate.stamp_ns <= after_ns:
                return False
            result = candidate
            return True

        self._spin_until(
            available,
            f"fresh {stream_label} vision {class_id} for {label}",
            finite(self.motion.get("max_visual_tracking_gap_sec"), "visual tracking gap", positive=True),
        )
        assert result is not None
        return result

    def wait_fresh_static_obstacles(
        self, after_ns: int, label: str
    ) -> dict[str, VisionPose]:
        """Require both marker poses from one fresh RGB-D image epoch."""

        result: dict[str, VisionPose] | None = None

        def available() -> bool:
            nonlocal result
            observed = self.latest_static_obstacles
            if set(observed) != set(self.static_marker_classes):
                return False
            stamps = {item.stamp_ns for item in observed.values()}
            if len(stamps) != 1 or next(iter(stamps)) <= after_ns:
                return False
            result = dict(observed)
            return True

        self._spin_until(
            available,
            f"fresh RGB-D static-obstacle poses for {label}",
            finite(
                self.motion.get("max_visual_tracking_gap_sec"),
                "visual tracking gap",
                positive=True,
            ),
        )
        assert result is not None
        return result

    def vision_tcp_sample(
        self,
        class_id: str,
        after_ns: int,
        label: str,
        *,
        guard_only: bool = False,
        transport_only: bool = False,
    ) -> dict[str, Any]:
        visual = self.wait_fresh_vision(
            class_id,
            after_ns,
            label,
            guard_only=guard_only,
            transport_only=transport_only,
        )
        # Pair the RGB-D object pose with the TCP transform from the exact
        # same simulation timestamp. A current transform would be invalid for
        # an observation taken while the arm was still moving.
        tcp = self.current_tcp(visual.stamp_ns)
        relative = {axis: getattr(visual, axis) - tcp[axis] for axis in ("x", "y", "z")}
        return {
            "vision": visual.as_dict(),
            "vision_stream": visual.source_stream,
            "tcp": {"position_m": [tcp["x"], tcp["y"], tcp["z"]], "tf_stamp_ns": tcp["stamp_ns"]},
            "green_minus_tcp_m": [relative["x"], relative["y"], relative["z"]],
            "tf_alignment": "tcp_lookup_at_vision_source_stamp",
            "sampled_at": self.stage_time(),
        }

    def fresh_proximate_vision_tcp_sample(
        self,
        class_id: str,
        after_ns: int,
        label: str,
        maximum_distance: float,
        minimum_distance: float = 0.0,
    ) -> dict[str, Any]:
        """Return a time-aligned visual/TCP sample from a proximity-valid camera.

        The P5 cameras have complementary views, so one physical grasp is
        never accepted merely because a particular view is clear. Every stream
        receives the same action-start lower timestamp and must itself report
        the target close to its timestamp-aligned TCP before this method
        returns.
        """

        choices = (
            # The P5-only nadir observer is explicitly preflighted and keeps
            # the ball visible above the finger rails during lift/transport.
            ("phase5_transport_rgbd", {"transport_only": True}),
            ("phase5_guard_rgbd", {"guard_only": True}),
            ("primary_rgbd", {}),
        )
        rejections: list[str] = []
        for stream, options in choices:
            try:
                sample = self.vision_tcp_sample(class_id, after_ns, f"{label} {stream}", **options)
            except PickPlaceError as exc:
                rejections.append(f"{stream} unavailable: {exc}")
                continue
            distance = math.sqrt(sum(value * value for value in sample["green_minus_tcp_m"]))
            if minimum_distance <= distance <= maximum_distance:
                sample["stream_selection"] = stream
                sample["tcp_distance_m"] = distance
                return sample
            rejections.append(f"{stream} TCP distance {distance:.4f} m")
        raise PickPlaceError(
            f"no fresh proximate RGB-D observation for {label}: {'; '.join(rejections)}"
        )

    def observe_gripper(self, target: float, label: str) -> dict[str, Any]:
        tolerance = finite(self.gripper.get("observed_position_tolerance_m"), "gripper tolerance", positive=True)
        previous = self.latest_joint_sequence
        matching = 0
        samples_needed = 3
        end = self._bounded_deadline(
            finite(self.gripper.get("action_timeout_sec"), "gripper action timeout", positive=True),
            f"gripper observation for {label}",
        )
        observed = math.nan
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.10, max(0.0, end - time.monotonic())))
            if self.latest_joint_sequence <= previous or self.latest_joint_positions is None:
                continue
            previous = self.latest_joint_sequence
            observed = self.latest_joint_positions[self.gripper_joint]
            if abs(observed - target) <= tolerance:
                matching += 1
                if matching >= samples_needed:
                    return {"target_position_m": target, "observed_position_m": observed, **self.stage_time()}
            else:
                matching = 0
        raise PickPlaceError(f"gripper {label} was not observed at target {target}: {observed}")

    def observe_gripper_contact_close(self, target: float, label: str) -> dict[str, Any]:
        """Observe a bounded physical close without inferring force or grasp."""

        tolerance = finite(self.gripper.get("observed_position_tolerance_m"), "gripper tolerance", positive=True)
        minimum = finite(
            self.gripper.get("minimum_contact_position_m"),
            "minimum contact close position",
            positive=True,
        )
        if minimum >= target:
            raise PickPlaceError("minimum contact close position must be below the pinch envelope")
        previous = self.latest_joint_sequence
        matching = 0
        samples_needed = 3
        end = self._bounded_deadline(
            finite(self.gripper.get("action_timeout_sec"), "gripper action timeout", positive=True),
            f"gripper contact-close observation for {label}",
        )
        observed = math.nan
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.10, max(0.0, end - time.monotonic())))
            if self.latest_joint_sequence <= previous or self.latest_joint_positions is None:
                continue
            previous = self.latest_joint_sequence
            observed = self.latest_joint_positions[self.gripper_joint]
            if minimum <= observed <= target + tolerance:
                matching += 1
                if matching >= samples_needed:
                    return {
                        "target_position_m": target,
                        "observed_position_m": observed,
                        "minimum_contact_position_m": minimum,
                        "endpoint_observation_semantics": "bounded_contact_close_joint_state",
                        **self.stage_time(),
                    }
            else:
                matching = 0
        raise PickPlaceError(
            f"gripper {label} was not observed in contact-close range "
            f"[{minimum}, {target + tolerance}]: {observed}"
        )

    def command_gripper(self, label: str, target: float) -> dict[str, Any]:
        goal = ParallelGripperCommand.Goal()
        # Jazzy's parallel-gripper controller accepts exactly one named
        # position. Velocity/effort stay empty: effort is neither commanded
        # nor treated as grasp evidence in this simulation profile.
        goal.command.name = [self.gripper_joint]
        goal.command.position = [target]
        goal.command.velocity = []
        goal.command.effort = []
        sent = self.gripper_client.send_goal_async(goal)
        self._spin_until(
            sent.done,
            f"gripper accepts {label}",
            finite(self.gripper.get("action_timeout_sec"), "gripper action timeout", positive=True),
        )
        handle = sent.result()
        if handle is None or not handle.accepted:
            raise PickPlaceError(f"gripper action was not accepted for {label}")
        completed = handle.get_result_async()
        self._spin_until(
            completed.done,
            f"gripper completes {label}",
            finite(self.gripper.get("action_timeout_sec"), "gripper action timeout", positive=True),
        )
        result = completed.result()
        if result is None:
            raise PickPlaceError(f"gripper action did not succeed for {label}")
        action_result = result.result
        if action_result is None:
            raise PickPlaceError(f"gripper action returned no result payload for {label}")
        state_names = [str(name) for name in action_result.state.name]
        state_positions = [float(value) for value in action_result.state.position]
        if not all(math.isfinite(value) for value in state_positions):
            raise PickPlaceError(f"gripper {label} action result state is malformed")
        contact_close = label == "pinch_envelope_close"
        controller_succeeded = (
            result.status == GoalStatus.STATUS_SUCCEEDED
            and action_result.reached_goal
            and not action_result.stalled
        )
        controller_stalled_contact = (
            result.status == GoalStatus.STATUS_ABORTED
            and action_result.stalled
            and not action_result.reached_goal
        )
        if contact_close:
            # A stalled close is not a force/grasp claim. It is accepted only
            # as a repeated, bounded joint-state contact endpoint and is later
            # independently proved by visual lift and transport retention.
            if not (controller_succeeded or controller_stalled_contact):
                raise PickPlaceError(
                    f"gripper contact-close has no approved terminal state for {label} "
                    f"(status={int(result.status)}, reached_goal={bool(action_result.reached_goal)}, "
                    f"stalled={bool(action_result.stalled)})"
                )
            observation = self.observe_gripper_contact_close(target, label)
        elif not controller_succeeded:
            # A controller terminal status never substitutes for actuator
            # evidence. Sample the real joint endpoint before reporting this
            # bounded-chain failure so runtime evidence distinguishes a false
            # controller report from an unachieved command.
            try:
                observation = self.observe_gripper(target, f"{label} after controller failure")
            except PickPlaceError as exc:
                raise PickPlaceError(
                    f"gripper action did not succeed for {label} "
                    f"(status={int(result.status)}, reached_goal={bool(action_result.reached_goal)}, "
                    f"stalled={bool(action_result.stalled)}); {exc}"
                ) from exc
            raise PickPlaceError(
                f"gripper action did not succeed for {label} "
                f"(status={int(result.status)}, reached_goal={bool(action_result.reached_goal)}, "
                f"stalled={bool(action_result.stalled)}) despite observed endpoint "
                f"{float(observation['observed_position_m']):.5f} m"
            )
        else:
            observation = self.observe_gripper(target, label)
        # The Jazzy parallel-gripper controller legitimately returns an empty
        # Result.state.name while retaining an implementation-specific
        # position array.  Its action status/result state is not grasp
        # evidence; the bounded, repeated /joint_states observation is the
        # required actuator endpoint gate.  A named result must still be
        # self-consistent and identify this controller's joint.
        if state_names:
            if (
                len(state_names) != len(state_positions)
                or self.gripper_joint not in state_names
            ):
                raise PickPlaceError(f"gripper {label} action result state is malformed")
            result_position = state_positions[state_names.index(self.gripper_joint)]
            result_position_source = "action_result_named_joint"
            result_state_semantics = "named_joint_state_informational"
        else:
            result_position = float(observation["observed_position_m"])
            result_position_source = "joint_states_endpoint_observation"
            result_state_semantics = "unnamed_controller_state_informational"
        return {
            "action_status": int(result.status),
            "action_goal_accepted": True,
            "terminal_status_semantics": (
                "controller_succeeded_with_observed_contact_range"
                if contact_close and controller_succeeded
                else "controller_stalled_with_observed_contact_range"
                if contact_close and controller_stalled_contact
                else "controller_succeeded_with_observed_target"
            ),
            "result_state_names": state_names,
            "result_state_positions": state_positions,
            "result_state_semantics": result_state_semantics,
            "result_position_m": result_position,
            "result_position_source": result_position_source,
            "result_effort": "UNTRUSTED_NOT_USED",
            "result_stalled": bool(action_result.stalled),
            "result_reached_goal": bool(action_result.reached_goal),
            "observation": observation,
        }

    def planning_scene_object_ids(self) -> list[str]:
        query = GetPlanningScene.Request()
        # WORLD_OBJECT_GEOMETRY = 16; avoid a version-specific enum import
        # because this is an auditable postcondition listing.
        query.components.components = 16
        listed = self.call_service(
            self.scene_query_client, query, "read Phase-5 planning scene"
        )
        return [str(item.id) for item in listed.scene.world.collision_objects]

    def require_static_obstacles_in_scene(self, label: str) -> list[str]:
        required = {record["object_id"] for record in self.static_obstacle_records}
        if required != {
            f"p5_vision_static_{marker_class}" for marker_class in self.static_marker_classes
        }:
            raise PickPlaceError(
                "MoveIt motion requested before the exact two static obstacle records exist"
            )
        active = set(self.planning_scene_object_ids())
        if not required.issubset(active):
            raise PickPlaceError(
                f"MoveIt scene lost fresh RGB-D static obstacles before {label}: "
                f"missing {sorted(required.difference(active))}"
            )
        return sorted(required)

    def apply_scene(self, collision_objects: list[CollisionObject], attached: list[AttachedCollisionObject] | None = None) -> list[str]:
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = collision_objects
        if attached is not None:
            scene.robot_state.attached_collision_objects = attached
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self.call_service(self.scene_client, request, "apply Phase-5 planning scene")
        if not response.success:
            raise PickPlaceError("MoveIt did not accept the Phase-5 planning-scene update")
        return self.planning_scene_object_ids()

    def make_box(self, object_id: str, center: list[float], dimensions: list[float]) -> CollisionObject:
        collision = CollisionObject()
        collision.id = object_id
        collision.header.frame_id = self.planning_frame
        collision.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = dimensions
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = center
        pose.orientation.w = 1.0
        collision.primitives = [primitive]
        collision.primitive_poses = [pose]
        return collision

    def make_sphere(self, object_id: str, center: list[float], radius: float) -> CollisionObject:
        collision = CollisionObject()
        collision.id = object_id
        collision.header.frame_id = self.planning_frame
        collision.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [radius]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = center
        pose.orientation.w = 1.0
        collision.primitives = [primitive]
        collision.primitive_poses = [pose]
        return collision

    def static_obstacle_collision_objects(
        self, after_ns: int
    ) -> tuple[list[CollisionObject], list[dict[str, Any]]]:
        observed = self.wait_fresh_static_obstacles(after_ns, "initial planning scene")
        top_to_center = float(
            self.static_obstacle_geometry["marker_top_to_box_center_z_offset_m"]
        )
        box_size = list(self.static_obstacle_geometry["collision_box_size_xyz_m"])
        collision_objects: list[CollisionObject] = []
        records: list[dict[str, Any]] = []
        for marker_class in self.static_marker_classes:
            marker = observed[marker_class]
            object_id = f"p5_vision_static_{marker_class}"
            center = [marker.x, marker.y, marker.z - top_to_center]
            collision_objects.append(self.make_box(object_id, center, box_size))
            records.append(
                {
                    "object_id": object_id,
                    "marker": marker.as_dict(),
                    "marker_top_to_box_center_z_offset_m": top_to_center,
                    "collision_box_center_m": center,
                    "collision_box_size_xyz_m": box_size,
                    "source": "fresh_phase5_rgbd_static_obstacle_localizer",
                }
            )
        return collision_objects, records

    def apply_initial_scene(self, static_after_ns: int) -> list[str]:
        table = mapping(self.geometry.get("table"), "table geometry")
        board = mapping(self.geometry.get("drawing_board"), "board geometry")
        objects = [
            self.make_box(
                str(table.get("id")),
                list_of_numbers(table.get("center_xyz_m"), "table center", 3),
                list_of_numbers(table.get("size_xyz_m"), "table size", 3),
            ),
            self.make_box(
                str(board.get("id")),
                list_of_numbers(board.get("center_xyz_m"), "board center", 3),
                list_of_numbers(board.get("size_xyz_m"), "board size", 3),
            ),
        ]
        static_objects, static_records = self.static_obstacle_collision_objects(
            static_after_ns
        )
        objects.extend(static_objects)
        for class_id, observed in self.pre_pick.items():
            if class_id in {"green_ball", "drawing_board"}:
                continue
            if class_id.endswith("ball"):
                objects.append(
                    self.make_sphere(
                        f"p5_vision_{class_id}",
                        [observed.x, observed.y, observed.z],
                        finite(self.geometry.get("ball_radius_m"), "ball radius", positive=True),
                    )
                )
            elif class_id.endswith("cube"):
                size = finite(self.geometry.get("cube_size_m"), "cube size", positive=True)
                objects.append(
                    self.make_box(
                        f"p5_vision_{class_id}",
                        [observed.x, observed.y, observed.z],
                        [size, size, size],
                    )
                )
        applied_ids = self.apply_scene(objects)
        required_static_ids = {record["object_id"] for record in static_records}
        if not required_static_ids.issubset(set(applied_ids)):
            raise PickPlaceError(
                "MoveIt planning scene does not contain every fresh RGB-D static obstacle"
            )
        self.static_obstacle_records = static_records
        return applied_ids

    def attach_green_in_scene(self) -> list[str]:
        attached = AttachedCollisionObject()
        attached.link_name = self.tcp_link
        attached.touch_links = [
            self.tcp_link,
            "gripper_base",
            "two_finger_left_link",
            "two_finger_right_link",
        ]
        attached.object.id = "p5_green_attached"
        attached.object.header.frame_id = self.tcp_link
        attached.object.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [finite(self.geometry.get("ball_radius_m"), "ball radius", positive=True)]
        relative = Pose()
        # The visual guard proves this measured relative offset; attached
        # geometry uses the calibrated nominal TCP-to-pinch-centre offset.
        relative.position.x = -finite(
            self.robot.get("tcp_ahead_of_pinch_center_m"), "TCP offset", positive=True
        )
        relative.orientation.w = 1.0
        attached.object.primitives = [primitive]
        attached.object.primitive_poses = [relative]
        return self.apply_scene([], [attached])

    def detach_green_in_scene(self, placement_pose: VisionPose) -> list[str]:
        remove = AttachedCollisionObject()
        remove.link_name = self.tcp_link
        remove.object.id = "p5_green_attached"
        remove.object.operation = CollisionObject.REMOVE
        green_world = self.make_sphere(
            "p5_vision_green_ball",
            [placement_pose.x, placement_pose.y, placement_pose.z],
            finite(self.geometry.get("ball_radius_m"), "ball radius", positive=True),
        )
        return self.apply_scene([green_world], [remove])

    def remove_world_collision_object(self, object_id: str) -> list[str]:
        """Remove one explicit collision object from the live MoveIt scene."""

        remove = CollisionObject()
        remove.id = object_id
        remove.header.frame_id = self.planning_frame
        remove.operation = CollisionObject.REMOVE
        return self.apply_scene([remove])

    def approach_choice(self, grasp_center: VisionPose) -> dict[str, Any]:
        """Choose the first fully collision-plannable descending side grasp.

        Candidate directions are fixed robot/gripper calibrations. The choice
        itself depends only on the fresh RGB-D PlanningScene and live MoveIt
        plan results: neither seeded geometry nor Gazebo object poses enter
        this decision. All three legs are planned before any arm motion; each
        leg explicitly starts at the previous planned endpoint.
        """

        raw_candidates = self.robot.get("side_grasp_candidates")
        if not isinstance(raw_candidates, list) or not raw_candidates:
            raise PickPlaceError("side_grasp_candidates must be a nonempty configuration list")
        pregrasp_offset = finite(
            self.motion.get("pregrasp_offset_m"), "pregrasp offset", positive=True
        )
        checkpoint_backoff = finite(
            self.motion.get("visual_checkpoint_backoff_m"),
            "visual checkpoint backoff",
            positive=True,
        )
        if checkpoint_backoff >= pregrasp_offset:
            raise PickPlaceError("visual checkpoint backoff must be smaller than the pregrasp offset")
        candidates: list[dict[str, Any]] = []
        candidate_ids: set[str] = set()
        for index, raw_candidate in enumerate(raw_candidates):
            candidate = mapping(raw_candidate, f"side grasp candidate {index}")
            candidate_id = candidate.get("id")
            if not isinstance(candidate_id, str) or not candidate_id:
                raise PickPlaceError(f"side grasp candidate {index} has no nonempty id")
            if candidate_id in candidate_ids:
                raise PickPlaceError(f"duplicate side grasp candidate id: {candidate_id}")
            candidate_ids.add(candidate_id)
            quaternion = list_of_numbers(
                candidate.get("orientation_xyzw"),
                f"side grasp candidate {candidate_id} orientation",
                4,
            )
            direction = quaternion_x_axis(quaternion)
            if direction[2] > -0.10 or abs(direction[2]) > 0.90:
                raise PickPlaceError(
                    f"side grasp candidate {candidate_id} must keep local +X nonvertical and descending"
                )
            grasp_tcp = self.target_tcp(grasp_center, direction)
            pregrasp_tcp = (
                grasp_tcp[0] - direction[0] * pregrasp_offset,
                grasp_tcp[1] - direction[1] * pregrasp_offset,
                grasp_tcp[2] - direction[2] * pregrasp_offset,
            )
            checkpoint_tcp = (
                grasp_tcp[0] - direction[0] * checkpoint_backoff,
                grasp_tcp[1] - direction[1] * checkpoint_backoff,
                grasp_tcp[2] - direction[2] * checkpoint_backoff,
            )
            record: dict[str, Any] = {
                "candidate_id": candidate_id,
                "orientation_xyzw": quaternion,
                "local_x_direction_world": list(direction),
                "grasp_tcp_position_m": list(grasp_tcp),
                "visual_checkpoint_tcp_position_m": list(checkpoint_tcp),
                "pregrasp_tcp_position_m": list(pregrasp_tcp),
            }
            try:
                pregrasp_probe = self.probe_pose(
                    f"side_grasp_{candidate_id}_pregrasp_probe",
                    self.pose_stamped(pregrasp_tcp, quaternion),
                )
                checkpoint_probe = self.probe_pose(
                    f"side_grasp_{candidate_id}_visual_checkpoint_probe",
                    self.pose_stamped(checkpoint_tcp, quaternion),
                    mapping(pregrasp_probe["plan_endpoint"], "pregrasp probe endpoint"),
                )
                grasp_probe = self.probe_pose(
                    f"side_grasp_{candidate_id}_grasp_probe",
                    self.pose_stamped(grasp_tcp, quaternion),
                    mapping(checkpoint_probe["plan_endpoint"], "visual checkpoint probe endpoint"),
                )
            except PickPlaceError as error:
                record.update(
                    {
                        "status": "REJECTED_BY_COLLISION_AWARE_MOVEIT",
                        "reason": str(error),
                    }
                )
                candidates.append(record)
                continue
            record.update(
                {
                    "status": "SELECTED",
                    "pregrasp_probe": pregrasp_probe,
                    "visual_checkpoint_probe": checkpoint_probe,
                    "grasp_probe": grasp_probe,
                }
            )
            candidates.append(record)
            return {
                "candidate_id": candidate_id,
                "orientation_xyzw": quaternion,
                # Candidate choice becomes execution evidence immediately
                # after PLAN_APPROACH. Keep every vector JSON-native so the
                # same strict list validator is used before later motions.
                "direction": list(direction),
                "grasp_tcp": list(grasp_tcp),
                "visual_checkpoint_tcp": list(checkpoint_tcp),
                "pregrasp_tcp": list(pregrasp_tcp),
                "policy": "fresh_rgbd_scene_live_moveit_side_grasp_pair_probe",
                "candidate_probes": candidates,
            }
        reasons = "; ".join(
            f"{item['candidate_id']}: {item.get('reason', 'not accepted')}" for item in candidates
        )
        raise PickPlaceError(
            "no descending side-grasp candidate had a collision-aware MoveIt plan pair: " + reasons
        )

    def target_tcp(self, center: VisionPose, direction: tuple[float, float, float], z_offset: float = 0.0) -> tuple[float, float, float]:
        ahead = finite(self.robot.get("tcp_ahead_of_pinch_center_m"), "TCP offset", positive=True)
        return (
            center.x + direction[0] * ahead,
            center.y + direction[1] * ahead,
            center.z + direction[2] * ahead + z_offset,
        )

    def select_placement(self) -> tuple[VisionPose, dict[str, Any]]:
        red = self.pre_pick["red_ball"]
        green = self.pre_pick["green_ball"]
        target_distance = finite(self.placement.get("target_distance_m"), "placement target distance", positive=True)
        minimum = finite(self.placement.get("minimum_non_overlap_distance_m"), "minimum placement distance", positive=True)
        maximum = finite(self.placement.get("maximum_beside_distance_m"), "maximum placement distance", positive=True)
        if not minimum <= target_distance <= maximum:
            raise PickPlaceError("placement target distance is outside required beside bounds")
        bounds = mapping(self.geometry.get("placement_bounds_xy_m"), "placement bounds")
        bounds_x = list_of_numbers(bounds.get("x"), "placement x bounds", 2)
        bounds_y = list_of_numbers(bounds.get("y"), "placement y bounds", 2)
        clearance = finite(self.placement.get("dynamic_clearance_m"), "placement clearance", positive=True)
        ball_radius = finite(self.geometry.get("ball_radius_m"), "ball radius", positive=True)
        cube_half_diagonal = math.sqrt(2.0) * finite(self.geometry.get("cube_size_m"), "cube size", positive=True) / 2.0
        if len(self.static_obstacle_records) != len(self.static_marker_classes):
            raise PickPlaceError(
                "placement selection requires both fresh RGB-D static obstacle records"
            )
        choices: list[tuple[float, int, VisionPose]] = []
        directions = ((-1.0, 0.0), (0.0, -1.0), (0.0, 1.0), (1.0, 0.0))
        for index, (dx, dy) in enumerate(directions):
            candidate = VisionPose(
                class_id="green_ball",
                detection_id="planned_from_red_vision",
                x=red.x + target_distance * dx,
                y=red.y + target_distance * dy,
                z=finite(mapping(self.geometry.get("table"), "table geometry").get("surface_z_m"), "table surface", positive=True) + ball_radius,
                stamp_ns=red.stamp_ns,
                confidence=1.0,
            )
            if not bounds_x[0] + ball_radius <= candidate.x <= bounds_x[1] - ball_radius:
                continue
            if not bounds_y[0] + ball_radius <= candidate.y <= bounds_y[1] - ball_radius:
                continue
            minimum_clearance = math.inf
            valid = True
            for class_id, observed in self.pre_pick.items():
                if class_id in {"green_ball", "red_ball", "drawing_board"}:
                    continue
                object_radius = cube_half_diagonal if class_id.endswith("cube") else ball_radius
                distance = math.hypot(candidate.x - observed.x, candidate.y - observed.y)
                margin = distance - (ball_radius + object_radius + clearance)
                minimum_clearance = min(minimum_clearance, margin)
                if margin < 0.0:
                    valid = False
                    break
            if valid:
                for obstacle in self.static_obstacle_records:
                    center = list_of_numbers(
                        obstacle.get("collision_box_center_m"),
                        "static obstacle collision centre",
                        3,
                    )
                    size = list_of_numbers(
                        obstacle.get("collision_box_size_xyz_m"),
                        "static obstacle collision size",
                        3,
                    )
                    obstacle_half_diagonal = math.hypot(size[0], size[1]) / 2.0
                    distance = math.hypot(candidate.x - center[0], candidate.y - center[1])
                    margin = distance - (
                        ball_radius + obstacle_half_diagonal + clearance
                    )
                    minimum_clearance = min(minimum_clearance, margin)
                    if margin < 0.0:
                        valid = False
                        break
            if valid:
                choices.append((minimum_clearance, -index, candidate))
        if not choices:
            raise PickPlaceError("no vision-safe non-overlapping placement candidate exists")
        _, _, selected = max(choices, key=lambda entry: (entry[0], entry[1]))
        return selected, {
            "target_distance_m": target_distance,
            "minimum_non_overlap_distance_m": minimum,
            "maximum_beside_distance_m": maximum,
            "selection_source": "pre_pick_vision_only",
            "static_obstacle_source": "fresh_phase5_rgbd_static_obstacle_localizer",
            "candidate_count": len(choices),
            "selected": selected.as_dict(),
        }

    def add_attachment_system(self) -> dict[str, Any]:
        """Add retention after the guard; query model identity only, never pose."""

        model_name = str(self.attachment.get("robot_model_name"))
        listed = subprocess.run(
            ["gz", "model", "-m", model_name],
            check=False,
            text=True,
            capture_output=True,
            timeout=5,
        )
        if listed.returncode != 0:
            raise PickPlaceError(f"cannot resolve spawned robot identity for guarded attachment: {listed.stderr.strip()}")
        model_id: int | None = None
        for line in listed.stdout.splitlines():
            if line.strip().startswith("Model:"):
                pieces = line.split("[")
                if len(pieces) > 1 and "]" in pieces[1]:
                    try:
                        model_id = int(pieces[1].split("]", 1)[0])
                    except ValueError:
                        pass
                break
        if model_id is None or model_id <= 0:
            raise PickPlaceError("Gazebo model identity output did not contain a positive model ID")
        parent = str(self.robot.get("gazebo_attachment_parent_link"))
        child_model = str(self.attachment.get("child_model_name"))
        child_link = str(self.attachment.get("child_link_name"))
        detach_topic = str(self.attachment.get("detach_topic"))
        state_topic = str(self.attachment.get("state_topic"))
        innerxml = (
            f"<parent_link>{parent}</parent_link>"
            f"<child_model>{child_model}</child_model>"
            f"<child_link>{child_link}</child_link>"
            f"<attach_topic>/phase5/grasp/attach</attach_topic>"
            f"<detach_topic>{detach_topic}</detach_topic>"
            f"<output_topic>{state_topic}</output_topic>"
        )
        payload = (
            f"entity {{ id: {model_id} type: MODEL }} plugins {{ "
            f"filename: \"{self.attachment.get('dynamic_plugin_filename')}\" "
            f"name: \"{self.attachment.get('dynamic_plugin_name')}\" "
            f"innerxml: \"{innerxml}\" }}"
        )
        service = str(self.attachment.get("add_system_service"))
        response = subprocess.run(
            [
                "gz",
                "service",
                "-s",
                service,
                "--reqtype",
                "gz.msgs.EntityPlugin_V",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "5000",
                "--req",
                payload,
            ],
            check=False,
            text=True,
            capture_output=True,
            timeout=8,
        )
        if response.returncode != 0 or "data: true" not in response.stdout:
            raise PickPlaceError(f"guarded attachment system add failed: {response.stderr.strip()} {response.stdout.strip()}")
        return {
            "activation": "dynamic_entity_system_add_after_vision_tcp_close_guard",
            "robot_model_name": model_name,
            "robot_model_id": model_id,
            "gazebo_parent_link": parent,
            "child_model": child_model,
            "child_link": child_link,
            "service_stdout": response.stdout.strip(),
            **self.stage_time(),
        }

    def wait_attachment_state(
        self, desired: str, label: str, *, after_index: int = 0
    ) -> dict[str, Any]:
        if after_index < 0:
            raise PickPlaceError("attachment lifecycle index must be non-negative")

        def observed_after_guard() -> bool:
            return any(
                item["state"] == desired for item in self.attachment_states[after_index:]
            )

        self._spin_until(
            observed_after_guard,
            f"attachment state {desired} for {label}",
            finite(self.attachment.get("state_timeout_sec"), "attachment state timeout", positive=True),
        )
        matching = [
            item for item in self.attachment_states[after_index:] if item["state"] == desired
        ]
        if not matching:
            raise PickPlaceError(f"attachment state {desired} lacked a timestamped observation")
        return matching[-1]

    def record_stage(self, stage: str, status: str, details: dict[str, Any]) -> None:
        self.stage_records.append({"stage": stage, "status": status, "details": details, **self.stage_time()})

    def run_stage(self, stage: str, operation: Callable[[], dict[str, Any]]) -> dict[str, Any]:
        self.current_stage = stage
        try:
            details = operation()
        except Exception as exc:  # noqa: BLE001 - convert every terminal fault into evidence
            self.record_stage(stage, "FAIL", {"error": str(exc), "error_type": type(exc).__name__})
            raise
        self.record_stage(stage, "PASS", details)
        return details

    @staticmethod
    def visual_position(sample: dict[str, Any]) -> dict[str, float]:
        position = sample["vision"]["position_m"]
        return {"x": float(position[0]), "y": float(position[1]), "z": float(position[2])}

    def collect_retention(
        self,
        after_ns: int,
        baseline: dict[str, Any] | None,
        label: str,
        *,
        sample_count: int | None = None,
    ) -> list[dict[str, Any]]:
        base_relative = None if baseline is None else baseline["green_minus_tcp_m"]
        maximum_drift = finite(self.motion.get("max_tcp_relative_drift_m"), "maximum tracking drift", positive=True)
        required_samples = (
            finite_int(self.motion.get("minimum_retention_samples"), "retention samples", minimum=1)
            if sample_count is None
            else finite_int(sample_count, f"{label} retention samples", minimum=1)
        )
        samples: list[dict[str, Any]] = []
        newest = after_ns
        for index in range(required_samples):
            if base_relative is None:
                # The grasp guard is intentionally observed during the final
                # approach, before the fingers occlude the ball. Establish the
                # retention baseline from the first post-attachment lift image
                # instead. Its calibrated attached-object distance is bounded
                # separately from the approach checkpoint; all later samples
                # use the unchanged relative-drift bound.
                sample = self.fresh_proximate_vision_tcp_sample(
                    "green_ball",
                    newest,
                    f"{label} post-attachment baseline sample {index + 1}",
                    finite(
                        self.motion.get("retention_sample_max_tcp_distance_m"),
                        "retention sample proximity",
                        positive=True,
                    ),
                )
                base_relative = sample["green_minus_tcp_m"]
                drift = 0.0
                sample["retention_baseline"] = "post_attachment_vision_tcp_sample"
                sample["tcp_relative_drift_m"] = drift
                samples.append(sample)
                newest = int(sample["vision"]["source_stamp_ns"])
                continue
            # The +Y P5 guard camera protects the close/lift view, but an arm
            # can obscure any one camera at a later waypoint. Each fallback
            # independently waits for a new visual sample from this action;
            # neither a cache entry nor simulator pose can satisfy the gate.
            guard_error: str | None = None
            try:
                sample = self.vision_tcp_sample(
                    "green_ball", newest, f"{label} sample {index + 1}", guard_only=True
                )
            except PickPlaceError as exc:
                guard_error = str(exc)
            else:
                relative = sample["green_minus_tcp_m"]
                drift = math.sqrt(
                    sum((float(relative[i]) - float(base_relative[i])) ** 2 for i in range(3))
                )
                if drift <= maximum_drift:
                    sample["stream_selection"] = "phase5_guard_rgbd"
                else:
                    guard_error = f"guard TCP-relative drift {drift:.4f} m"
            if guard_error is not None:
                primary_error: str | None = None
                try:
                    primary_sample = self.vision_tcp_sample(
                        "green_ball", newest, f"{label} primary fallback sample {index + 1}"
                    )
                except PickPlaceError as exc:
                    primary_error = str(exc)
                else:
                    primary_relative = primary_sample["green_minus_tcp_m"]
                    primary_drift = math.sqrt(
                        sum(
                            (float(primary_relative[i]) - float(base_relative[i])) ** 2
                            for i in range(3)
                        )
                    )
                    if primary_drift <= maximum_drift:
                        sample = primary_sample
                        drift = primary_drift
                        sample["stream_selection"] = "primary_rgbd_fallback_after_guard_rejection"
                        primary_error = None
                    else:
                        primary_error = f"primary TCP-relative drift {primary_drift:.4f} m"
                if primary_error is not None:
                    try:
                        transport_sample = self.vision_tcp_sample(
                            "green_ball",
                            newest,
                            f"{label} transport fallback sample {index + 1}",
                            transport_only=True,
                        )
                    except PickPlaceError as exc:
                        raise PickPlaceError(
                            f"object lost during {label}: {guard_error}; {primary_error}; "
                            f"transport RGB-D unavailable: {exc}"
                        ) from exc
                    transport_relative = transport_sample["green_minus_tcp_m"]
                    transport_drift = math.sqrt(
                        sum(
                            (float(transport_relative[i]) - float(base_relative[i])) ** 2
                            for i in range(3)
                        )
                    )
                    if transport_drift > maximum_drift:
                        raise PickPlaceError(
                            f"object lost during {label}: {guard_error}; {primary_error}; "
                            f"transport TCP-relative drift {transport_drift:.4f} m"
                        )
                    sample = transport_sample
                    drift = transport_drift
                    sample["stream_selection"] = "phase5_transport_rgbd_fallback_after_guard_primary_rejection"
            sample["tcp_relative_drift_m"] = drift
            samples.append(sample)
            newest = int(sample["vision"]["source_stamp_ns"])
        if not samples:
            raise PickPlaceError(f"no retained-object vision sample exists for {label}")
        return samples

    def run_episode(self) -> dict[str, Any]:
        self.wait_for_endpoints()
        pre_stamp = self.pre_pick["green_ball"].stamp_ns
        green = self.pre_pick["green_ball"]
        red = self.pre_pick["red_ball"]
        grasp_center = VisionPose(
            class_id=green.class_id,
            detection_id=green.detection_id,
            x=green.x,
            y=green.y,
            z=green.z
            + finite(
                self.robot.get("grasp_center_height_clearance_m"),
                "grasp centre/table clearance",
                positive=True,
            ),
            stamp_ns=green.stamp_ns,
            confidence=green.confidence,
        )
        placement_context: dict[str, Any] = {}
        approach_context: dict[str, Any] = {}

        def observe_pick_and_apply_scene() -> dict[str, Any]:
            scene_ids = self.apply_initial_scene(pre_stamp)
            place_green, placement_details = self.select_placement()
            placement_context["green"] = place_green
            placement_context["details"] = placement_details
            return {
                "vision_only": True,
                "pre_pick_source_stamp_ns": pre_stamp,
                "green": green.as_dict(),
                "red": red.as_dict(),
                "collision_safe_grasp_center": grasp_center.as_dict(),
                "planning_scene_object_ids": scene_ids,
                "static_obstacle_vision": self.static_obstacle_records,
                "ground_truth_input_used": False,
            }

        observe = self.run_stage(
            "OBSERVE_PICK",
            observe_pick_and_apply_scene,
        )
        place_green = placement_context["green"]
        placement_details = mapping(placement_context["details"], "placement details")

        def select_and_move_pregrasp() -> dict[str, Any]:
            choice = self.approach_choice(grasp_center)
            approach_context.update(choice)
            return {
                "approach_selection_policy": choice["policy"],
                "selected_side_grasp_candidate": choice["candidate_id"],
                "side_grasp_candidate_probes": choice["candidate_probes"],
                "planned_pregrasp": self.move_pose(
                    "pregrasp",
                    self.pose_stamped(
                        choice["pregrasp_tcp"],
                        list_of_numbers(choice["orientation_xyzw"], "selected side grasp orientation", 4),
                    ),
                ),
            }

        plan = self.run_stage(
            "PLAN_APPROACH",
            select_and_move_pregrasp,
        )
        quaternion = list_of_numbers(
            approach_context.get("orientation_xyzw"), "selected side grasp orientation", 4
        )
        direction_values = list_of_numbers(
            approach_context.get("direction"), "selected side grasp direction", 3
        )
        direction = (direction_values[0], direction_values[1], direction_values[2])
        grasp_tcp_values = list_of_numbers(
            approach_context.get("grasp_tcp"), "selected side grasp TCP", 3
        )
        grasp_tcp = (grasp_tcp_values[0], grasp_tcp_values[1], grasp_tcp_values[2])
        checkpoint_tcp_values = list_of_numbers(
            approach_context.get("visual_checkpoint_tcp"), "visual checkpoint TCP", 3
        )
        checkpoint_tcp = (
            checkpoint_tcp_values[0], checkpoint_tcp_values[1], checkpoint_tcp_values[2]
        )
        opened = self.run_stage(
            "OPEN",
            lambda: self.command_gripper(
                "open", finite(self.gripper.get("open_position_m"), "open position", positive=True)
            ),
        )
        checkpoint_started = self.sim_time_ns()

        def approach_with_visual_checkpoint() -> dict[str, Any]:
            checkpoint_motion = self.move_pose(
                "visual_checkpoint", self.pose_stamped(checkpoint_tcp, quaternion)
            )
            checkpoint_guard = self.fresh_proximate_vision_tcp_sample(
                "green_ball",
                checkpoint_started,
                "visual checkpoint guard",
                finite(
                    self.motion.get("visual_checkpoint_max_tcp_distance_m"),
                    "visual checkpoint maximum TCP distance",
                    positive=True,
                ),
            )
            approach_context["visual_checkpoint_guard"] = checkpoint_guard
            return {
                "target_object_collision_exemption": "green_ball_target_only",
                "visual_checkpoint": checkpoint_motion,
                "visual_checkpoint_guard": checkpoint_guard,
                "unobserved_final_approach_m": finite(
                    self.motion.get("visual_checkpoint_backoff_m"),
                    "visual checkpoint backoff",
                    positive=True,
                ),
                "planned_grasp": self.move_pose(
                    "grasp_approach", self.pose_stamped(grasp_tcp, quaternion)
                ),
            }

        approached = self.run_stage("APPROACH", approach_with_visual_checkpoint)
        def close_and_attach() -> dict[str, Any]:
            # The visual checkpoint is sampled before the fingers occlude the
            # ball. The bounded final insertion has already completed through
            # MoveIt; CLOSE still requires the actual joint-state endpoint.
            close_guard = mapping(
                approach_context.get("visual_checkpoint_guard"), "visual checkpoint grasp guard"
            )
            close_action = self.command_gripper(
                "pinch_envelope_close",
                finite(
                    self.gripper.get("pinch_envelope_position_m"),
                    "pinch envelope",
                    positive=True,
                ),
            )
            close_position = float(close_action["observation"]["observed_position_m"])
            pinch_target = finite(
                self.gripper.get("pinch_envelope_position_m"),
                "pinch envelope",
                positive=True,
            )
            tolerance = finite(
                self.gripper.get("observed_position_tolerance_m"),
                "gripper tolerance",
                positive=True,
            )
            minimum_contact = finite(
                self.gripper.get("minimum_contact_position_m"),
                "minimum contact close position",
                positive=True,
            )
            if not minimum_contact <= close_position <= pinch_target + tolerance:
                raise PickPlaceError("attachment guard rejected an unobserved contact-close endpoint")
            tcp_distance = float(close_guard["tcp_distance_m"])
            attachment_start = len(self.attachment_states)
            activation = self.add_attachment_system()
            # The plugin can publish its initial attached state before DDS
            # discovery completes. This explicit publish provides a second,
            # timestamped lifecycle transition after the close/proximity gate.
            self.attach_publisher.publish(Empty())
            attached_state = self.wait_attachment_state(
                "attached", "guarded attachment", after_index=attachment_start
            )
            attached_scene_ids = self.attach_green_in_scene()
            attachment_record = {
                "guard": {
                    "fresh_vision_tcp_sample": close_guard,
                    "vision_sample_timing": "visual_checkpoint_tf_aligned_before_observed_close",
                    "tcp_distance_m": tcp_distance,
                    "observed_close_position_m": close_position,
                    "force_or_effort_used": False,
                },
                "activation": activation,
                "attached_state": attached_state,
                "planning_scene_object_ids": attached_scene_ids,
            }
            return {**close_action, "attachment_record": attachment_record}

        closed = self.run_stage("CLOSE", close_and_attach)
        attachment_record = mapping(
            closed.get("attachment_record"), "CLOSE attachment record"
        )
        baseline = mapping(attachment_record.get("guard"), "attachment guard")[
            "fresh_vision_tcp_sample"
        ]

        def lift_and_verify() -> dict[str, Any]:
            lift_tcp = (
                grasp_tcp[0],
                grasp_tcp[1],
                grasp_tcp[2]
                + finite(self.motion.get("lift_target_m"), "lift target", positive=True),
            )
            lift_motion = self.move_pose("lift", self.pose_stamped(lift_tcp, quaternion))
            lift_samples = self.collect_retention(
                finite_int(lift_motion["sim_time_ns"], "lift motion endpoint stamp", minimum=1),
                None,
                "lift",
                sample_count=finite_int(
                    self.motion.get("lift_retention_samples"),
                    "lift retention samples",
                    minimum=1,
                ),
            )
            baseline_position = self.visual_position(baseline)
            lift_delta = (
                max(self.visual_position(sample)["z"] for sample in lift_samples)
                - baseline_position["z"]
            )
            if lift_delta < finite(
                self.motion.get("min_lift_m"), "minimum lift", positive=True
            ):
                raise PickPlaceError(
                    f"green ball did not visually lift enough: {lift_delta:.4f} m"
                )
            return {
                "motion": lift_motion,
                "attachment": attachment_record,
                "vision_retention_samples": lift_samples,
                "lift_delta_m": lift_delta,
                "minimum_lift_m": finite(
                    self.motion.get("min_lift_m"), "minimum lift", positive=True
                ),
            }

        lifted = self.run_stage("LIFT_VERIFY", lift_and_verify)
        lift_samples = lifted["vision_retention_samples"]
        lift_delta = float(lifted["lift_delta_m"])
        retention_baseline = lift_samples[-1]
        transport_tcp = self.target_tcp(
            VisionPose(
                class_id=place_green.class_id,
                detection_id=place_green.detection_id,
                x=place_green.x,
                y=place_green.y,
                z=place_green.z + finite(self.motion.get("lift_target_m"), "lift target", positive=True),
                stamp_ns=place_green.stamp_ns,
                confidence=place_green.confidence,
            ),
            direction,
        )

        def transport_and_verify() -> dict[str, Any]:
            if self._attachment_state != "attached":
                raise PickPlaceError("transport started without a confirmed attached lifecycle state")
            transport_motion = self.move_pose(
                "transport", self.pose_stamped(transport_tcp, quaternion)
            )
            transport_samples = self.collect_retention(
                finite_int(
                    transport_motion["sim_time_ns"], "transport motion endpoint stamp", minimum=1
                ),
                retention_baseline,
                "transport",
            )
            lift_reference = self.visual_position(lift_samples[-1])
            transport_final = self.visual_position(transport_samples[-1])
            transport_distance = math.hypot(
                transport_final["x"] - lift_reference["x"],
                transport_final["y"] - lift_reference["y"],
            )
            if transport_distance < finite(
                self.motion.get("minimum_transport_m"),
                "minimum transport",
                positive=True,
            ):
                raise PickPlaceError(
                    "green ball did not visually transport enough: "
                    f"{transport_distance:.4f} m"
                )
            return {
                "motion": transport_motion,
                "placement_policy": placement_details,
                "vision_retention_samples": transport_samples,
                "transport_planar_distance_m": transport_distance,
                "attachment_state": self._attachment_state,
            }

        transported = self.run_stage("TRANSPORT", transport_and_verify)
        transport_samples = transported["vision_retention_samples"]
        transport_distance = float(transported["transport_planar_distance_m"])
        lower_center = VisionPose(
            class_id=place_green.class_id,
            detection_id=place_green.detection_id,
            x=place_green.x,
            y=place_green.y,
            z=place_green.z + finite(self.motion.get("release_clearance_m"), "release clearance", positive=True),
            stamp_ns=place_green.stamp_ns,
            confidence=place_green.confidence,
        )

        def lower_and_verify() -> dict[str, Any]:
            lower_motion = self.move_pose(
                "lower", self.pose_stamped(self.target_tcp(lower_center, direction), quaternion)
            )
            lower_samples = self.collect_retention(
                finite_int(lower_motion["sim_time_ns"], "lower motion endpoint stamp", minimum=1),
                transport_samples[-1],
                "lower",
                sample_count=finite_int(
                    self.motion.get("lower_retention_samples"),
                    "lower retention samples",
                    minimum=1,
                ),
            )
            return {"motion": lower_motion, "vision_retention_samples": lower_samples}

        lowered = self.run_stage("LOWER", lower_and_verify)
        lower_samples = lowered["vision_retention_samples"]

        def release_and_verify() -> dict[str, Any]:
            release_open = self.command_gripper(
                "release_open",
                finite(self.gripper.get("open_position_m"), "open position", positive=True),
            )
            lifecycle_start = len(self.attachment_states)
            self.detach_publisher.publish(Empty())
            detached_state = self.wait_attachment_state(
                "detached", "release", after_index=lifecycle_start
            )
            scene_after_release = self.detach_green_in_scene(place_green)
            minimum_post_stamp = self.sim_time_ns()
            return {
                "gripper_open": release_open,
                "detached_state": detached_state,
                "planning_scene_object_ids": scene_after_release,
                "post_place_minimum_source_stamp_ns": minimum_post_stamp,
                "support_proof": "PENDING_FRESH_POST_RELEASE_RGBD_EVALUATION",
            }

        released = self.run_stage("RELEASE", release_and_verify)
        release_open = mapping(released.get("gripper_open"), "release gripper action")
        release_started = finite_int(
            released.get("post_place_minimum_source_stamp_ns"),
            "post-place minimum source stamp",
            minimum=1,
        )

        def retreat_after_release() -> dict[str, Any]:
            if self._attachment_state != "detached":
                raise PickPlaceError("retreat started before the detached lifecycle state")
            settle_end = self._bounded_deadline(
                finite(
                    self.motion.get("post_release_settle_sec"),
                    "post-release settle",
                    positive=True,
                ),
                "post-release settling",
            )
            while time.monotonic() < settle_end:
                rclpy.spin_once(
                    self, timeout_sec=min(0.10, max(0.0, settle_end - time.monotonic()))
                )
            # Immediately after detach, the now-physical ball legitimately
            # overlaps the open finger rails. Preserve collision checking for
            # the robot, table and RGB-D static obstacles, but remove only
            # this released target collision proxy for the first contact
            # clearance motion. Its Gazebo physics is already normal after
            # the recorded detach event; the later fresh P4 RGB-D seal—not a
            # planning-scene pose—proves its placed result.
            clearance_scene = self.remove_world_collision_object("p5_vision_green_ball")
            retreat_tcp = tuple(
                list_of_numbers(
                    self.motion.get("post_release_retreat_tcp_xyz_m"),
                    "post-release retreat TCP",
                    3,
                )
            )
            retreat_motion = self.move_pose(
                "retreat", self.pose_stamped(retreat_tcp, quaternion)
            )
            return {
                "motion": retreat_motion,
                "gripper_is_open": release_open["observation"],
                "attachment_state": self._attachment_state,
                "retreat_target_policy": "p5_safe_startup_tcp_for_unobstructed_phase4_post_place_vision",
                "released_target_collision_exemption": {
                    "object_id": "p5_vision_green_ball",
                    "scope": "open_finger_contact_clearance_after_recorded_detach_only",
                    "planning_scene_object_ids": clearance_scene,
                },
            }

        retreated = self.run_stage("RETREAT", retreat_after_release)
        return {
            "schema_version": 1,
            "scope": "SIMULATION_ONLY",
            "seed": self.seed,
            "online_ground_truth_input_used": False,
            "pre_pick_evidence": {
                "source_stamp_ns": pre_stamp,
                "green": green.as_dict(),
                "red": red.as_dict(),
                "sealed_artifact": "external_phase4_vision_evidence",
            },
            "required_stages_executed": list(STAGES),
            "stage_records": self.stage_records,
            "motion_records": self.motion_records,
            "attachment_states": self.attachment_states,
            "attachment": attachment_record,
            "retention": {
                "baseline": baseline,
                "lift_samples": lift_samples,
                "transport_samples": transport_samples,
                "lower_samples": lower_samples,
                "lift_delta_m": lift_delta,
                "transport_planar_distance_m": transport_distance,
            },
            "release": released,
            "post_place": {
                "minimum_source_stamp_ns": release_started,
                "planned_green": place_green.as_dict(),
                "red_pre_pick": red.as_dict(),
                "evaluation_required": "fresh_vision_only_post_place_evidence",
            },
            "status": "PENDING_POST_PLACE_VISION_EVALUATION",
            "finished_at_utc": utc_now(),
        }


def load_config(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise PickPlaceError(f"runtime config is missing: {path}")
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise PickPlaceError(f"cannot parse runtime config: {exc}") from exc
    result = mapping(data, "runtime config")
    if result.get("schema_version") != 1 or result.get("scope") != "SIMULATION_ONLY":
        raise PickPlaceError("runtime config has an unsupported schema/scope")
    return result


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--pre-pick-evidence", required=True, type=Path)
    result.add_argument("--output-root", required=True, type=Path)
    result.add_argument("--seed", required=True, type=int)
    result.add_argument("--timeout", type=float, default=210.0)
    result.add_argument("--config", type=Path)
    return result


def main(args: list[str] | None = None) -> int:
    options = parser().parse_args(args)
    runtime: PickPlaceRuntime | None = None
    evidence: dict[str, Any] | None = None
    output = options.output_root / "execution_evidence.json"
    try:
        if options.seed < 0 or not math.isfinite(options.timeout) or options.timeout <= 0.0:
            raise PickPlaceError("--seed and --timeout are invalid")
        config_path = options.config
        if config_path is None:
            config_path = Path(get_package_share_directory("physical_ai_skills")) / "config" / "verified_pick_place.yaml"
        config = load_config(config_path)
        pre_pick, pre_evidence = load_pre_pick_evidence(options.pre_pick_evidence)
        rclpy.init(args=None)
        runtime = PickPlaceRuntime(config, pre_pick, pre_evidence, options.output_root, options.seed, options.timeout)
        evidence = runtime.run_episode()
        atomic_write_json(output, evidence)
    except (PickPlaceError, OSError, ValueError, subprocess.SubprocessError) as exc:
        failed: dict[str, Any] = {
            "schema_version": 1,
            "scope": "SIMULATION_ONLY",
            "seed": options.seed,
            "online_ground_truth_input_used": False,
            "status": "FAIL",
            "error": str(exc),
            "error_type": type(exc).__name__,
            "failed_at_utc": utc_now(),
        }
        if runtime is not None:
            failed["failed_stage"] = runtime.current_stage
            failed["stage_records"] = runtime.stage_records
            failed["motion_records"] = runtime.motion_records
            failed["attachment_states"] = runtime.attachment_states
        try:
            atomic_write_json(output, failed)
        except OSError:
            pass
        print(f"P5_EPISODE=FAIL: {exc}", file=sys.stderr)
        return 1
    finally:
        if runtime is not None:
            runtime.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    assert evidence is not None
    print("P5_EXECUTION=PASS_PENDING_POST_PLACE_VISION")
    print(f"P5_EXECUTION_EVIDENCE={output}")
    print(f"P5_POST_PLACE_MINIMUM_SOURCE_STAMP_NS={evidence['post_place']['minimum_source_stamp_ns']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
