#!/usr/bin/env python3
"""Execute a runtime-generated drawing through collision-aware MoveIt.

The node consumes a live RGB-D-derived drawing-board center, creates every
curve from a ``DrawingIntent`` in board-local coordinates, maps it through a
named board frame, and requests MoveIt's Cartesian-path service with collision
checking enabled.  It never accepts a joint trajectory as drawing input.
"""

from __future__ import annotations

import argparse
import copy
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import tempfile
import time
from typing import Any, Callable

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
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
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPlanningScene
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
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from .board_frame import BoardFrame
from .drawing_intent import DrawingIntent, DrawingIntentError, parse_drawing_instruction
from .geometry import BoardPoint, generate_local_geometry


SUCCESS = MoveItErrorCodes.SUCCESS


class DrawingRuntimeError(RuntimeError):
    """The bounded drawing episode cannot safely continue."""


@dataclass(frozen=True)
class VisionPose:
    class_id: str
    x: float
    y: float
    z: float
    stamp_ns: int
    confidence: float
    detection_id: str
    frame_id: str = "world"

    def as_dict(self) -> dict[str, Any]:
        return {
            "class_id": self.class_id,
            "position_m": [self.x, self.y, self.z],
            "frame_id": self.frame_id,
            "source_stamp_ns": self.stamp_ns,
            "confidence": self.confidence,
            "detection_id": self.detection_id,
            "source_mode": "VISION",
        }


@dataclass(frozen=True)
class StaticObstacleContract:
    """Vision-only collision geometry shared with the reviewed P5 lab."""

    output_topic: str
    marker_class_ids: tuple[str, str]
    marker_top_to_box_center_z_offset_m: float
    collision_box_size_xyz_m: tuple[float, float, float]
    confidence_floor: float


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def finite(value: Any, label: str, *, positive: bool = False) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise DrawingRuntimeError(f"{label} must be numeric") from exc
    if not math.isfinite(result) or (positive and result <= 0.0):
        raise DrawingRuntimeError(f"{label} must be finite{' and positive' if positive else ''}")
    return result


def integer(value: Any, label: str, *, minimum: int = 0) -> int:
    if isinstance(value, bool):
        raise DrawingRuntimeError(f"{label} must be an integer")
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise DrawingRuntimeError(f"{label} must be an integer") from exc
    if result != value or result < minimum:
        raise DrawingRuntimeError(f"{label} must be an integer >= {minimum}")
    return result


def mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise DrawingRuntimeError(f"{label} must be a mapping")
    return value


def vector(value: Any, label: str, length: int) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise DrawingRuntimeError(f"{label} must contain exactly {length} values")
    return tuple(finite(item, f"{label}[{index}]") for index, item in enumerate(value))


def stamp_ns(header: Any) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
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


def load_static_obstacle_contract() -> StaticObstacleContract:
    """Read the P5 RGB-D marker-to-collision contract without a scene layout."""

    config_path = (
        Path(get_package_share_directory("abb_lab_moveit_config"))
        / "config"
        / "phase5_static_obstacle_vision.yaml"
    )
    if not config_path.is_file():
        raise DrawingRuntimeError("Phase-5 static-obstacle vision contract is missing")
    try:
        document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise DrawingRuntimeError("cannot parse Phase-5 static-obstacle vision contract") from exc
    geometry = mapping(
        mapping(document, "Phase-5 static-obstacle vision contract").get(
            "phase5_static_obstacle_geometry"
        ),
        "Phase-5 static-obstacle geometry",
    )
    marker_class_ids = geometry.get("expected_marker_class_ids")
    if (
        not isinstance(marker_class_ids, list)
        or len(marker_class_ids) != 2
        or any(not isinstance(item, str) or not item for item in marker_class_ids)
        or len(set(marker_class_ids)) != 2
    ):
        raise DrawingRuntimeError("Phase-5 static-obstacle contract must contain two marker classes")
    output_topic = str(geometry.get("output_topic", ""))
    if output_topic != "/phase5/perception/static_obstacle_poses":
        raise DrawingRuntimeError("Phase-5 static-obstacle contract has an unexpected topic")
    return StaticObstacleContract(
        output_topic=output_topic,
        marker_class_ids=(str(marker_class_ids[0]), str(marker_class_ids[1])),
        marker_top_to_box_center_z_offset_m=finite(
            geometry.get("marker_top_to_box_center_z_offset_m"),
            "static-obstacle top-to-centre offset",
            positive=True,
        ),
        collision_box_size_xyz_m=vector(
            geometry.get("collision_box_size_xyz_m"),
            "static-obstacle collision box size",
            3,
        ),
        confidence_floor=finite(
            geometry.get("confidence_floor"),
            "static-obstacle confidence floor",
            positive=True,
        ),
    )


class DrawingRuntime(Node):
    """One bounded, evidence-producing contactless simulated drawing episode."""

    def __init__(
        self,
        config: dict[str, Any],
        intent: DrawingIntent,
        output_root: Path,
        timeout_sec: float,
    ) -> None:
        super().__init__("phase6_drawing_runtime", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.config = config
        self.intent = intent
        self.output_root = output_root
        self.deadline = time.monotonic() + finite(timeout_sec, "episode timeout", positive=True)
        self.robot = mapping(config.get("robot"), "robot config")
        self.planner = mapping(config.get("planner"), "planner config")
        self.board_config = mapping(config.get("board"), "board config")
        self.scene_config = mapping(config.get("scene"), "scene config")
        self.static_obstacle_contract = load_static_obstacle_contract()
        self.arm_joints = tuple(self.robot.get("arm_joint_names", ()))
        if self.arm_joints != tuple(f"joint_{index}" for index in range(1, 7)):
            raise DrawingRuntimeError("drawing runtime requires the bounded six-axis ABB arm")
        self.gripper_joint = str(self.robot.get("gripper_joint", ""))
        self.planning_frame = str(self.robot.get("planning_frame", ""))
        self.tcp_link = str(self.robot.get("tcp_link", ""))
        if self.planning_frame != "world" or self.tcp_link != "drawing_tip" or not self.gripper_joint:
            raise DrawingRuntimeError("drawing runtime requires the collision-bearing Phase-6 drawing_tip and bounded gripper joint")
        self.velocity_limits = {
            name: finite(mapping(self.robot.get("velocity_limits_rad_per_sec"), "velocity limits").get(name), f"velocity limit {name}", positive=True)
            for name in self.arm_joints
        }
        configured_position_limits = mapping(
            self.robot.get("joint_position_limits_rad"),
            "joint position limits",
        )
        self.position_limits: dict[str, tuple[float, float]] = {}
        for name in self.arm_joints:
            lower, upper = vector(
                configured_position_limits.get(name),
                f"position limits {name}",
                2,
            )
            if lower >= upper:
                raise DrawingRuntimeError(f"position limits for {name} are invalid")
            self.position_limits[name] = (lower, upper)
        self.plan_client: Client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.cartesian_client: Client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.scene_client: Client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.scene_query_client: Client = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_joints: dict[str, float] | None = None
        self.joint_sequence = 0
        self.latest_vision: dict[str, VisionPose] = {}
        self.vision_sequence = 0
        self.latest_static_obstacles: dict[str, VisionPose] = {}
        self.static_obstacle_sequence = 0
        self.trace: list[dict[str, Any]] = []
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, qos_profile_sensor_data)
        self.create_subscription(ObjectPoseArray, "/perception/world_object_poses", self._on_vision, qos_profile_sensor_data)
        self.create_subscription(
            ObjectPoseArray,
            self.static_obstacle_contract.output_topic,
            self._on_static_obstacles,
            qos_profile_sensor_data,
        )

    def remaining(self) -> float:
        return self.deadline - time.monotonic()

    def _bounded_deadline(self, requested: float, label: str) -> float:
        duration = finite(requested, f"{label} timeout", positive=True)
        if self.remaining() <= 0.0:
            raise DrawingRuntimeError(f"global timeout expired before {label}")
        return time.monotonic() + min(duration, self.remaining())

    def _spin_until(self, predicate: Callable[[], bool], label: str, requested: float) -> None:
        end = self._bounded_deadline(requested, label)
        while time.monotonic() < end:
            if predicate():
                return
            rclpy.spin_once(self, timeout_sec=min(0.10, max(0.0, end - time.monotonic())))
        if not predicate():
            raise DrawingRuntimeError(f"timed out waiting for {label}")

    def _on_joint_state(self, message: JointState) -> None:
        positions = {str(name): float(position) for name, position in zip(message.name, message.position)}
        required = (*self.arm_joints, self.gripper_joint)
        if any(name not in positions or not math.isfinite(positions[name]) for name in required):
            return
        self.latest_joints = {name: positions[name] for name in required}
        self.joint_sequence += 1

    def _on_vision(self, message: ObjectPoseArray) -> None:
        observed: dict[str, VisionPose] = {}
        for item in message.poses:
            values = (item.pose.pose.position.x, item.pose.pose.position.y, item.pose.pose.position.z)
            source_stamp = stamp_ns(item.source_header)
            if (
                item.source_mode != item.SOURCE_VISION
                or item.header.frame_id != self.planning_frame
                or source_stamp <= 0
                or not item.class_id
                or not item.detection_id
                or float(item.confidence) < 0.60
                or not all(math.isfinite(float(value)) for value in values)
            ):
                continue
            observed[str(item.class_id)] = VisionPose(
                class_id=str(item.class_id),
                x=float(values[0]),
                y=float(values[1]),
                z=float(values[2]),
                stamp_ns=source_stamp,
                confidence=float(item.confidence),
                detection_id=str(item.detection_id),
                frame_id=str(item.header.frame_id),
            )
        if observed:
            self.latest_vision.update(observed)
            self.vision_sequence += 1

    def _on_static_obstacles(self, message: ObjectPoseArray) -> None:
        """Accept one coherent RGB-D epoch for both collision fiducials."""

        expected = set(self.static_obstacle_contract.marker_class_ids)
        observed: dict[str, VisionPose] = {}
        for item in message.poses:
            class_id = str(item.class_id)
            if class_id not in expected:
                continue
            values = (item.pose.pose.position.x, item.pose.pose.position.y, item.pose.pose.position.z)
            source_stamp = stamp_ns(item.source_header)
            if (
                class_id in observed
                or item.source_mode != item.SOURCE_VISION
                or item.header.frame_id != self.planning_frame
                or source_stamp <= 0
                or not item.detection_id
                or float(item.confidence) < self.static_obstacle_contract.confidence_floor
                or not all(math.isfinite(float(value)) for value in values)
            ):
                return
            observed[class_id] = VisionPose(
                class_id=class_id,
                x=float(values[0]),
                y=float(values[1]),
                z=float(values[2]),
                stamp_ns=source_stamp,
                confidence=float(item.confidence),
                detection_id=str(item.detection_id),
                frame_id=str(item.header.frame_id),
            )
        if set(observed) != expected or len({item.stamp_ns for item in observed.values()}) != 1:
            return
        self.latest_static_obstacles = observed
        self.static_obstacle_sequence += 1

    def wait_for_endpoints(self) -> None:
        timeout = finite(self.planner.get("service_wait_timeout_sec"), "service wait", positive=True)
        for client, label in (
            (self.plan_client, "/plan_kinematic_path"),
            (self.cartesian_client, "/compute_cartesian_path"),
            (self.scene_client, "/apply_planning_scene"),
            (self.scene_query_client, "/get_planning_scene"),
        ):
            end = self._bounded_deadline(timeout, label)
            while time.monotonic() < end:
                if client.wait_for_service(timeout_sec=min(0.2, max(0.0, end - time.monotonic()))):
                    break
            else:
                raise DrawingRuntimeError(f"MoveIt service is unavailable: {label}")
        end = self._bounded_deadline(timeout, "/execute_trajectory")
        while time.monotonic() < end:
            if self.execute_client.wait_for_server(timeout_sec=min(0.2, max(0.0, end - time.monotonic()))):
                break
        else:
            raise DrawingRuntimeError("MoveIt action is unavailable: /execute_trajectory")
        self._spin_until(lambda: self.latest_joints is not None, "joint-state observation", timeout)

    def call_service(self, client: Client, request: Any, label: str) -> Any:
        future = client.call_async(request)
        self._spin_until(
            future.done,
            f"service result for {label}",
            finite(self.planner.get("service_call_timeout_sec"), "service call", positive=True),
        )
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            raise DrawingRuntimeError(f"service failure for {label}: {exc}") from exc
        if result is None:
            raise DrawingRuntimeError(f"service returned no result for {label}")
        return result

    def current_positions(self) -> dict[str, float]:
        self._spin_until(lambda: self.latest_joints is not None, "current joint state", 4.0)
        assert self.latest_joints is not None
        return dict(self.latest_joints)

    def robot_state(self, positions: dict[str, float] | None = None) -> RobotState:
        joints = self.current_positions() if positions is None else positions
        result = RobotState()
        result.is_diff = False
        result.joint_state.name = [*self.arm_joints, self.gripper_joint]
        result.joint_state.position = [*[float(joints[name]) for name in self.arm_joints], float(joints[self.gripper_joint])]
        return result

    def pose(self, position: tuple[float, float, float], orientation: tuple[float, float, float, float]) -> Pose:
        result = Pose()
        result.position.x, result.position.y, result.position.z = position
        result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = orientation
        return result

    def pose_stamped(self, position: tuple[float, float, float], orientation: tuple[float, float, float, float]) -> PoseStamped:
        result = PoseStamped()
        result.header.frame_id = self.planning_frame
        result.pose = self.pose(position, orientation)
        return result

    def build_pose_request(self, target: PoseStamped) -> MotionPlanRequest:
        request = MotionPlanRequest()
        request.group_name = str(self.robot.get("group_name"))
        request.pipeline_id = str(self.planner.get("pipeline_id"))
        request.planner_id = str(self.planner.get("planner_id"))
        request.start_state = self.robot_state()
        request.num_planning_attempts = integer(self.planner.get("planning_attempts"), "planning attempts", minimum=1)
        request.allowed_planning_time = finite(self.planner.get("allowed_planning_time_sec"), "planning time", positive=True)
        request.max_velocity_scaling_factor = finite(self.planner.get("velocity_scaling"), "velocity scaling", positive=True)
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
    def _set_time(point: Any, seconds: float) -> None:
        if not math.isfinite(seconds) or seconds < 0.0:
            raise DrawingRuntimeError("trajectory has an invalid duration")
        sec = int(math.floor(seconds))
        nanosec = int(round((seconds - sec) * 1_000_000_000.0))
        if nanosec == 1_000_000_000:
            sec += 1
            nanosec = 0
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

    def retime(
        self,
        trajectory: Any,
        label: str,
        *,
        cartesian: bool = False,
        profile: str | None = None,
    ) -> tuple[Any, dict[str, Any]]:
        result = copy.deepcopy(trajectory)
        joint_trajectory = result.joint_trajectory
        if tuple(joint_trajectory.joint_names) != self.arm_joints:
            raise DrawingRuntimeError(f"{label} did not use the exact six-axis controller order")
        source = list(joint_trajectory.points)
        if len(source) < 2:
            raise DrawingRuntimeError(f"{label} has fewer than two trajectory points")
        cleaned, previous = [], None
        for index, point in enumerate(source):
            positions = [finite(item, f"{label} point {index}") for item in point.positions]
            if len(positions) != len(self.arm_joints):
                raise DrawingRuntimeError(f"{label} point {index} has an invalid joint count")
            if previous is not None and max(abs(a - b) for a, b in zip(positions, previous)) <= 1e-12:
                continue
            copied = copy.deepcopy(point)
            copied.positions, copied.velocities, copied.accelerations, copied.effort = positions, [], [], []
            cleaned.append(copied)
            previous = positions
        if len(cleaned) < 2:
            raise DrawingRuntimeError(f"{label} contains no measurable motion")
        selected_profile = profile or ("cartesian" if cartesian else "pose")
        if selected_profile not in {"pose", "cartesian", "entry"}:
            raise DrawingRuntimeError(f"{label} requested an unknown retiming profile")
        if selected_profile == "entry" and not cartesian:
            raise DrawingRuntimeError(f"{label} entry profile requires a Cartesian trajectory")
        prefix = "" if selected_profile == "pose" else f"{selected_profile}_"
        velocity_scale = finite(
            self.planner.get(f"{prefix}velocity_scaling", self.planner.get("velocity_scaling")),
            "velocity scaling",
            positive=True,
        )
        safety = finite(
            self.planner.get(f"{prefix}retime_safety_factor", self.planner.get("retime_safety_factor")),
            "retime safety factor",
            positive=True,
        )
        minimum = finite(
            self.planner.get(f"{prefix}minimum_segment_duration_sec", self.planner.get("minimum_segment_duration_sec")),
            "minimum duration",
            positive=True,
        )
        elapsed, durations = 0.0, []
        self._set_time(cleaned[0], 0.0)
        for index in range(1, len(cleaned)):
            prior, current = cleaned[index - 1], cleaned[index]
            needed = max(abs(float(current.positions[joint]) - float(prior.positions[joint])) / (self.velocity_limits[name] * velocity_scale) for joint, name in enumerate(self.arm_joints))
            duration = max(minimum, needed * safety)
            elapsed += duration
            self._set_time(current, elapsed)
            durations.append(duration)
        terminal_hold_sec = 0.0
        if cartesian:
            # Keep the collision-checked final joint target flat for a bounded
            # interval before the controller begins its strict goal-time
            # evaluation.  This adds no geometric or joint-space motion.
            terminal_hold_sec = finite(
                self.planner.get(f"{prefix}terminal_hold_sec"),
                "Cartesian terminal hold",
                positive=True,
            )
            terminal_hold = copy.deepcopy(cleaned[-1])
            terminal_hold.velocities, terminal_hold.accelerations, terminal_hold.effort = [], [], []
            elapsed += terminal_hold_sec
            self._set_time(terminal_hold, elapsed)
            cleaned.append(terminal_hold)
        joint_trajectory.points = cleaned
        return result, {
            "method": "velocity_only_conservative_retime",
            "velocity_scaling": velocity_scale,
            "safety_factor": safety,
            "minimum_segment_duration_sec": minimum,
            "profile": selected_profile,
            "segment_durations_sec": durations,
            "duration_sec": elapsed,
            "terminal_hold": {
                "duration_sec": terminal_hold_sec,
                "repeats_collision_checked_endpoint": cartesian,
            },
            "acceleration_limits_used": False,
        }

    def current_tcp_pose(self) -> dict[str, Any] | None:
        try:
            transform = self.tf_buffer.lookup_transform(self.planning_frame, self.tcp_link, Time(), timeout=RosDuration(seconds=0.12))
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        position = (float(translation.x), float(translation.y), float(translation.z))
        orientation = (float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w))
        if not all(math.isfinite(value) for value in (*position, *orientation)):
            return None
        norm = math.sqrt(sum(value * value for value in orientation))
        if norm <= 0.0:
            return None
        return {
            "sim_time_ns": stamp_ns(transform.header),
            "position_m": list(position),
            "orientation_xyzw": [value / norm for value in orientation],
        }

    def _record_tcp(self, phase: str) -> None:
        pose = self.current_tcp_pose()
        if pose is not None:
            self.trace.append({"phase": phase, "wall_time_utc": utc_now(), **pose})

    def observe_tcp_endpoint(
        self,
        expected_position: tuple[float, float, float],
        expected_orientation: tuple[float, float, float, float],
        label: str,
    ) -> dict[str, Any]:
        position_tolerance = finite(
            self.planner.get("entry_tcp_position_tolerance_m"),
            "entry TCP position tolerance",
            positive=True,
        )
        orientation_tolerance = finite(
            self.planner.get("entry_tcp_orientation_tolerance_rad"),
            "entry TCP orientation tolerance",
            positive=True,
        )
        samples_needed = integer(self.planner.get("entry_tcp_samples"), "entry TCP samples", minimum=1)
        expected_norm = math.sqrt(sum(value * value for value in expected_orientation))
        if expected_norm <= 0.0:
            raise DrawingRuntimeError("entry TCP expected orientation is invalid")
        expected_quaternion = tuple(value / expected_norm for value in expected_orientation)
        matching, last_stamp, observed = 0, -1, None
        end = self._bounded_deadline(
            finite(self.planner.get("entry_tcp_observation_timeout_sec"), "entry TCP observation", positive=True),
            f"TCP endpoint {label}",
        )
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.10, max(0.0, end - time.monotonic())))
            candidate = self.current_tcp_pose()
            if candidate is None or candidate["sim_time_ns"] <= last_stamp:
                continue
            last_stamp = int(candidate["sim_time_ns"])
            position_error = math.sqrt(
                sum(
                    (float(candidate["position_m"][index]) - expected_position[index]) ** 2
                    for index in range(3)
                )
            )
            dot = abs(
                sum(
                    float(candidate["orientation_xyzw"][index]) * expected_quaternion[index]
                    for index in range(4)
                )
            )
            orientation_error = 2.0 * math.acos(min(1.0, max(-1.0, dot)))
            observed = {
                **candidate,
                "position_error_m": position_error,
                "orientation_error_rad": orientation_error,
            }
            if position_error <= position_tolerance and orientation_error <= orientation_tolerance:
                matching += 1
                if matching >= samples_needed:
                    return {
                        "expected_position_m": list(expected_position),
                        "expected_orientation_xyzw": list(expected_quaternion),
                        "observed": observed,
                        "position_tolerance_m": position_tolerance,
                        "orientation_tolerance_rad": orientation_tolerance,
                        "consecutive_samples": samples_needed,
                    }
            else:
                matching = 0
        raise DrawingRuntimeError(f"TCP endpoint not observed for {label}: {observed}")

    def execute(self, trajectory: Any, label: str) -> dict[str, Any]:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        goal.controller_names = ["joint_trajectory_controller"]
        sent = self.execute_client.send_goal_async(goal)
        self._spin_until(sent.done, f"MoveIt accepts {label}", finite(self.planner.get("service_call_timeout_sec"), "service call", positive=True))
        try:
            handle = sent.result()
        except Exception as exc:  # noqa: BLE001
            raise DrawingRuntimeError(f"could not send {label} to MoveIt: {exc}") from exc
        if handle is None or not handle.accepted:
            raise DrawingRuntimeError(f"MoveIt rejected {label}")
        completed = handle.get_result_async()
        end = self._bounded_deadline(finite(self.planner.get("action_result_timeout_sec"), "action result", positive=True), f"MoveIt executes {label}")
        next_sample = 0.0
        while time.monotonic() < end and not completed.done():
            rclpy.spin_once(self, timeout_sec=min(0.10, max(0.0, end - time.monotonic())))
            if time.monotonic() >= next_sample:
                self._record_tcp(label)
                next_sample = time.monotonic() + 0.08
        if not completed.done():
            raise DrawingRuntimeError(f"timed out waiting for MoveIt {label}")
        result = completed.result()
        if result is None:
            raise DrawingRuntimeError(f"MoveIt produced no result for {label}")
        details = self.error_details(result.result.error_code)
        details["action_status"] = int(result.status)
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code.val != SUCCESS:
            raise DrawingRuntimeError(f"MoveIt execution failed for {label}: {details}")
        self._record_tcp(label)
        return details

    def observe_endpoint(self, expected: dict[str, float], label: str) -> dict[str, float]:
        tolerance = finite(self.planner.get("observed_joint_tolerance_rad"), "joint endpoint tolerance", positive=True)
        samples_needed = integer(self.planner.get("endpoint_samples"), "endpoint samples", minimum=1)
        initial, matching, observed = self.joint_sequence, 0, None
        end = self._bounded_deadline(finite(self.planner.get("endpoint_observation_timeout_sec"), "endpoint observation", positive=True), f"endpoint {label}")
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.10, max(0.0, end - time.monotonic())))
            if self.latest_joints is None or self.joint_sequence <= initial:
                continue
            initial, observed = self.joint_sequence, dict(self.latest_joints)
            if max(abs(observed[name] - expected[name]) for name in self.arm_joints) <= tolerance:
                matching += 1
                if matching >= samples_needed:
                    return {name: observed[name] for name in self.arm_joints}
            else:
                matching = 0
        raise DrawingRuntimeError(f"joint-state endpoint not observed for {label}: {observed}")

    def planning_scene_ids(self) -> list[str]:
        request = GetPlanningScene.Request()
        request.components.components = 16  # WORLD_OBJECT_GEOMETRY
        result = self.call_service(self.scene_query_client, request, "read planning scene")
        return [str(item.id) for item in result.scene.world.collision_objects]

    def require_static_obstacles_in_scene(self, advertised_ids: list[str], label: str) -> list[str]:
        table = mapping(self.scene_config.get("table"), "table scene geometry")
        required = {
            str(table.get("id")),
            "p6_live_vision_drawing_board",
        }
        required.update(
            f"p6_vision_static_{class_id}"
            for class_id in self.static_obstacle_contract.marker_class_ids
        )
        if not required.issubset(set(advertised_ids)):
            raise DrawingRuntimeError(f"{label} was not given the complete collision scene")
        active = self.planning_scene_ids()
        if not required.issubset(set(active)):
            raise DrawingRuntimeError(
                f"MoveIt scene lost required collision geometry before {label}: "
                f"missing {sorted(required.difference(active))}"
            )
        return active

    def make_box(self, object_id: str, center: tuple[float, float, float], dimensions: tuple[float, float, float]) -> CollisionObject:
        item = CollisionObject()
        item.id, item.header.frame_id, item.operation = object_id, self.planning_frame, CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type, primitive.dimensions = SolidPrimitive.BOX, list(dimensions)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = center
        pose.orientation.w = 1.0
        item.primitives, item.primitive_poses = [primitive], [pose]
        return item

    def make_sphere(self, object_id: str, center: tuple[float, float, float], radius: float) -> CollisionObject:
        item = CollisionObject()
        item.id, item.header.frame_id, item.operation = object_id, self.planning_frame, CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type, primitive.dimensions = SolidPrimitive.SPHERE, [radius]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = center
        pose.orientation.w = 1.0
        item.primitives, item.primitive_poses = [primitive], [pose]
        return item

    def apply_scene(self, objects: list[CollisionObject]) -> list[str]:
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = objects
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self.call_service(self.scene_client, request, "apply Phase-6 planning scene")
        if not response.success:
            raise DrawingRuntimeError("MoveIt refused the Phase-6 planning-scene update")
        return self.planning_scene_ids()

    def wait_board(self) -> VisionPose:
        result: VisionPose | None = None

        def available() -> bool:
            nonlocal result
            candidate = self.latest_vision.get("drawing_board")
            if candidate is None:
                return False
            result = candidate
            return True

        self._spin_until(available, "fresh RGB-D drawing-board observation", finite(self.planner.get("vision_timeout_sec"), "vision wait", positive=True))
        assert result is not None
        return result

    def wait_static_obstacles(self, after_ns: int) -> dict[str, VisionPose]:
        minimum_stamp = integer(after_ns, "static-obstacle freshness stamp", minimum=1)
        result: dict[str, VisionPose] | None = None

        def available() -> bool:
            nonlocal result
            expected = set(self.static_obstacle_contract.marker_class_ids)
            if set(self.latest_static_obstacles) != expected:
                return False
            if any(item.stamp_ns <= minimum_stamp for item in self.latest_static_obstacles.values()):
                return False
            result = dict(self.latest_static_obstacles)
            return True

        self._spin_until(
            available,
            "fresh RGB-D static-obstacle observations",
            finite(self.planner.get("vision_timeout_sec"), "vision wait", positive=True),
        )
        assert result is not None
        return result

    def board_frame(self, observed: VisionPose) -> BoardFrame:
        reference = str(self.board_config.get("observation_reference", ""))
        if reference != "robot_facing_front_surface":
            raise DrawingRuntimeError("drawing-board observation must be its robot-facing front surface")
        normal = vector(self.board_config.get("robot_facing_normal_world"), "board normal", 3)
        front_surface_offset = finite(
            self.board_config.get("front_surface_offset_m"),
            "board front offset",
            positive=True,
        )
        board_size = vector(self.board_config.get("size_xyz_m"), "board size", 3)
        if not math.isclose(front_surface_offset, board_size[0] / 2.0, rel_tol=0.0, abs_tol=1e-6):
            raise DrawingRuntimeError("drawing-board front offset must equal half its collision thickness")
        surface = (observed.x, observed.y, observed.z)
        center = tuple(
            surface[index] - front_surface_offset * normal[index]
            for index in range(3)
        )
        return BoardFrame(
            center_world_m=center,
            normal_world=normal,
            horizontal_world=vector(self.board_config.get("horizontal_axis_world"), "board horizontal axis", 3),
            vertical_world=vector(self.board_config.get("vertical_axis_world"), "board vertical axis", 3),
            front_surface_offset_m=front_surface_offset,
        )

    def static_obstacle_collision_objects(
        self,
        observed: dict[str, VisionPose],
    ) -> tuple[list[CollisionObject], list[dict[str, Any]]]:
        collision_objects: list[CollisionObject] = []
        records: list[dict[str, Any]] = []
        for class_id in self.static_obstacle_contract.marker_class_ids:
            marker = observed.get(class_id)
            if marker is None:
                raise DrawingRuntimeError(f"static-obstacle marker is missing: {class_id}")
            center = (
                marker.x,
                marker.y,
                marker.z - self.static_obstacle_contract.marker_top_to_box_center_z_offset_m,
            )
            object_id = f"p6_vision_static_{class_id}"
            collision_objects.append(
                self.make_box(
                    object_id,
                    center,
                    self.static_obstacle_contract.collision_box_size_xyz_m,
                )
            )
            records.append(
                {
                    "object_id": object_id,
                    "marker": marker.as_dict(),
                    "marker_top_to_box_center_z_offset_m": self.static_obstacle_contract.marker_top_to_box_center_z_offset_m,
                    "collision_box_center_m": list(center),
                    "collision_box_size_xyz_m": list(
                        self.static_obstacle_contract.collision_box_size_xyz_m
                    ),
                    "source": "fresh_phase5_rgbd_static_obstacle_localizer",
                }
            )
        return collision_objects, records

    def apply_drawing_scene(
        self,
        frame: BoardFrame,
        static_obstacles: dict[str, VisionPose],
    ) -> tuple[list[str], list[dict[str, Any]]]:
        table = mapping(self.scene_config.get("table"), "table scene geometry")
        table_center = vector(table.get("center_xyz_m"), "table center", 3)
        table_size = vector(table.get("size_xyz_m"), "table size", 3)
        board_size = vector(self.board_config.get("size_xyz_m"), "board size", 3)
        static_objects, static_records = self.static_obstacle_collision_objects(static_obstacles)
        objects = [
            self.make_box(str(table.get("id")), table_center, table_size),
            self.make_box("p6_live_vision_drawing_board", frame.center_world_m, board_size),
        ]
        objects.extend(static_objects)
        ball_radius = finite(self.scene_config.get("ball_radius_m"), "ball radius", positive=True)
        cube_size = finite(self.scene_config.get("cube_size_m"), "cube size", positive=True)
        for class_id, observed in sorted(self.latest_vision.items()):
            if class_id == "drawing_board":
                continue
            center = (observed.x, observed.y, observed.z)
            if class_id.endswith("ball"):
                objects.append(self.make_sphere(f"p6_live_vision_{class_id}", center, ball_radius))
            elif class_id.endswith("cube"):
                objects.append(self.make_box(f"p6_live_vision_{class_id}", center, (cube_size, cube_size, cube_size)))
        active = self.apply_scene(objects)
        required = {
            str(table.get("id")),
            "p6_live_vision_drawing_board",
            *(record["object_id"] for record in static_records),
        }
        if not required.issubset(active):
            raise DrawingRuntimeError(f"MoveIt scene is missing required collision geometry: {sorted(required.difference(active))}")
        return active, static_records

    def plan_pose(self, label: str, target: PoseStamped, scene_ids: list[str]) -> dict[str, Any]:
        active_scene_ids = self.require_static_obstacles_in_scene(scene_ids, label)
        response = self.call_service(self.plan_client, self._motion_plan_service(self.build_pose_request(target)), f"plan {label}")
        plan = response.motion_plan_response
        details = self.error_details(plan.error_code)
        if plan.error_code.val != SUCCESS:
            raise DrawingRuntimeError(f"collision-aware plan failed for {label}: {details}")
        trajectory, retiming = self.retime(plan.trajectory, label)
        points = trajectory.joint_trajectory.points
        endpoint = {name: float(points[-1].positions[index]) for index, name in enumerate(self.arm_joints)}
        return {
            "trajectory": trajectory,
            "target": {"frame_id": target.header.frame_id, "position_m": [target.pose.position.x, target.pose.position.y, target.pose.position.z], "orientation_xyzw": [target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w]},
            "planning": {**details, "collision_aware": True, "planning_scene_ids": active_scene_ids, "waypoint_count": len(points), "retiming": retiming},
            "endpoint": endpoint,
        }

    @staticmethod
    def _motion_plan_service(request: MotionPlanRequest) -> GetMotionPlan.Request:
        result = GetMotionPlan.Request()
        result.motion_plan_request = request
        return result

    def cartesian_path(
        self,
        label: str,
        waypoints: list[tuple[float, float, float]],
        orientation: tuple[float, float, float, float],
        scene_ids: list[str],
        *,
        start_positions: dict[str, float] | None = None,
        retime_profile: str = "cartesian",
    ) -> dict[str, Any]:
        if retime_profile not in {"cartesian", "entry"}:
            raise DrawingRuntimeError(f"{label} requested an invalid Cartesian retiming profile")
        profile_prefix = f"{retime_profile}_"
        active_scene_ids = self.require_static_obstacles_in_scene(scene_ids, label)
        request = GetCartesianPath.Request()
        request.header.frame_id = self.planning_frame
        request.start_state = self.robot_state(start_positions)
        request.group_name = str(self.robot.get("group_name"))
        request.link_name = self.tcp_link
        request.waypoints = [self.pose(point, orientation) for point in waypoints]
        request.max_step = finite(self.planner.get("cartesian_max_step_m"), "Cartesian max step", positive=True)
        request.jump_threshold = finite(self.planner.get("cartesian_jump_threshold"), "Cartesian jump threshold")
        request.prismatic_jump_threshold = finite(self.planner.get("cartesian_prismatic_jump_threshold"), "Cartesian prismatic jump threshold")
        request.revolute_jump_threshold = finite(self.planner.get("cartesian_revolute_jump_threshold"), "Cartesian revolute jump threshold")
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = finite(
            self.planner.get(f"{profile_prefix}velocity_scaling", self.planner.get("velocity_scaling")),
            "Cartesian velocity scaling",
            positive=True,
        )
        request.max_acceleration_scaling_factor = finite(self.planner.get("cartesian_acceleration_scaling"), "Cartesian acceleration scaling", positive=True)
        response = self.call_service(self.cartesian_client, request, f"Cartesian {label}")
        details = self.error_details(response.error_code)
        fraction = float(response.fraction)
        if response.error_code.val != SUCCESS or fraction < 0.999999:
            raise DrawingRuntimeError(f"collision-aware Cartesian path failed for {label}: fraction={fraction}, {details}")
        trajectory, retiming = self.retime(
            response.solution,
            label,
            cartesian=True,
            profile=retime_profile,
        )
        if retime_profile == "entry":
            maximum = finite(
                self.planner.get("entry_max_duration_sec"),
                "entry maximum duration",
                positive=True,
            )
            if retiming["duration_sec"] > maximum:
                raise DrawingRuntimeError(
                    f"{label} exceeds the bounded entry duration: {retiming['duration_sec']:.3f}s > {maximum:.3f}s"
                )
        points = trajectory.joint_trajectory.points
        endpoint = {name: float(points[-1].positions[index]) for index, name in enumerate(self.arm_joints)}
        return {
            "trajectory": trajectory,
            "endpoint": endpoint,
            "cartesian": {**details, "fraction": fraction, "avoid_collisions": True, "requested_waypoint_count": len(waypoints), "trajectory_waypoint_count": len(points), "planning_scene_ids": active_scene_ids, "retiming": retiming},
        }

    def route_joint_limit_margin(
        self,
        approach: dict[str, Any],
        segments: list[dict[str, Any]],
        *,
        approach_label: str = "approach",
    ) -> dict[str, Any]:
        """Fail closed on a pre-motion route with insufficient ABB limit margin."""

        required = finite(
            self.planner.get("preflight_minimum_joint_limit_margin_rad"),
            "pre-motion joint-limit margin",
            positive=True,
        )
        trajectories = [(approach_label, approach["trajectory"])]
        trajectories.extend(
            (str(segment["label"]), segment["trajectory"]) for segment in segments
        )
        by_joint = {name: math.inf for name in self.arm_joints}
        route_minimum, closest_joint, closest_trajectory, closest_point = (
            math.inf,
            "",
            "",
            -1,
        )
        for label, trajectory in trajectories:
            joint_trajectory = trajectory.joint_trajectory
            if tuple(joint_trajectory.joint_names) != self.arm_joints:
                raise DrawingRuntimeError(f"{label} did not use the exact six-axis controller order")
            if not joint_trajectory.points:
                raise DrawingRuntimeError(f"{label} has no joint trajectory points")
            for point_index, point in enumerate(joint_trajectory.points):
                if len(point.positions) != len(self.arm_joints):
                    raise DrawingRuntimeError(f"{label} point {point_index} has an invalid joint count")
                for joint_index, name in enumerate(self.arm_joints):
                    lower, upper = self.position_limits[name]
                    position = finite(point.positions[joint_index], f"{label} point {point_index} {name}")
                    margin = min(position - lower, upper - position)
                    if margin < 0.0:
                        raise DrawingRuntimeError(f"{label} point {point_index} violates the ABB hard limit for {name}")
                    by_joint[name] = min(by_joint[name], margin)
                    if margin < route_minimum:
                        route_minimum, closest_joint = margin, name
                        closest_trajectory, closest_point = label, point_index
        if not math.isfinite(route_minimum):
            raise DrawingRuntimeError("pre-motion route has no finite joint-limit margin")
        result = {
            "minimum_required_margin_rad": required,
            "route_minimum_margin_rad": route_minimum,
            "closest_joint": closest_joint,
            "closest_trajectory": closest_trajectory,
            "closest_trajectory_point_index": closest_point,
            "by_joint_minimum_margin_rad": by_joint,
        }
        if route_minimum < required:
            raise DrawingRuntimeError(
                "pre-motion route joint-limit margin is too small: "
                f"{closest_joint}={route_minimum:.6f} rad < {required:.6f} rad "
                f"in {closest_trajectory} point {closest_point}"
            )
        return result

    def preflight_cartesian_segments(
        self,
        waypoints: list[tuple[float, float, float]],
        orientation: tuple[float, float, float, float],
        scene_ids: list[str],
        start_positions: dict[str, float],
    ) -> list[dict[str, Any]]:
        """Build the entire segmented route before the robot is allowed to move.

        The approach planner can legitimately choose different IK branches for
        the same board pose.  A branch that reaches the first pen pose might
        still be unable to trace the next point collision-free.  Preflighting
        every contiguous Cartesian segment from that exact planned endpoint
        makes the branch selection deterministic before any physical motion.
        Each segment remains a MoveIt ``avoid_collisions=True`` Cartesian plan;
        no joint-space drawing waypoints are authored by this runtime.
        """

        segment_size = integer(
            self.planner.get("cartesian_segment_waypoints"),
            "Cartesian segment waypoint count",
            minimum=2,
        )
        if len(waypoints) < 2:
            raise DrawingRuntimeError("drawing geometry needs at least two Cartesian waypoints")
        planned: list[dict[str, Any]] = []
        predicted_state = dict(start_positions)
        start = 0
        while start < len(waypoints) - 1:
            end = min(len(waypoints), start + segment_size)
            current = waypoints[start:end]
            label = f"draw_segment_{len(planned):02d}"
            segment = self.cartesian_path(
                label,
                current,
                orientation,
                scene_ids,
                start_positions=predicted_state,
            )
            planned.append({"label": label, "waypoints": current, **segment})
            predicted_state.update(segment["endpoint"])
            if end == len(waypoints):
                break
            start = end - 1
        if not planned:
            raise DrawingRuntimeError("no Cartesian drawing segments were generated")
        return planned

    def execute_entry_alignment(
        self,
        preflight_entry: dict[str, Any],
        first_waypoint: tuple[float, float, float],
        orientation: tuple[float, float, float, float],
        scene_ids: list[str],
    ) -> dict[str, Any]:
        """Reach the first generated point before any curve chord is allowed.

        This remains a live-state MoveIt Cartesian request.  It never feeds a
        joint trajectory back into planning, and its target is exactly the
        first drawing-local point mapped through the live board frame.
        """

        label = "entry_alignment"
        live_entry = self.cartesian_path(
            label,
            [first_waypoint],
            orientation,
            scene_ids,
            retime_profile="entry",
        )
        live_joint_limit_margin = self.route_joint_limit_margin(
            {"trajectory": live_entry["trajectory"]},
            [],
            approach_label=label,
        )
        execution = self.execute(live_entry["trajectory"], label)
        observed = self.observe_endpoint(live_entry["endpoint"], label)
        tcp_gate = self.observe_tcp_endpoint(first_waypoint, orientation, label)
        endpoint_tolerance = finite(
            self.planner.get("observed_joint_tolerance_rad"),
            "joint endpoint tolerance",
            positive=True,
        )
        endpoint_samples = integer(
            self.planner.get("endpoint_samples"),
            "endpoint sample count",
            minimum=1,
        )
        return {
            **live_entry["cartesian"],
            "pre_motion_cartesian": preflight_entry["cartesian"],
            "live_joint_limit_margin": live_joint_limit_margin,
            "target_provenance": {
                "strategy": "live_state_to_runtime_generated_first_waypoint",
                "target_count": 1,
            },
            "execution": execution,
            "expected_endpoint": live_entry["endpoint"],
            "observed_endpoint": observed,
            "endpoint_gate": {
                "tolerance_rad": endpoint_tolerance,
                "consecutive_samples": endpoint_samples,
            },
            "tcp_gate": tcp_gate,
        }

    def execute_preflighted_cartesian_segments(
        self,
        planned: list[dict[str, Any]],
        orientation: tuple[float, float, float, float],
        scene_ids: list[str],
        *,
        requested_waypoint_count: int,
    ) -> dict[str, Any]:
        """Replan each accepted Cartesian segment from the observed live state.

        The pre-motion gate chooses an approach IK branch only after its whole
        route has been validated.  The Gazebo position loop may still settle a
        few milliradians away from an earlier segment's nominal endpoint.  A
        copied joint trajectory would then be rejected by MoveIt's strict
        start-state guard.  From that observed state, re-request only the
        segment's *forward* generated targets: returning first to the nominal
        previous endpoint would add an artificial backtrack before tracing the
        next curve chord.  The resulting live-start route is independently
        Cartesian collision checked; no joint-space correction is authored.
        """

        records: list[dict[str, Any]] = []
        for preflight_segment in planned:
            label = str(preflight_segment["label"])
            preflight_waypoints = list(preflight_segment["waypoints"])
            live_forward_targets = preflight_waypoints[1:]
            if not live_forward_targets:
                raise DrawingRuntimeError(f"{label} has no forward Cartesian target")
            live_segment = self.cartesian_path(
                label,
                live_forward_targets,
                orientation,
                scene_ids,
            )
            live_joint_limit_margin = self.route_joint_limit_margin(
                {"trajectory": live_segment["trajectory"]},
                [],
                approach_label=label,
            )
            execution = self.execute(live_segment["trajectory"], label)
            observed = self.observe_endpoint(live_segment["endpoint"], label)
            endpoint_tolerance = finite(
                self.planner.get("observed_joint_tolerance_rad"),
                "joint endpoint tolerance",
                positive=True,
            )
            endpoint_samples = integer(
                self.planner.get("endpoint_samples"),
                "endpoint sample count",
                minimum=1,
            )
            records.append(
                {
                    **live_segment["cartesian"],
                    "pre_motion_cartesian": preflight_segment["cartesian"],
                    "live_joint_limit_margin": live_joint_limit_margin,
                    "live_execution_targets": {
                        "strategy": "live_state_to_forward_generated_waypoints",
                        "preflight_waypoint_count": len(preflight_waypoints),
                        "live_forward_target_count": len(live_forward_targets),
                    },
                    "execution": execution,
                    "expected_endpoint": live_segment["endpoint"],
                    "observed_endpoint": observed,
                    "endpoint_gate": {
                        "tolerance_rad": endpoint_tolerance,
                        "consecutive_samples": endpoint_samples,
                    },
                }
            )
        fractions = [finite(record["fraction"], "Cartesian segment fraction") for record in records]
        return {
            "fraction": min(fractions),
            "avoid_collisions": True,
            "requested_waypoint_count": requested_waypoint_count,
            "trajectory_waypoint_count": sum(integer(record["trajectory_waypoint_count"], "segment trajectory points", minimum=2) for record in records),
            "segment_count": len(records),
            "segment_waypoint_limit": integer(
                self.planner.get("cartesian_segment_waypoints"),
                "Cartesian segment waypoint count",
                minimum=2,
            ),
            "execution": {"action_status": GoalStatus.STATUS_SUCCEEDED, "semantics": "all_cartesian_segments_succeeded"},
            "segments": records,
        }

    def orientations(self) -> list[tuple[str, tuple[float, float, float, float]]]:
        values = self.board_config.get("orientation_candidates")
        if not isinstance(values, list) or not values:
            raise DrawingRuntimeError("board orientation candidates are missing")
        result = []
        for index, item in enumerate(values):
            candidate = mapping(item, f"orientation candidate {index}")
            name = str(candidate.get("id", ""))
            quaternion = vector(candidate.get("xyzw"), f"orientation candidate {name}", 4)
            norm = math.sqrt(sum(value * value for value in quaternion))
            if not name or not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-5):
                raise DrawingRuntimeError(f"orientation candidate {index} is invalid")
            result.append((name, quaternion))
        return result

    def _shift_normal(self, point: tuple[float, float, float], frame: BoardFrame, amount: float) -> tuple[float, float, float]:
        return tuple(point[index] + amount * frame.normal_world[index] for index in range(3))

    def run(self) -> dict[str, Any]:
        self.wait_for_endpoints()
        board_observation = self.wait_board()
        static_obstacles = self.wait_static_obstacles(board_observation.stamp_ns)
        frame = self.board_frame(board_observation)
        horizontal_half = finite(self.board_config.get("horizontal_half_extent_m"), "board horizontal half extent", positive=True)
        vertical_half = finite(self.board_config.get("vertical_half_extent_m"), "board vertical half extent", positive=True)
        samples = integer(self.board_config.get("curve_samples"), "curve samples", minimum=16)
        local_points = generate_local_geometry(self.intent, samples=samples, horizontal_half_m=horizontal_half, vertical_half_m=vertical_half)
        standoff = finite(self.board_config.get("virtual_tip_standoff_m"), "virtual tip standoff", positive=True)
        world_points = frame.map_points(local_points, standoff_m=standoff)
        scene_ids, static_obstacle_records = self.apply_drawing_scene(frame, static_obstacles)
        approach_offset = finite(self.board_config.get("approach_offset_m"), "approach offset", positive=True)
        route_preflight_attempts = integer(
            self.planner.get("orientation_route_preflight_attempts"),
            "orientation route preflight attempt count",
            minimum=1,
        )
        attempts: list[dict[str, Any]] = []
        selected: dict[str, Any] | None = None
        for identifier, orientation in self.orientations():
            route_failures: list[dict[str, Any]] = []
            approach: dict[str, Any] | None = None
            preflight_entry: dict[str, Any] | None = None
            planned_segments: list[dict[str, Any]] | None = None
            route_joint_limit_margin: dict[str, Any] | None = None
            accepted_preflight_attempt: int | None = None
            for preflight_attempt in range(1, route_preflight_attempts + 1):
                try:
                    # Reach a clear standoff in free space, then preflight a
                    # separate collision-checked Cartesian entry to p0.  The
                    # first generated curve segment therefore never hides a
                    # normal descent or controller-residual correction.
                    approach_position = self._shift_normal(world_points[0], frame, approach_offset)
                    candidate_approach = self.plan_pose(
                        f"{identifier}_approach",
                        self.pose_stamped(approach_position, orientation),
                        scene_ids,
                    )
                    predicted_start = self.current_positions()
                    predicted_start.update(candidate_approach["endpoint"])
                    candidate_entry = {
                        "label": "entry_alignment",
                        "waypoints": [world_points[0]],
                        **self.cartesian_path(
                            "entry_alignment",
                            [world_points[0]],
                            orientation,
                            scene_ids,
                            start_positions=predicted_start,
                            retime_profile="entry",
                        ),
                    }
                    predicted_start.update(candidate_entry["endpoint"])
                    candidate_segments = self.preflight_cartesian_segments(
                        world_points,
                        orientation,
                        scene_ids,
                        predicted_start,
                    )
                    candidate_joint_limit_margin = self.route_joint_limit_margin(
                        candidate_approach,
                        [candidate_entry, *candidate_segments],
                    )
                except DrawingRuntimeError as exc:
                    route_failures.append({"preflight_attempt": preflight_attempt, "result": "REJECTED", "reason": str(exc)})
                    continue
                approach = candidate_approach
                preflight_entry = candidate_entry
                planned_segments = candidate_segments
                route_joint_limit_margin = candidate_joint_limit_margin
                accepted_preflight_attempt = preflight_attempt
                break
            if approach is None or preflight_entry is None or planned_segments is None or route_joint_limit_margin is None or accepted_preflight_attempt is None:
                attempts.append(
                    {
                        "orientation_candidate": identifier,
                        "result": "REJECTED",
                        "preflight_attempts": route_failures,
                    }
                )
                continue
            # Only a pre-motion route rejection may advance to the next
            # calibrated orientation.  The selected route has already passed
            # collision-aware Cartesian checks for every segment from the exact
            # approach endpoint.  Once motion starts, failure is fail-closed:
            # do not try another pose from an uncertain robot state.
            approach_execution = self.execute(approach["trajectory"], f"{identifier}_approach")
            approach_observed = self.observe_endpoint(approach["endpoint"], f"{identifier}_approach")
            entry_alignment = self.execute_entry_alignment(
                preflight_entry,
                world_points[0],
                orientation,
                scene_ids,
            )
            cartesian = self.execute_preflighted_cartesian_segments(
                planned_segments,
                orientation,
                scene_ids,
                requested_waypoint_count=len(world_points),
            )
            retreat_position = self._shift_normal(world_points[-1], frame, approach_offset)
            retreat = self.plan_pose(f"{identifier}_retreat", self.pose_stamped(retreat_position, orientation), scene_ids)
            retreat_execution = self.execute(retreat["trajectory"], f"{identifier}_retreat")
            retreat_observed = self.observe_endpoint(retreat["endpoint"], f"{identifier}_retreat")
            selected = {
                "orientation_candidate": identifier,
                "orientation_xyzw": list(orientation),
                "pre_motion_cartesian_gate": {
                    "result": "PASS",
                    "segment_count": len(planned_segments),
                    "collision_aware": True,
                    "avoid_collisions": True,
                    "start_state": "planned_approach_then_entry_endpoint",
                    "execution_route": "preflighted_live_entry_then_cartesian_trajectories",
                    "joint_limit_margin": route_joint_limit_margin,
                    "accepted_preflight_attempt": accepted_preflight_attempt,
                    "rejected_preflight_attempts": route_failures,
                },
                "approach": {"target": approach["target"], "planning": approach["planning"], "execution": approach_execution, "observed_endpoint": approach_observed},
                "entry_alignment": entry_alignment,
                "cartesian": cartesian,
                "retreat": {"target": retreat["target"], "planning": retreat["planning"], "execution": retreat_execution, "observed_endpoint": retreat_observed},
            }
            break
        if selected is None:
            raise DrawingRuntimeError(f"no collision-aware board orientation completed: {attempts}")
        if len(self.trace) < integer(self.board_config.get("minimum_tcp_trace_samples"), "minimum trace samples", minimum=1):
            raise DrawingRuntimeError("insufficient live TCP trace samples during drawing")
        return {
            "schema_version": 1,
            "scope": "SIMULATION_ONLY_CONTACTLESS_DRAWING_TRACE",
            "status": {"P6_DRAWING_RUNTIME": "PASS", "A05": "RUNTIME_EVIDENCE_RECORDED"},
            "started_at_utc": utc_now(),
            "online_ground_truth_input_used": False,
            "drawing_intent": {"shape": self.intent.shape.value, "size_m": self.intent.size_m, "position": self.intent.position.value, "orientation_rad": self.intent.orientation_rad, "source_text": self.intent.source_text},
            "geometry": {
                "generated_at_runtime": True,
                "coordinate_chain": "drawing-local -> drawing-board frame -> world -> MoveIt Cartesian waypoints -> ABB execution",
                "fixed_world_geometry_input": False,
                "local_points": [asdict(point) for point in local_points],
                "world_waypoints_m": [list(point) for point in world_points],
            },
            "board_frame": {
                "name": "drawing_board",
                "observation": board_observation.as_dict(),
                "observation_reference": "robot_facing_front_surface",
                "center_source": "fresh_rgbd_front_surface",
                "front_surface_world_m": [
                    board_observation.x,
                    board_observation.y,
                    board_observation.z,
                ],
                "derived_center_world_m": list(frame.center_world_m),
                "collision_center_world_m": list(frame.center_world_m),
                "normal_world": list(frame.normal_world),
                "horizontal_world": list(frame.horizontal_world),
                "vertical_world": list(frame.vertical_world),
                "front_surface_offset_m": frame.front_surface_offset_m,
                "virtual_tip_standoff_m": standoff,
                "board_collision_preserved": True,
                "tool_link": self.tcp_link,
                "tool_model": "phase6_collision_bearing_drawing_pen",
            },
            "planning_scene_ids": scene_ids,
            "static_collision_obstacles": {
                "source_topic": self.static_obstacle_contract.output_topic,
                "required_marker_class_ids": list(self.static_obstacle_contract.marker_class_ids),
                "records": static_obstacle_records,
            },
            "orientation_attempts": attempts,
            "execution": selected,
            "tcp_trace": self.trace,
            "finished_at_utc": utc_now(),
        }


def load_config(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise DrawingRuntimeError(f"drawing config is missing: {path}")
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise DrawingRuntimeError(f"cannot parse drawing config: {exc}") from exc
    config = mapping(document, "drawing config")
    if config.get("schema_version") != 1 or config.get("scope") != "SIMULATION_ONLY":
        raise DrawingRuntimeError("drawing config has an unsupported schema or scope")
    return config


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--instruction", required=True, help="One drawing-domain instruction.")
    result.add_argument("--output-root", required=True, type=Path, help="Empty evidence directory outside the source tree.")
    result.add_argument("--timeout", type=float, default=150.0, help="Bounded wall-clock episode timeout.")
    result.add_argument("--config", type=Path, help="Override installed drawing runtime config.")
    return result


def main(args: list[str] | None = None) -> int:
    options = parser().parse_args(args)
    try:
        if options.output_root.exists() and any(options.output_root.iterdir()):
            raise DrawingRuntimeError("output root must be new or empty")
        intent = parse_drawing_instruction(options.instruction)
        config_path = options.config or Path(get_package_share_directory("physical_ai_drawing")) / "config" / "drawing_runtime.yaml"
        config = load_config(config_path)
        rclpy.init(args=None)
        runtime: DrawingRuntime | None = None
        try:
            runtime = DrawingRuntime(config, intent, options.output_root, options.timeout)
            evidence = runtime.run()
            atomic_write_json(options.output_root / "drawing_execution_evidence.json", evidence)
        finally:
            if runtime is not None:
                runtime.destroy_node()
            rclpy.shutdown()
    except (DrawingIntentError, DrawingRuntimeError) as exc:
        print(f"P6_DRAWING_RUNTIME=FAIL: {exc}")
        return 1
    print("P6_DRAWING_RUNTIME=PASS")
    print(f"P6_DRAWING_EVIDENCE={options.output_root / 'drawing_execution_evidence.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
