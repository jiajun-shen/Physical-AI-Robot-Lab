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

    def as_dict(self) -> dict[str, Any]:
        return {
            "class_id": self.class_id,
            "position_m": [self.x, self.y, self.z],
            "source_stamp_ns": self.stamp_ns,
            "confidence": self.confidence,
            "detection_id": self.detection_id,
            "source_mode": "VISION",
        }


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
        self.arm_joints = tuple(self.robot.get("arm_joint_names", ()))
        if self.arm_joints != tuple(f"joint_{index}" for index in range(1, 7)):
            raise DrawingRuntimeError("drawing runtime requires the bounded six-axis ABB arm")
        self.gripper_joint = str(self.robot.get("gripper_joint", ""))
        self.planning_frame = str(self.robot.get("planning_frame", ""))
        self.tcp_link = str(self.robot.get("tcp_link", ""))
        if self.planning_frame != "world" or self.tcp_link != "gripper_tcp" or not self.gripper_joint:
            raise DrawingRuntimeError("drawing runtime has an unexpected planning frame, TCP, or gripper joint")
        self.velocity_limits = {
            name: finite(mapping(self.robot.get("velocity_limits_rad_per_sec"), "velocity limits").get(name), f"velocity limit {name}", positive=True)
            for name in self.arm_joints
        }
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
        self.trace: list[dict[str, Any]] = []
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, qos_profile_sensor_data)
        self.create_subscription(ObjectPoseArray, "/perception/world_object_poses", self._on_vision, qos_profile_sensor_data)

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
            )
        if observed:
            self.latest_vision.update(observed)
            self.vision_sequence += 1

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

    def retime(self, trajectory: Any, label: str) -> tuple[Any, dict[str, Any]]:
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
        velocity_scale = finite(self.planner.get("velocity_scaling"), "velocity scaling", positive=True)
        safety = finite(self.planner.get("retime_safety_factor"), "retime safety factor", positive=True)
        minimum = finite(self.planner.get("minimum_segment_duration_sec"), "minimum duration", positive=True)
        elapsed, durations = 0.0, []
        self._set_time(cleaned[0], 0.0)
        for index in range(1, len(cleaned)):
            prior, current = cleaned[index - 1], cleaned[index]
            needed = max(abs(float(current.positions[joint]) - float(prior.positions[joint])) / (self.velocity_limits[name] * velocity_scale) for joint, name in enumerate(self.arm_joints))
            duration = max(minimum, needed * safety)
            elapsed += duration
            self._set_time(current, elapsed)
            durations.append(duration)
        joint_trajectory.points = cleaned
        return result, {
            "method": "velocity_only_conservative_retime",
            "velocity_scaling": velocity_scale,
            "safety_factor": safety,
            "minimum_segment_duration_sec": minimum,
            "segment_durations_sec": durations,
            "duration_sec": elapsed,
            "acceleration_limits_used": False,
        }

    def _record_tcp(self, phase: str) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(self.planning_frame, self.tcp_link, Time(), timeout=RosDuration(seconds=0.12))
        except TransformException:
            return
        translation = transform.transform.translation
        values = (float(translation.x), float(translation.y), float(translation.z))
        if all(math.isfinite(value) for value in values):
            self.trace.append({"phase": phase, "wall_time_utc": utc_now(), "sim_time_ns": stamp_ns(transform.header), "position_m": list(values)})

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

    def board_frame(self, observed: VisionPose) -> BoardFrame:
        return BoardFrame(
            center_world_m=(observed.x, observed.y, observed.z),
            normal_world=vector(self.board_config.get("robot_facing_normal_world"), "board normal", 3),
            horizontal_world=vector(self.board_config.get("horizontal_axis_world"), "board horizontal axis", 3),
            vertical_world=vector(self.board_config.get("vertical_axis_world"), "board vertical axis", 3),
            front_surface_offset_m=finite(self.board_config.get("front_surface_offset_m"), "board front offset", positive=True),
        )

    def apply_drawing_scene(self, board: VisionPose) -> list[str]:
        table = mapping(self.scene_config.get("table"), "table scene geometry")
        table_center = vector(table.get("center_xyz_m"), "table center", 3)
        table_size = vector(table.get("size_xyz_m"), "table size", 3)
        board_size = vector(self.board_config.get("size_xyz_m"), "board size", 3)
        objects = [
            self.make_box(str(table.get("id")), table_center, table_size),
            self.make_box("p6_live_vision_drawing_board", (board.x, board.y, board.z), board_size),
        ]
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
        required = {str(table.get("id")), "p6_live_vision_drawing_board"}
        if not required.issubset(active):
            raise DrawingRuntimeError(f"MoveIt scene is missing required collision geometry: {sorted(required.difference(active))}")
        return active

    def plan_pose(self, label: str, target: PoseStamped, scene_ids: list[str]) -> dict[str, Any]:
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
            "planning": {**details, "collision_aware": True, "planning_scene_ids": scene_ids, "waypoint_count": len(points), "retiming": retiming},
            "endpoint": endpoint,
        }

    @staticmethod
    def _motion_plan_service(request: MotionPlanRequest) -> GetMotionPlan.Request:
        result = GetMotionPlan.Request()
        result.motion_plan_request = request
        return result

    def cartesian_path(self, label: str, waypoints: list[tuple[float, float, float]], orientation: tuple[float, float, float, float], scene_ids: list[str]) -> dict[str, Any]:
        request = GetCartesianPath.Request()
        request.header.frame_id = self.planning_frame
        request.start_state = self.robot_state()
        request.group_name = str(self.robot.get("group_name"))
        request.link_name = self.tcp_link
        request.waypoints = [self.pose(point, orientation) for point in waypoints]
        request.max_step = finite(self.planner.get("cartesian_max_step_m"), "Cartesian max step", positive=True)
        request.jump_threshold = finite(self.planner.get("cartesian_jump_threshold"), "Cartesian jump threshold")
        request.prismatic_jump_threshold = finite(self.planner.get("cartesian_prismatic_jump_threshold"), "Cartesian prismatic jump threshold")
        request.revolute_jump_threshold = finite(self.planner.get("cartesian_revolute_jump_threshold"), "Cartesian revolute jump threshold")
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = finite(self.planner.get("velocity_scaling"), "velocity scaling", positive=True)
        request.max_acceleration_scaling_factor = finite(self.planner.get("cartesian_acceleration_scaling"), "Cartesian acceleration scaling", positive=True)
        response = self.call_service(self.cartesian_client, request, f"Cartesian {label}")
        details = self.error_details(response.error_code)
        fraction = float(response.fraction)
        if response.error_code.val != SUCCESS or fraction < 0.999999:
            raise DrawingRuntimeError(f"collision-aware Cartesian path failed for {label}: fraction={fraction}, {details}")
        trajectory, retiming = self.retime(response.solution, label)
        points = trajectory.joint_trajectory.points
        endpoint = {name: float(points[-1].positions[index]) for index, name in enumerate(self.arm_joints)}
        return {
            "trajectory": trajectory,
            "endpoint": endpoint,
            "cartesian": {**details, "fraction": fraction, "avoid_collisions": True, "requested_waypoint_count": len(waypoints), "trajectory_waypoint_count": len(points), "planning_scene_ids": scene_ids, "retiming": retiming},
        }

    def execute_cartesian_segments(
        self,
        waypoints: list[tuple[float, float, float]],
        orientation: tuple[float, float, float, float],
        scene_ids: list[str],
    ) -> dict[str, Any]:
        """Execute one generated curve as short, contiguous Cartesian paths.

        This avoids accumulating a controller tracking error over a long
        no-acceleration-limits trajectory.  Every segment is independently
        generated by MoveIt's Cartesian service with ``avoid_collisions=True``;
        overlapping end/start waypoints preserve the one curve's continuity.
        """

        segment_size = integer(
            self.planner.get("cartesian_segment_waypoints"),
            "Cartesian segment waypoint count",
            minimum=2,
        )
        if len(waypoints) < 2:
            raise DrawingRuntimeError("drawing geometry needs at least two Cartesian waypoints")
        records: list[dict[str, Any]] = []
        start = 0
        while start < len(waypoints) - 1:
            end = min(len(waypoints), start + segment_size)
            current = waypoints[start:end]
            label = f"draw_segment_{len(records):02d}"
            planned = self.cartesian_path(label, current, orientation, scene_ids)
            execution = self.execute(planned["trajectory"], label)
            observed = self.observe_endpoint(planned["endpoint"], label)
            records.append({**planned["cartesian"], "execution": execution, "observed_endpoint": observed})
            if end == len(waypoints):
                break
            start = end - 1
        if not records:
            raise DrawingRuntimeError("no Cartesian drawing segments were generated")
        fractions = [finite(record["fraction"], "Cartesian segment fraction") for record in records]
        return {
            "fraction": min(fractions),
            "avoid_collisions": True,
            "requested_waypoint_count": len(waypoints),
            "trajectory_waypoint_count": sum(integer(record["trajectory_waypoint_count"], "segment trajectory points", minimum=2) for record in records),
            "segment_count": len(records),
            "segment_waypoint_limit": segment_size,
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
        frame = self.board_frame(board_observation)
        horizontal_half = finite(self.board_config.get("horizontal_half_extent_m"), "board horizontal half extent", positive=True)
        vertical_half = finite(self.board_config.get("vertical_half_extent_m"), "board vertical half extent", positive=True)
        samples = integer(self.board_config.get("curve_samples"), "curve samples", minimum=16)
        local_points = generate_local_geometry(self.intent, samples=samples, horizontal_half_m=horizontal_half, vertical_half_m=vertical_half)
        standoff = finite(self.board_config.get("virtual_tip_standoff_m"), "virtual tip standoff", positive=True)
        world_points = frame.map_points(local_points, standoff_m=standoff)
        scene_ids = self.apply_drawing_scene(board_observation)
        approach_offset = finite(self.board_config.get("approach_offset_m"), "approach offset", positive=True)
        attempts: list[dict[str, Any]] = []
        selected: dict[str, Any] | None = None
        for identifier, orientation in self.orientations():
            try:
                approach_position = self._shift_normal(world_points[0], frame, approach_offset)
                approach = self.plan_pose(f"{identifier}_approach", self.pose_stamped(approach_position, orientation), scene_ids)
            except DrawingRuntimeError as exc:
                attempts.append({"orientation_candidate": identifier, "result": "REJECTED", "reason": str(exc)})
                continue
            # Only a pre-motion approach-planning rejection may advance to the
            # next calibrated orientation. Once motion has started, a failure
            # leaves robot state uncertain, so every subsequent gate is
            # fail-closed rather than trying another pose from that state.
            approach_execution = self.execute(approach["trajectory"], f"{identifier}_approach")
            approach_observed = self.observe_endpoint(approach["endpoint"], f"{identifier}_approach")
            cartesian = self.execute_cartesian_segments(world_points, orientation, scene_ids)
            retreat_position = self._shift_normal(world_points[-1], frame, approach_offset)
            retreat = self.plan_pose(f"{identifier}_retreat", self.pose_stamped(retreat_position, orientation), scene_ids)
            retreat_execution = self.execute(retreat["trajectory"], f"{identifier}_retreat")
            retreat_observed = self.observe_endpoint(retreat["endpoint"], f"{identifier}_retreat")
            selected = {
                "orientation_candidate": identifier,
                "orientation_xyzw": list(orientation),
                "approach": {"target": approach["target"], "planning": approach["planning"], "execution": approach_execution, "observed_endpoint": approach_observed},
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
                "center_source": "fresh_rgbd_vision",
                "normal_world": list(frame.normal_world),
                "horizontal_world": list(frame.horizontal_world),
                "vertical_world": list(frame.vertical_world),
                "front_surface_offset_m": frame.front_surface_offset_m,
                "virtual_tip_standoff_m": standoff,
                "board_collision_preserved": True,
            },
            "planning_scene_ids": scene_ids,
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
