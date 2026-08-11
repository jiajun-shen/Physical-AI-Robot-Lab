#!/usr/bin/env python3
"""Run bounded, evidence-producing MoveIt checks against the live ABB stack.

This verifier is deliberately a ROS client, not a mock or a controller CLI
wrapper.  It asks the running ``move_group`` process to plan two motions and
executes the returned trajectories through ``/execute_trajectory``.  It then
uses ``/compute_fk`` to derive a Cartesian target, and finally proves that an
explicitly applied world collision object makes that target invalid and causes
MoveIt planning to reject it.

The root P2.2 harness must treat a non-zero exit as a failed runtime gate.  No
PASS marker is printed until all three demonstrations have completed and the
JSON evidence document has been assembled.

The shipped smoke configuration binds the project model's ``tool0`` link; the
runtime client reads that value from configuration rather than silently
hard-coding a link name.
"""

from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
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
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PlanningSceneComponents,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetMotionPlan,
    GetPositionFK,
    GetPlanningScene,
    GetStateValidity,
)
import rclpy
from rclpy.action import ActionClient
from rclpy.client import Client
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
import yaml


EXPECTED_JOINTS = tuple(f"joint_{index}" for index in range(1, 7))
CONTROLLER_NAME = "joint_trajectory_controller"
SUCCESS = MoveItErrorCodes.SUCCESS


class VerificationError(RuntimeError):
    """A required live planning, execution, or observation proof is absent."""


@dataclass(frozen=True)
class SmokeConfig:
    """Reviewed, finite runtime inputs for the P2.2 simulation smoke run."""

    group_name: str
    base_frame: str
    tool_link: str
    joint_names: tuple[str, ...]
    pipeline_id: str
    planner_id: str
    planning_attempts: int
    allowed_planning_time_sec: float
    velocity_scaling: float
    home: dict[str, float]
    joint_execution: dict[str, float]
    joint_alternate: dict[str, float]
    fk_pose_seed: dict[str, float]
    endpoint_samples: int
    minimum_joint_motion_rad: float
    observed_home_tolerance_rad: float
    joint_goal_tolerance_rad: float
    observed_joint_tolerance_rad: float
    pose_goal_position_tolerance_m: float
    pose_goal_orientation_tolerance_rad: float
    observed_pose_position_tolerance_m: float
    observed_pose_orientation_tolerance_rad: float
    velocity_only_retime_safety_factor: float
    minimum_segment_duration_sec: float
    service_wait_timeout_sec: float
    service_call_timeout_sec: float
    action_result_timeout_sec: float
    endpoint_observation_timeout_sec: float
    collision_object_id: str
    collision_box_dimensions_m: tuple[float, float, float]


def require_mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise VerificationError(f"{label} must be a YAML mapping")
    return value


def require_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise VerificationError(f"{label} must be a non-empty string")
    return value


def require_finite(value: Any, label: str, *, positive: bool = False) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise VerificationError(f"{label} must be numeric") from exc
    if not math.isfinite(number):
        raise VerificationError(f"{label} must be finite")
    if positive and number <= 0.0:
        raise VerificationError(f"{label} must be positive")
    return number


def require_unit_interval(value: Any, label: str) -> float:
    number = require_finite(value, label, positive=True)
    if number > 1.0:
        raise VerificationError(f"{label} must be no greater than 1.0")
    return number


def require_positive_int(value: Any, label: str) -> int:
    if isinstance(value, bool):
        raise VerificationError(f"{label} must be a positive integer")
    try:
        number = int(value)
    except (TypeError, ValueError) as exc:
        raise VerificationError(f"{label} must be a positive integer") from exc
    if number < 1 or number != value:
        raise VerificationError(f"{label} must be a positive integer")
    return number


def require_joint_map(
    value: Any, label: str, joint_names: tuple[str, ...]
) -> dict[str, float]:
    mapping = require_mapping(value, label)
    if set(mapping) != set(joint_names):
        raise VerificationError(
            f"{label} joints must be exactly {list(joint_names)}, got {sorted(mapping)}"
        )
    return {
        name: require_finite(mapping[name], f"{label}.{name}") for name in joint_names
    }


def load_smoke_config(path: Path) -> SmokeConfig:
    if not path.is_file():
        raise VerificationError(f"smoke configuration does not exist: {path}")
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise VerificationError(f"could not parse smoke configuration {path}") from exc
    root = require_mapping(document, str(path))
    if root.get("schema_version") != 1:
        raise VerificationError("smoke configuration schema_version must be 1")
    if root.get("scope") != "SIMULATION_ONLY":
        raise VerificationError("smoke configuration scope must be SIMULATION_ONLY")

    robot = require_mapping(root.get("robot"), "robot")
    joint_names_raw = robot.get("joint_names")
    if not isinstance(joint_names_raw, list):
        raise VerificationError("robot.joint_names must be a list")
    joint_names = tuple(require_string(value, "robot.joint_names") for value in joint_names_raw)
    if joint_names != EXPECTED_JOINTS:
        raise VerificationError(
            f"robot.joint_names must be exactly {list(EXPECTED_JOINTS)}"
        )

    planner = require_mapping(root.get("planner"), "planner")
    targets = require_mapping(root.get("targets"), "targets")
    guards = require_mapping(root.get("guards"), "guards")
    collision = require_mapping(root.get("collision"), "collision")
    dimensions_raw = collision.get("box_dimensions_m")
    if not isinstance(dimensions_raw, list) or len(dimensions_raw) != 3:
        raise VerificationError("collision.box_dimensions_m must contain three values")
    dimensions = tuple(
        require_finite(value, "collision.box_dimensions_m", positive=True)
        for value in dimensions_raw
    )

    return SmokeConfig(
        group_name=require_string(robot.get("group_name"), "robot.group_name"),
        base_frame=require_string(robot.get("base_frame"), "robot.base_frame"),
        tool_link=require_string(robot.get("tool_link"), "robot.tool_link"),
        joint_names=joint_names,
        pipeline_id=require_string(planner.get("pipeline_id"), "planner.pipeline_id"),
        planner_id=require_string(planner.get("planner_id"), "planner.planner_id"),
        planning_attempts=require_positive_int(
            planner.get("planning_attempts"), "planner.planning_attempts"
        ),
        allowed_planning_time_sec=require_finite(
            planner.get("allowed_planning_time_sec"),
            "planner.allowed_planning_time_sec",
            positive=True,
        ),
        velocity_scaling=require_unit_interval(
            planner.get("velocity_scaling"), "planner.velocity_scaling"
        ),
        home=require_joint_map(targets.get("home"), "targets.home", joint_names),
        joint_execution=require_joint_map(
            targets.get("joint_execution"), "targets.joint_execution", joint_names
        ),
        joint_alternate=require_joint_map(
            targets.get("joint_alternate"), "targets.joint_alternate", joint_names
        ),
        fk_pose_seed=require_joint_map(
            targets.get("fk_pose_seed"), "targets.fk_pose_seed", joint_names
        ),
        endpoint_samples=require_positive_int(
            guards.get("endpoint_samples"), "guards.endpoint_samples"
        ),
        minimum_joint_motion_rad=require_finite(
            guards.get("minimum_joint_motion_rad"),
            "guards.minimum_joint_motion_rad",
            positive=True,
        ),
        observed_home_tolerance_rad=require_finite(
            guards.get("observed_home_tolerance_rad"),
            "guards.observed_home_tolerance_rad",
            positive=True,
        ),
        joint_goal_tolerance_rad=require_finite(
            guards.get("joint_goal_tolerance_rad"),
            "guards.joint_goal_tolerance_rad",
            positive=True,
        ),
        observed_joint_tolerance_rad=require_finite(
            guards.get("observed_joint_tolerance_rad"),
            "guards.observed_joint_tolerance_rad",
            positive=True,
        ),
        pose_goal_position_tolerance_m=require_finite(
            guards.get("pose_goal_position_tolerance_m"),
            "guards.pose_goal_position_tolerance_m",
            positive=True,
        ),
        pose_goal_orientation_tolerance_rad=require_finite(
            guards.get("pose_goal_orientation_tolerance_rad"),
            "guards.pose_goal_orientation_tolerance_rad",
            positive=True,
        ),
        observed_pose_position_tolerance_m=require_finite(
            guards.get("observed_pose_position_tolerance_m"),
            "guards.observed_pose_position_tolerance_m",
            positive=True,
        ),
        observed_pose_orientation_tolerance_rad=require_finite(
            guards.get("observed_pose_orientation_tolerance_rad"),
            "guards.observed_pose_orientation_tolerance_rad",
            positive=True,
        ),
        velocity_only_retime_safety_factor=require_finite(
            guards.get("velocity_only_retime_safety_factor"),
            "guards.velocity_only_retime_safety_factor",
            positive=True,
        ),
        minimum_segment_duration_sec=require_finite(
            guards.get("minimum_segment_duration_sec"),
            "guards.minimum_segment_duration_sec",
            positive=True,
        ),
        service_wait_timeout_sec=require_finite(
            guards.get("service_wait_timeout_sec"),
            "guards.service_wait_timeout_sec",
            positive=True,
        ),
        service_call_timeout_sec=require_finite(
            guards.get("service_call_timeout_sec"),
            "guards.service_call_timeout_sec",
            positive=True,
        ),
        action_result_timeout_sec=require_finite(
            guards.get("action_result_timeout_sec"),
            "guards.action_result_timeout_sec",
            positive=True,
        ),
        endpoint_observation_timeout_sec=require_finite(
            guards.get("endpoint_observation_timeout_sec"),
            "guards.endpoint_observation_timeout_sec",
            positive=True,
        ),
        collision_object_id=require_string(collision.get("object_id"), "collision.object_id"),
        collision_box_dimensions_m=(
            float(dimensions[0]),
            float(dimensions[1]),
            float(dimensions[2]),
        ),
    )


def load_canonical_velocity_limits(
    path: Path, joint_names: tuple[str, ...]
) -> dict[str, float]:
    """Read only the project-authoritative, verified velocity maxima.

    This intentionally rejects acceleration/deceleration/jerk data rather than
    filling in a guessed value. The values are used only to schedule position
    waypoints conservatively before ``ExecuteTrajectory``.
    """
    if not path.is_file():
        raise VerificationError(f"canonical joint limits do not exist: {path}")
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise VerificationError(f"could not parse canonical joint limits {path}") from exc
    root = require_mapping(document, str(path))
    if root.get("scope") != "SIMULATION_ONLY":
        raise VerificationError("canonical joint limits must be SIMULATION_ONLY")
    limits = require_mapping(root.get("joint_limits"), "canonical joint_limits")
    if set(limits) != set(joint_names):
        raise VerificationError(
            "canonical joint limits must cover exactly "
            f"{list(joint_names)}, got {sorted(limits)}"
        )
    velocity_limits: dict[str, float] = {}
    for name in joint_names:
        limit = require_mapping(limits[name], f"canonical joint_limits.{name}")
        if limit.get("has_velocity_limits") is not True:
            raise VerificationError(
                f"canonical velocity limit is not enabled for {name}"
            )
        velocity_limits[name] = require_finite(
            limit.get("max_velocity"),
            f"canonical joint_limits.{name}.max_velocity",
            positive=True,
        )
    return velocity_limits


def map_joint_state(message: JointState, joint_names: tuple[str, ...]) -> dict[str, float]:
    names = list(message.name)
    if len(names) != len(set(names)):
        raise VerificationError("/joint_states contains duplicate joint names")
    if len(names) != len(joint_names) or set(names) != set(joint_names):
        raise VerificationError(
            "/joint_states joint names must be exactly "
            f"{list(joint_names)}, got {names}"
        )
    if len(message.position) != len(names):
        raise VerificationError("/joint_states position length does not match names")
    positions = {name: float(message.position[index]) for index, name in enumerate(names)}
    if not all(math.isfinite(value) for value in positions.values()):
        raise VerificationError("/joint_states contains a non-finite joint position")
    if message.velocity and len(message.velocity) != len(names):
        raise VerificationError("/joint_states velocity length does not match names")
    if message.velocity and not all(math.isfinite(float(value)) for value in message.velocity):
        raise VerificationError("/joint_states contains a non-finite joint velocity")
    return positions


def map_distance(left: dict[str, float], right: dict[str, float]) -> float:
    return max(abs(left[name] - right[name]) for name in EXPECTED_JOINTS)


def map_as_json(values: dict[str, float]) -> dict[str, float]:
    return {name: float(values[name]) for name in EXPECTED_JOINTS}


def pose_as_json(pose: PoseStamped) -> dict[str, Any]:
    return {
        "frame_id": pose.header.frame_id,
        "position": {
            "x": float(pose.pose.position.x),
            "y": float(pose.pose.position.y),
            "z": float(pose.pose.position.z),
        },
        "orientation": {
            "x": float(pose.pose.orientation.x),
            "y": float(pose.pose.orientation.y),
            "z": float(pose.pose.orientation.z),
            "w": float(pose.pose.orientation.w),
        },
    }


def require_finite_pose(
    pose: PoseStamped, label: str, expected_frame: str
) -> None:
    if pose.header.frame_id != expected_frame:
        raise VerificationError(
            f"{label} must be expressed in {expected_frame!r}, got "
            f"{pose.header.frame_id!r}"
        )
    values = (
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise VerificationError(f"{label} contains non-finite values")
    norm = math.sqrt(
        pose.pose.orientation.x**2
        + pose.pose.orientation.y**2
        + pose.pose.orientation.z**2
        + pose.pose.orientation.w**2
    )
    if norm < 0.5:
        raise VerificationError(f"{label} has a degenerate quaternion")


def quaternion_angular_error(left: Pose, right: Pose) -> float:
    left_norm = math.sqrt(
        left.orientation.x**2
        + left.orientation.y**2
        + left.orientation.z**2
        + left.orientation.w**2
    )
    right_norm = math.sqrt(
        right.orientation.x**2
        + right.orientation.y**2
        + right.orientation.z**2
        + right.orientation.w**2
    )
    if left_norm < 0.5 or right_norm < 0.5:
        raise VerificationError("cannot compare a degenerate orientation quaternion")
    dot_product = (
        left.orientation.x * right.orientation.x
        + left.orientation.y * right.orientation.y
        + left.orientation.z * right.orientation.z
        + left.orientation.w * right.orientation.w
    ) / (left_norm * right_norm)
    return 2.0 * math.acos(max(-1.0, min(1.0, abs(dot_product))))


def position_error(left: Pose, right: Pose) -> float:
    return math.sqrt(
        (left.position.x - right.position.x) ** 2
        + (left.position.y - right.position.y) ** 2
        + (left.position.z - right.position.z) ** 2
    )


class MoveGroupRuntimeVerifier(Node):
    """Bounded client for real MoveGroup planning, execution, and scene APIs."""

    def __init__(
        self,
        config: SmokeConfig,
        canonical_velocity_limits: dict[str, float],
        canonical_limits_path: Path,
        timeout_sec: float,
    ) -> None:
        super().__init__(
            "abb_lab_moveit_runtime_verifier",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.config = config
        self.canonical_velocity_limits = dict(canonical_velocity_limits)
        self.canonical_limits_path = canonical_limits_path
        self.deadline = time.monotonic() + timeout_sec
        self.plan_client: Client = self.create_client(
            GetMotionPlan, "/plan_kinematic_path"
        )
        self.fk_client: Client = self.create_client(GetPositionFK, "/compute_fk")
        self.scene_client: Client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self.scene_query_client: Client = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self.validity_client: Client = self.create_client(
            GetStateValidity, "/check_state_validity"
        )
        self.execute_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )
        self.latest_joint_state: dict[str, float] | None = None
        self.latest_joint_sequence = 0
        self.valid_joint_state_frames = 0
        self.invalid_joint_state_frames = 0
        self.last_joint_state_error = ""
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            qos_profile_sensor_data,
        )

    def remaining(self) -> float:
        return self.deadline - time.monotonic()

    def _bounded_deadline(self, maximum_sec: float, label: str) -> float:
        if maximum_sec <= 0.0:
            raise VerificationError(f"{label} timeout must be positive")
        remaining = self.remaining()
        if remaining <= 0.0:
            raise VerificationError(f"global verifier timeout expired before {label}")
        return min(self.deadline, time.monotonic() + min(maximum_sec, remaining))

    def _spin_until(
        self, predicate: Callable[[], bool], label: str, maximum_sec: float
    ) -> None:
        deadline = self._bounded_deadline(maximum_sec, label)
        while time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(
                self,
                timeout_sec=max(0.0, min(0.1, deadline - time.monotonic())),
            )
        if not predicate():
            raise VerificationError(f"timed out waiting for {label}")

    def _on_joint_state(self, message: JointState) -> None:
        try:
            positions = map_joint_state(message, self.config.joint_names)
        except VerificationError as exc:
            self.invalid_joint_state_frames += 1
            self.last_joint_state_error = str(exc)
            return
        self.latest_joint_state = positions
        self.latest_joint_sequence += 1
        self.valid_joint_state_frames += 1

    def wait_for_endpoints(self) -> None:
        endpoints: tuple[tuple[Client, str], ...] = (
            (self.plan_client, "/plan_kinematic_path"),
            (self.fk_client, "/compute_fk"),
            (self.scene_client, "/apply_planning_scene"),
            (self.scene_query_client, "/get_planning_scene"),
            (self.validity_client, "/check_state_validity"),
        )
        for client, name in endpoints:
            deadline = self._bounded_deadline(
                self.config.service_wait_timeout_sec, f"service {name}"
            )
            while time.monotonic() < deadline:
                if client.wait_for_service(
                    timeout_sec=max(0.0, min(0.2, deadline - time.monotonic()))
                ):
                    break
            else:
                raise VerificationError(f"service is not ready: {name}")

        deadline = self._bounded_deadline(
            self.config.service_wait_timeout_sec, "action /execute_trajectory"
        )
        while time.monotonic() < deadline:
            if self.execute_client.wait_for_server(
                timeout_sec=max(0.0, min(0.2, deadline - time.monotonic()))
            ):
                return
        raise VerificationError("action server is not ready: /execute_trajectory")

    def wait_for_joint_state(self) -> dict[str, float]:
        self._spin_until(
            lambda: self.latest_joint_state is not None,
            "an exact six-axis /joint_states sample",
            self.config.endpoint_observation_timeout_sec,
        )
        assert self.latest_joint_state is not None
        return dict(self.latest_joint_state)

    def call_service(self, client: Client, request: Any, label: str) -> Any:
        future = client.call_async(request)
        self._spin_until(
            future.done,
            f"service result for {label}",
            self.config.service_call_timeout_sec,
        )
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - surface a ROS future failure
            raise VerificationError(f"service call failed for {label}: {exc}") from exc
        if response is None:
            raise VerificationError(f"service returned no result for {label}")
        return response

    def robot_state(self, positions: dict[str, float]) -> RobotState:
        state = RobotState()
        state.is_diff = False
        state.joint_state.name = list(self.config.joint_names)
        state.joint_state.position = [positions[name] for name in self.config.joint_names]
        return state

    def plan_request_base(self, start_positions: dict[str, float]) -> MotionPlanRequest:
        request = MotionPlanRequest()
        request.group_name = self.config.group_name
        request.pipeline_id = self.config.pipeline_id
        request.planner_id = self.config.planner_id
        request.start_state = self.robot_state(start_positions)
        request.num_planning_attempts = self.config.planning_attempts
        request.allowed_planning_time = self.config.allowed_planning_time_sec
        request.max_velocity_scaling_factor = self.config.velocity_scaling
        return request

    def build_joint_request(
        self, start_positions: dict[str, float], target_positions: dict[str, float]
    ) -> MotionPlanRequest:
        request = self.plan_request_base(start_positions)
        constraints = Constraints()
        for name in self.config.joint_names:
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = target_positions[name]
            constraint.tolerance_above = self.config.joint_goal_tolerance_rad
            constraint.tolerance_below = self.config.joint_goal_tolerance_rad
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        request.goal_constraints = [constraints]
        return request

    def build_pose_request(
        self, start_positions: dict[str, float], target: PoseStamped
    ) -> MotionPlanRequest:
        require_finite_pose(
            target, "FK-derived pose target", self.config.base_frame
        )
        request = self.plan_request_base(start_positions)
        constraints = Constraints()

        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.config.pose_goal_position_tolerance_m]
        region.primitives = [sphere]
        region.primitive_poses = [copy.deepcopy(target.pose)]

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.config.base_frame
        position_constraint.link_name = self.config.tool_link
        position_constraint.constraint_region = region
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.config.base_frame
        orientation_constraint.link_name = self.config.tool_link
        orientation_constraint.orientation = copy.deepcopy(target.pose.orientation)
        orientation_constraint.absolute_x_axis_tolerance = (
            self.config.pose_goal_orientation_tolerance_rad
        )
        orientation_constraint.absolute_y_axis_tolerance = (
            self.config.pose_goal_orientation_tolerance_rad
        )
        orientation_constraint.absolute_z_axis_tolerance = (
            self.config.pose_goal_orientation_tolerance_rad
        )
        orientation_constraint.parameterization = OrientationConstraint.ROTATION_VECTOR
        orientation_constraint.weight = 1.0

        constraints.position_constraints = [position_constraint]
        constraints.orientation_constraints = [orientation_constraint]
        request.goal_constraints = [constraints]
        return request

    @staticmethod
    def error_details(error_code: MoveItErrorCodes) -> dict[str, Any]:
        return {
            "value": int(error_code.val),
            "message": str(error_code.message),
            "source": str(error_code.source),
        }

    def get_motion_plan(self, request: MotionPlanRequest, label: str) -> Any:
        service_request = GetMotionPlan.Request()
        service_request.motion_plan_request = request
        return self.call_service(self.plan_client, service_request, label)

    @staticmethod
    def set_time_from_start(point: Any, seconds: float) -> None:
        if not math.isfinite(seconds) or seconds < 0.0:
            raise VerificationError(f"invalid retimed waypoint time: {seconds}")
        whole_seconds = int(math.floor(seconds))
        nanoseconds = int(round((seconds - whole_seconds) * 1_000_000_000.0))
        if nanoseconds == 1_000_000_000:
            whole_seconds += 1
            nanoseconds = 0
        point.time_from_start = Duration(sec=whole_seconds, nanosec=nanoseconds)

    def retime_velocity_only(self, trajectory: Any, label: str) -> tuple[Any, dict[str, Any]]:
        """Assign safe point times using only canonical velocity maxima.

        OMPL supplies the geometric path.  This method intentionally supplies
        no velocity, acceleration, or effort arrays: the duration of each
        non-zero segment is derived from the largest per-joint position delta,
        the canonical velocity cap, the requested velocity scale, and a 2.0x
        safety factor. It therefore makes no acceleration/deceleration/jerk
        claim and does not use a guessed limit for any of those quantities.
        """
        retimed = copy.deepcopy(trajectory)
        joint_trajectory = retimed.joint_trajectory
        names = tuple(joint_trajectory.joint_names)
        if names != self.config.joint_names:
            raise VerificationError(
                f"{label} trajectory joint order must be "
                f"{list(self.config.joint_names)}, got {list(names)}"
            )
        if retimed.multi_dof_joint_trajectory.joint_names:
            raise VerificationError(f"{label} unexpectedly contains a multi-DOF trajectory")

        raw_points = list(joint_trajectory.points)
        if len(raw_points) < 2:
            raise VerificationError(f"{label} trajectory has fewer than two waypoints")
        retained_points: list[Any] = []
        previous_positions: list[float] | None = None
        dropped_duplicate_points = 0
        for index, source_point in enumerate(raw_points):
            positions = [float(value) for value in source_point.positions]
            if len(positions) != len(names):
                raise VerificationError(
                    f"{label} point {index} position vector does not match joint names"
                )
            if not all(math.isfinite(value) for value in positions):
                raise VerificationError(f"{label} point {index} has non-finite positions")
            if previous_positions is not None and max(
                abs(position - previous_positions[joint_index])
                for joint_index, position in enumerate(positions)
            ) <= 1e-12:
                dropped_duplicate_points += 1
                continue
            point = copy.deepcopy(source_point)
            point.velocities = []
            point.accelerations = []
            point.effort = []
            retained_points.append(point)
            previous_positions = positions
        if len(retained_points) < 2:
            raise VerificationError(
                f"{label} has no non-zero joint segment after duplicate waypoints are removed"
            )

        cumulative_duration_sec = 0.0
        maximum_scheduled_velocity = 0.0
        segment_durations: list[float] = []
        self.set_time_from_start(retained_points[0], cumulative_duration_sec)
        for point_index in range(1, len(retained_points)):
            previous = retained_points[point_index - 1]
            current = retained_points[point_index]
            per_joint_minimums = [
                abs(float(current.positions[joint_index]) - float(previous.positions[joint_index]))
                / (
                    self.canonical_velocity_limits[name]
                    * self.config.velocity_scaling
                )
                for joint_index, name in enumerate(names)
            ]
            unconservative_duration = max(per_joint_minimums)
            segment_duration = max(
                self.config.minimum_segment_duration_sec,
                self.config.velocity_only_retime_safety_factor
                * unconservative_duration,
            )
            if not math.isfinite(segment_duration) or segment_duration <= 0.0:
                raise VerificationError(f"{label} produced an invalid retime duration")
            cumulative_duration_sec += segment_duration
            self.set_time_from_start(current, cumulative_duration_sec)
            segment_durations.append(segment_duration)
            maximum_scheduled_velocity = max(
                maximum_scheduled_velocity,
                max(
                    abs(
                        float(current.positions[joint_index])
                        - float(previous.positions[joint_index])
                    )
                    / segment_duration
                    for joint_index, _ in enumerate(names)
                ),
            )

        joint_trajectory.points = retained_points
        return retimed, {
            "method": "cumulative_velocity_only_segment_retime",
            "canonical_velocity_limits_path": str(self.canonical_limits_path),
            "canonical_velocity_limits_rad_per_sec": {
                name: self.canonical_velocity_limits[name] for name in names
            },
            "requested_velocity_scaling": self.config.velocity_scaling,
            "safety_factor": self.config.velocity_only_retime_safety_factor,
            "minimum_segment_duration_sec": self.config.minimum_segment_duration_sec,
            "raw_waypoint_count": len(raw_points),
            "dropped_duplicate_waypoint_count": dropped_duplicate_points,
            "retimed_waypoint_count": len(retained_points),
            "segment_durations_sec": segment_durations,
            "duration_sec": cumulative_duration_sec,
            "maximum_scheduled_position_rate_rad_per_sec": maximum_scheduled_velocity,
            "velocity_arrays": "EMPTY",
            "acceleration_arrays": "EMPTY",
            "acceleration_limits_used": False,
        }

    def require_successful_plan(
        self, response: Any, label: str
    ) -> tuple[Any, dict[str, float], dict[str, Any]]:
        plan = response.motion_plan_response
        details = self.error_details(plan.error_code)
        if plan.error_code.val != SUCCESS:
            raise VerificationError(f"{label} was not planned successfully: {details}")

        retimed_trajectory, retiming = self.retime_velocity_only(plan.trajectory, label)
        trajectory = retimed_trajectory.joint_trajectory
        names = tuple(trajectory.joint_names)

        last_time_sec = -1.0
        for index, point in enumerate(trajectory.points):
            if len(point.positions) != len(names):
                raise VerificationError(
                    f"{label} point {index} position vector does not match joint names"
                )
            if not all(math.isfinite(float(value)) for value in point.positions):
                raise VerificationError(f"{label} point {index} has non-finite positions")
            time_from_start = (
                float(point.time_from_start.sec)
                + float(point.time_from_start.nanosec) / 1_000_000_000.0
            )
            if not math.isfinite(time_from_start) or time_from_start <= last_time_sec:
                raise VerificationError(
                    f"{label} waypoint times must be strictly increasing"
                )
            last_time_sec = time_from_start
        if last_time_sec <= 0.0:
            raise VerificationError(f"{label} has no positive trajectory duration")

        endpoint = {
            name: float(trajectory.points[-1].positions[index])
            for index, name in enumerate(names)
        }
        details["waypoint_count"] = len(trajectory.points)
        details["duration_sec"] = last_time_sec
        details["retiming"] = retiming
        return retimed_trajectory, endpoint, details

    def execute_trajectory(self, trajectory: Any, label: str) -> dict[str, Any]:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        goal.controller_names = [CONTROLLER_NAME]
        send_future = self.execute_client.send_goal_async(goal)
        self._spin_until(
            send_future.done,
            f"goal acceptance for {label}",
            self.config.service_call_timeout_sec,
        )
        try:
            goal_handle = send_future.result()
        except Exception as exc:  # noqa: BLE001 - surface a ROS future failure
            raise VerificationError(f"could not send {label} for execution: {exc}") from exc
        if goal_handle is None or not goal_handle.accepted:
            raise VerificationError(f"MoveIt did not accept {label} for execution")

        result_future = goal_handle.get_result_async()
        self._spin_until(
            result_future.done,
            f"execution result for {label}",
            self.config.action_result_timeout_sec,
        )
        try:
            wrapped_result = result_future.result()
        except Exception as exc:  # noqa: BLE001 - surface a ROS future failure
            raise VerificationError(f"could not receive execution result for {label}: {exc}") from exc
        if wrapped_result is None:
            raise VerificationError(f"MoveIt returned no execution result for {label}")
        details = self.error_details(wrapped_result.result.error_code)
        details["action_status"] = int(wrapped_result.status)
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise VerificationError(f"{label} action status was not SUCCEEDED: {details}")
        if wrapped_result.result.error_code.val != SUCCESS:
            raise VerificationError(f"{label} execution failed: {details}")
        return details

    def observe_endpoint(
        self, expected: dict[str, float], label: str
    ) -> dict[str, float]:
        start_sequence = self.latest_joint_sequence
        last_sequence = start_sequence
        matching_samples = 0
        observed: dict[str, float] | None = None
        deadline = self._bounded_deadline(
            self.config.endpoint_observation_timeout_sec,
            f"observed endpoint for {label}",
        )
        while time.monotonic() < deadline:
            rclpy.spin_once(
                self,
                timeout_sec=max(0.0, min(0.1, deadline - time.monotonic())),
            )
            if self.latest_joint_sequence == last_sequence:
                continue
            last_sequence = self.latest_joint_sequence
            assert self.latest_joint_state is not None
            observed = dict(self.latest_joint_state)
            if map_distance(observed, expected) <= self.config.observed_joint_tolerance_rad:
                matching_samples += 1
                if matching_samples >= self.config.endpoint_samples:
                    return observed
            else:
                matching_samples = 0
        raise VerificationError(
            f"{label} did not reach its planned endpoint for "
            f"{self.config.endpoint_samples} fresh joint-state samples; "
            f"last_observed={observed}, expected={expected}"
        )

    def compute_fk(self, positions: dict[str, float], label: str) -> PoseStamped:
        request = GetPositionFK.Request()
        request.header.frame_id = self.config.base_frame
        request.fk_link_names = [self.config.tool_link]
        request.robot_state = self.robot_state(positions)
        response = self.call_service(self.fk_client, request, label)
        details = self.error_details(response.error_code)
        if response.error_code.val != SUCCESS:
            raise VerificationError(f"{label} failed: {details}")
        if len(response.fk_link_names) != len(response.pose_stamped):
            raise VerificationError(f"{label} returned mismatched FK links and poses")
        try:
            index = list(response.fk_link_names).index(self.config.tool_link)
        except ValueError as exc:
            raise VerificationError(
                f"{label} did not return {self.config.tool_link}: {response.fk_link_names}"
            ) from exc
        pose = copy.deepcopy(response.pose_stamped[index])
        require_finite_pose(pose, label, self.config.base_frame)
        return pose

    def state_validity(
        self, positions: dict[str, float], label: str
    ) -> tuple[bool, dict[str, Any]]:
        request = GetStateValidity.Request()
        request.robot_state = self.robot_state(positions)
        request.group_name = self.config.group_name
        response = self.call_service(self.validity_client, request, label)
        contacts = [
            {
                "body_1": str(contact.contact_body_1),
                "body_2": str(contact.contact_body_2),
                "depth": float(contact.depth),
            }
            for contact in response.contacts
        ]
        return bool(response.valid), {"valid": bool(response.valid), "contacts": contacts}

    def collision_object_ids(self) -> list[str]:
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        response = self.call_service(
            self.scene_query_client, request, "read applied world collision objects"
        )
        return [str(object_.id) for object_ in response.scene.world.collision_objects]

    def apply_collision_object(self, target: PoseStamped) -> list[str]:
        require_finite_pose(
            target, "collision-object target pose", self.config.base_frame
        )
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(self.config.collision_box_dimensions_m)

        collision_object = CollisionObject()
        collision_object.header.frame_id = self.config.base_frame
        collision_object.id = self.config.collision_object_id
        collision_object.operation = CollisionObject.ADD
        collision_object.primitives = [primitive]
        collision_object.primitive_poses = [copy.deepcopy(target.pose)]

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [collision_object]
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self.call_service(
            self.scene_client, request, "apply FK-centred collision object"
        )
        if not response.success:
            raise VerificationError("MoveIt did not apply the collision object")
        object_ids = self.collision_object_ids()
        if self.config.collision_object_id not in object_ids:
            raise VerificationError(
                "MoveIt did not report the applied collision object in its planning scene"
            )
        return object_ids

    def remove_collision_object(self) -> list[str] | None:
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.config.base_frame
        collision_object.id = self.config.collision_object_id
        collision_object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [collision_object]
        request = ApplyPlanningScene.Request()
        request.scene = scene
        try:
            response = self.call_service(
                self.scene_client, request, "remove temporary collision object"
            )
            if not response.success:
                return None
            object_ids = self.collision_object_ids()
        except VerificationError:
            return None
        if self.config.collision_object_id in object_ids:
            return None
        return object_ids

    def choose_joint_target(self, current: dict[str, float]) -> tuple[str, dict[str, float]]:
        candidates = (
            ("joint_execution", self.config.joint_execution),
            ("joint_alternate", self.config.joint_alternate),
        )
        name, selected = max(candidates, key=lambda item: map_distance(current, item[1]))
        if map_distance(current, selected) < self.config.minimum_joint_motion_rad:
            raise VerificationError(
                "both reviewed joint targets are too close to the live start state"
            )
        return name, dict(selected)

    def require_meaningful_motion(
        self, before: dict[str, float], after: dict[str, float], label: str
    ) -> float:
        delta = map_distance(before, after)
        if delta < self.config.minimum_joint_motion_rad:
            raise VerificationError(
                f"{label} did not produce enough observed joint motion: {delta} rad"
            )
        return delta

    def require_observed_home(
        self, observed: dict[str, float], label: str
    ) -> float:
        deviation = map_distance(observed, self.config.home)
        if deviation > self.config.observed_home_tolerance_rad:
            raise VerificationError(
                f"{label} is not within the configured HOME tolerance: "
                f"{deviation} rad > {self.config.observed_home_tolerance_rad} rad"
            )
        return deviation


def default_smoke_path() -> Path:
    return Path(get_package_share_directory("abb_lab_moveit_config")) / "config" / (
        "irb1200_moveit_smoke.yaml"
    )


def default_canonical_limits_path() -> Path:
    return Path(get_package_share_directory("abb_lab_description")) / "config" / (
        "irb1200_5_90_control_limits.yaml"
    )


def write_evidence(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def require_non_success_collision_rejection(
    details: dict[str, Any], label: str
) -> None:
    """Accept any non-success planner result after explicit collision proof.

    OMPL in Jazzy can report its generic ``FAILURE`` (99999) when a goal region
    contains no valid states, rather than a MoveIt-specific GOAL_IN_COLLISION
    code.  The caller has already synchronously proved that the exact target is
    invalid in the applied scene and that its distinct start state is valid, so
    binding this evidence to one implementation-specific error value would be
    less reliable than requiring non-success plus those preceding facts.
    """
    if int(details["value"]) == SUCCESS:
        raise VerificationError(
            f"{label} unexpectedly succeeded despite the proven collision target: {details}"
        )


def run_verification(
    config: SmokeConfig,
    canonical_velocity_limits: dict[str, float],
    canonical_limits_path: Path,
    timeout_sec: float,
) -> dict[str, Any]:
    verifier = MoveGroupRuntimeVerifier(
        config, canonical_velocity_limits, canonical_limits_path, timeout_sec
    )
    evidence: dict[str, Any] = {
        "schema_version": 1,
        "scope": "SIMULATION_ONLY",
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "timeout_sec": timeout_sec,
        "robot": {
            "group_name": config.group_name,
            "base_frame": config.base_frame,
            "tool_link": config.tool_link,
            "joint_names": list(config.joint_names),
        },
        "velocity_only_retiming_contract": {
            "canonical_velocity_limits_path": str(canonical_limits_path),
            "canonical_velocity_limits_rad_per_sec": map_as_json(
                canonical_velocity_limits
            ),
            "requested_velocity_scaling": config.velocity_scaling,
            "safety_factor": config.velocity_only_retime_safety_factor,
            "minimum_segment_duration_sec": config.minimum_segment_duration_sec,
            "acceleration_limits_used": False,
        },
        "status": {
            "moveit_joint_execution": "NOT_RUN",
            "moveit_pose_execution": "NOT_RUN",
            "moveit_home_pose_home": "NOT_RUN",
            "moveit_collision_rejection": "NOT_RUN",
        },
    }
    collision_added = False
    try:
        verifier.wait_for_endpoints()
        initial = verifier.wait_for_joint_state()
        initial_home_deviation = verifier.require_observed_home(
            initial, "initial neutral HOME observation"
        )
        initial_home_valid, initial_home_validity = verifier.state_validity(
            initial, "initial HOME state validity before normal execution"
        )
        if not initial_home_valid:
            raise VerificationError("initial neutral HOME state is invalid")
        evidence["initial_home"] = {
            "requested_home": map_as_json(config.home),
            "observed_home": map_as_json(initial),
            "max_joint_deviation_rad": initial_home_deviation,
            "tolerance_rad": config.observed_home_tolerance_rad,
            "state_validity_before_collision": initial_home_validity,
        }

        target_name, joint_target = verifier.choose_joint_target(initial)
        joint_response = verifier.get_motion_plan(
            verifier.build_joint_request(initial, joint_target), "joint-goal motion plan"
        )
        joint_trajectory, joint_endpoint, joint_plan = verifier.require_successful_plan(
            joint_response, "joint-goal motion plan"
        )
        joint_execution = verifier.execute_trajectory(
            joint_trajectory, "joint-goal trajectory"
        )
        joint_observed = verifier.observe_endpoint(joint_endpoint, "joint-goal trajectory")
        joint_motion = verifier.require_meaningful_motion(
            initial, joint_observed, "joint-goal trajectory"
        )
        joint_endpoint_valid, joint_endpoint_validity = verifier.state_validity(
            joint_observed, "joint-goal endpoint state validity before collision object"
        )
        if not joint_endpoint_valid:
            raise VerificationError(
                "normal joint-goal endpoint is invalid before the collision object"
            )
        joint_return_response = verifier.get_motion_plan(
            verifier.build_joint_request(joint_observed, config.home),
            "joint-goal return-HOME motion plan",
        )
        (
            joint_return_trajectory,
            joint_return_endpoint,
            joint_return_plan,
        ) = verifier.require_successful_plan(
            joint_return_response, "joint-goal return-HOME motion plan"
        )
        joint_return_execution = verifier.execute_trajectory(
            joint_return_trajectory, "joint-goal return-HOME trajectory"
        )
        joint_home_observed = verifier.observe_endpoint(
            joint_return_endpoint, "joint-goal return-HOME trajectory"
        )
        joint_return_motion = verifier.require_meaningful_motion(
            joint_observed, joint_home_observed, "joint-goal return-HOME trajectory"
        )
        joint_home_deviation = verifier.require_observed_home(
            joint_home_observed, "HOME observation after joint-goal trajectory"
        )
        joint_home_valid, joint_home_validity = verifier.state_validity(
            joint_home_observed, "HOME state validity after joint-goal return"
        )
        if not joint_home_valid:
            raise VerificationError("HOME state after joint-goal return is invalid")
        evidence["joint_execution"] = {
            "target_name": target_name,
            "requested_target": map_as_json(joint_target),
            "planned_endpoint": map_as_json(joint_endpoint),
            "observed_endpoint": map_as_json(joint_observed),
            "observed_max_joint_delta_rad": joint_motion,
            "plan": joint_plan,
            "execution": joint_execution,
            "state_validity_before_collision": joint_endpoint_validity,
            "return_home": {
                "requested_home": map_as_json(config.home),
                "planned_endpoint": map_as_json(joint_return_endpoint),
                "observed_endpoint": map_as_json(joint_home_observed),
                "observed_max_joint_delta_rad": joint_return_motion,
                "max_home_deviation_rad": joint_home_deviation,
                "tolerance_rad": config.observed_home_tolerance_rad,
                "plan": joint_return_plan,
                "execution": joint_return_execution,
                "state_validity_before_collision": joint_home_validity,
            },
        }
        evidence["status"]["moveit_joint_execution"] = "PASS"

        seed_valid_before_scene, seed_validity = verifier.state_validity(
            config.fk_pose_seed, "FK pose seed state validity before collision object"
        )
        if not seed_valid_before_scene:
            raise VerificationError(
                "FK pose seed is already invalid before applying the collision object"
            )
        fk_target = verifier.compute_fk(config.fk_pose_seed, "FK-derived Cartesian target")
        pose_response = verifier.get_motion_plan(
            verifier.build_pose_request(joint_home_observed, fk_target),
            "FK-derived pose motion plan",
        )
        pose_trajectory, pose_endpoint, pose_plan = verifier.require_successful_plan(
            pose_response, "FK-derived pose motion plan"
        )
        pose_execution = verifier.execute_trajectory(
            pose_trajectory, "FK-derived pose trajectory"
        )
        pose_observed = verifier.observe_endpoint(pose_endpoint, "FK-derived pose trajectory")
        pose_motion = verifier.require_meaningful_motion(
            joint_home_observed, pose_observed, "FK-derived pose trajectory"
        )
        pose_endpoint_valid, pose_endpoint_validity = verifier.state_validity(
            pose_observed, "pose endpoint state validity before collision object"
        )
        if not pose_endpoint_valid:
            raise VerificationError(
                "normal FK-derived pose endpoint is invalid before the collision object"
            )
        observed_pose = verifier.compute_fk(
            pose_observed, "FK of observed pose-execution endpoint"
        )
        cartesian_error = position_error(observed_pose.pose, fk_target.pose)
        orientation_error = quaternion_angular_error(observed_pose.pose, fk_target.pose)
        if cartesian_error > config.observed_pose_position_tolerance_m:
            raise VerificationError(
                "executed pose endpoint missed its FK-derived Cartesian target: "
                f"{cartesian_error} m"
            )
        if orientation_error > config.observed_pose_orientation_tolerance_rad:
            raise VerificationError(
                "executed pose endpoint missed its FK-derived orientation target: "
                f"{orientation_error} rad"
            )
        pose_return_response = verifier.get_motion_plan(
            verifier.build_joint_request(pose_observed, config.home),
            "FK-derived pose return-HOME motion plan",
        )
        (
            pose_return_trajectory,
            pose_return_endpoint,
            pose_return_plan,
        ) = verifier.require_successful_plan(
            pose_return_response, "FK-derived pose return-HOME motion plan"
        )
        pose_return_execution = verifier.execute_trajectory(
            pose_return_trajectory, "FK-derived pose return-HOME trajectory"
        )
        post_pose_home_observed = verifier.observe_endpoint(
            pose_return_endpoint, "FK-derived pose return-HOME trajectory"
        )
        pose_return_motion = verifier.require_meaningful_motion(
            pose_observed,
            post_pose_home_observed,
            "FK-derived pose return-HOME trajectory",
        )
        post_pose_home_deviation = verifier.require_observed_home(
            post_pose_home_observed, "HOME observation after FK-derived pose trajectory"
        )
        post_pose_home_valid, post_pose_home_validity_before_scene = verifier.state_validity(
            post_pose_home_observed, "HOME state validity after FK-derived pose return"
        )
        if not post_pose_home_valid:
            raise VerificationError("HOME state after FK-derived pose return is invalid")
        evidence["pose_execution"] = {
            "fk_seed_joint_state": map_as_json(config.fk_pose_seed),
            "fk_target": pose_as_json(fk_target),
            "planned_endpoint": map_as_json(pose_endpoint),
            "observed_endpoint": map_as_json(pose_observed),
            "observed_endpoint_fk": pose_as_json(observed_pose),
            "observed_max_joint_delta_rad": pose_motion,
            "position_error_m": cartesian_error,
            "orientation_error_rad": orientation_error,
            "plan": pose_plan,
            "execution": pose_execution,
            "state_validity_before_collision": pose_endpoint_validity,
            "return_home": {
                "requested_home": map_as_json(config.home),
                "planned_endpoint": map_as_json(pose_return_endpoint),
                "observed_endpoint": map_as_json(post_pose_home_observed),
                "observed_max_joint_delta_rad": pose_return_motion,
                "max_home_deviation_rad": post_pose_home_deviation,
                "tolerance_rad": config.observed_home_tolerance_rad,
                "plan": pose_return_plan,
                "execution": pose_return_execution,
                "state_validity_before_collision": post_pose_home_validity_before_scene,
            },
        }
        evidence["status"]["moveit_pose_execution"] = "PASS"
        evidence["home_pose_home"] = {
            "route": "HOME -> FK-derived tool0 pose -> HOME",
            "initial_home": {
                "requested_home": map_as_json(config.home),
                "observed_home": map_as_json(initial),
                "max_home_deviation_rad": initial_home_deviation,
                "tolerance_rad": config.observed_home_tolerance_rad,
                "state_validity_before_collision": initial_home_validity,
            },
            "specified_pose": {
                "fk_seed_joint_state": map_as_json(config.fk_pose_seed),
                "fk_target": pose_as_json(fk_target),
                "observed_endpoint": map_as_json(pose_observed),
                "observed_endpoint_fk": pose_as_json(observed_pose),
                "position_error_m": cartesian_error,
                "orientation_error_rad": orientation_error,
                "state_validity_before_collision": pose_endpoint_validity,
            },
            "return_home": {
                "requested_home": map_as_json(config.home),
                "observed_home": map_as_json(post_pose_home_observed),
                "max_home_deviation_rad": post_pose_home_deviation,
                "tolerance_rad": config.observed_home_tolerance_rad,
                "state_validity_before_collision": post_pose_home_validity_before_scene,
            },
        }
        evidence["status"]["moveit_home_pose_home"] = "PASS"

        object_ids_after_apply = verifier.apply_collision_object(fk_target)
        collision_added = True
        seed_valid_after_scene, seed_validity_after_scene = verifier.state_validity(
            config.fk_pose_seed, "FK pose seed state validity with collision object"
        )
        if seed_valid_after_scene:
            raise VerificationError(
                "the applied collision object did not invalidate the FK pose seed"
            )
        post_pose_home_valid_after_scene, post_pose_home_validity_after_scene = verifier.state_validity(
            post_pose_home_observed, "HOME state validity with collision object"
        )
        if not post_pose_home_valid_after_scene:
            raise VerificationError(
                "collision object also invalidated the separated HOME state"
            )

        rejected_pose_response = verifier.get_motion_plan(
            verifier.build_pose_request(post_pose_home_observed, fk_target),
            "pose plan into an explicit collision object",
        )
        rejected_pose_details = verifier.error_details(
            rejected_pose_response.motion_plan_response.error_code
        )
        require_non_success_collision_rejection(
            rejected_pose_details, "pose collision plan"
        )

        rejected_joint_response = verifier.get_motion_plan(
            verifier.build_joint_request(post_pose_home_observed, config.fk_pose_seed),
            "joint plan into an explicit collision object",
        )
        rejected_joint_details = verifier.error_details(
            rejected_joint_response.motion_plan_response.error_code
        )
        require_non_success_collision_rejection(
            rejected_joint_details, "joint collision plan"
        )
        object_ids_after_removal = verifier.remove_collision_object()
        if object_ids_after_removal is None:
            raise VerificationError(
                "MoveIt did not remove the temporary collision object after the proof"
            )
        collision_added = False
        evidence["collision_rejection"] = {
            "temporary_object_id": config.collision_object_id,
            "temporary_box_dimensions_m": list(config.collision_box_dimensions_m),
            "world_object_ids_after_apply": object_ids_after_apply,
            "world_object_ids_after_removal": object_ids_after_removal,
            "fk_seed_valid_before_scene": seed_validity,
            "fk_seed_valid_after_scene": seed_validity_after_scene,
            "post_pose_home": {
                "requested_home": map_as_json(config.home),
                "observed_home": map_as_json(post_pose_home_observed),
                "max_home_deviation_rad": post_pose_home_deviation,
                "tolerance_rad": config.observed_home_tolerance_rad,
                "state_validity_before_scene": post_pose_home_validity_before_scene,
                "state_validity_after_scene": post_pose_home_validity_after_scene,
            },
            "rejected_pose_plan": rejected_pose_details,
            "rejected_joint_plan": rejected_joint_details,
            "temporary_collision_object_removed": True,
        }
        evidence["temporary_collision_object_removed"] = True
        evidence["status"]["moveit_collision_rejection"] = "PASS"
        return evidence
    finally:
        if collision_added:
            evidence["temporary_collision_object_removed"] = (
                verifier.remove_collision_object() is not None
            )
        evidence["joint_state_frames"] = {
            "valid": verifier.valid_joint_state_frames,
            "invalid": verifier.invalid_joint_state_frames,
            "last_invalid_error": verifier.last_joint_state_error,
        }
        verifier.destroy_node()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--timeout",
        required=True,
        type=float,
        help="finite wall-clock bound for all ROS calls and observations, in seconds",
    )
    parser.add_argument(
        "--evidence",
        required=True,
        type=Path,
        help="path for a JSON document containing the observed live evidence",
    )
    parser.add_argument(
        "--smoke-config",
        type=Path,
        default=None,
        help="optional installed irb1200_moveit_smoke.yaml override",
    )
    parser.add_argument(
        "--canonical-limits",
        type=Path,
        default=None,
        help="optional project-authoritative velocity-limit YAML override",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    evidence: dict[str, Any] = {
        "schema_version": 1,
        "scope": "SIMULATION_ONLY",
        "status": {
            "moveit_joint_execution": "NOT_RUN",
            "moveit_pose_execution": "NOT_RUN",
            "moveit_home_pose_home": "NOT_RUN",
            "moveit_collision_rejection": "NOT_RUN",
        },
    }
    try:
        timeout_sec = require_finite(args.timeout, "--timeout", positive=True)
        config_path = args.smoke_config or default_smoke_path()
        config = load_smoke_config(config_path)
        canonical_limits_path = args.canonical_limits or default_canonical_limits_path()
        canonical_velocity_limits = load_canonical_velocity_limits(
            canonical_limits_path, config.joint_names
        )
        rclpy.init(args=None)
        try:
            evidence = run_verification(
                config,
                canonical_velocity_limits,
                canonical_limits_path,
                timeout_sec,
            )
        finally:
            if rclpy.ok():
                rclpy.shutdown()
        write_evidence(args.evidence, evidence)
    except (VerificationError, OSError, ValueError) as exc:
        evidence["error"] = str(exc)
        try:
            write_evidence(args.evidence, evidence)
        except OSError as evidence_exc:
            print(f"ERROR: could not write evidence: {evidence_exc}", file=sys.stderr)
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print("MOVEIT_JOINT_EXECUTION=PASS")
    print("MOVEIT_POSE_EXECUTION=PASS")
    print("MOVEIT_HOME_POSE_HOME=PASS")
    print("MOVEIT_COLLISION_REJECTION=PASS")
    print(f"MOVEIT_RUNTIME_EVIDENCE={args.evidence}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
