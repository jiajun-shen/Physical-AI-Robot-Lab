#!/usr/bin/env python3
"""Bounded, evidence-producing runtime verification for the P3.1 gripper.

This client connects to the live project-owned Gazebo/ros2_control stack.  It
does not use a mock controller and it deliberately makes no contact, force,
attachment, or grasp-success claim.  A successful close only proves that the
actuated finger reached its commanded clearance in the empty laboratory.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Callable

from action_msgs.msg import GoalStatus
from control_msgs.action import ParallelGripperCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
import yaml


RIGHT_FINGER = "two_finger_right_joint"
LEFT_FINGER = "two_finger_left_joint"
ARM_JOINTS = tuple(f"joint_{index}" for index in range(1, 7))
REQUIRED_JOINTS = ARM_JOINTS + (RIGHT_FINGER, LEFT_FINGER)
ACTION_NAME = "/two_finger_gripper_controller/gripper_cmd"


class VerificationError(RuntimeError):
    """Raised when a required real-runtime observation is absent."""


@dataclass(frozen=True)
class GripperProfile:
    """Validated P3.1 operating values from the project-owned YAML profile."""

    minimum_position_m: float
    maximum_position_m: float
    maximum_velocity_m_per_s: float
    initial_position_m: float
    close_position_m: float
    open_position_m: float
    endpoint_tolerance_m: float = 0.003
    mirror_tolerance_m: float = 0.003
    minimum_action_motion_m: float = 0.007
    initial_sample_count: int = 5
    action_timeout_sec: float = 15.0
    settle_timeout_sec: float = 10.0


@dataclass(frozen=True)
class JointSample:
    """A finite, complete, time-ordered observation from /joint_states."""

    sequence: int
    stamp_ns: int
    positions: dict[str, float]
    velocities: dict[str, float]


def require_mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise VerificationError(f"{label} must be a YAML mapping")
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


def load_profile(path: Path) -> GripperProfile:
    """Load the bounded project profile; reject invented runtime values."""

    if not path.is_file():
        raise VerificationError(f"gripper profile does not exist: {path}")
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise VerificationError(f"could not parse gripper profile {path}") from exc
    root = require_mapping(document, str(path))
    if root.get("schema_version") != 1:
        raise VerificationError("gripper profile schema_version must be 1")
    if root.get("scope") != "SIMULATION_ONLY":
        raise VerificationError("gripper profile scope must be SIMULATION_ONLY")
    limits = require_mapping(root.get("joint_limits"), "joint_limits")
    right = require_mapping(limits.get(RIGHT_FINGER), f"joint_limits.{RIGHT_FINGER}")
    if right.get("has_position_limits") is not True:
        raise VerificationError("right finger must have enabled position limits")
    if right.get("has_velocity_limits") is not True:
        raise VerificationError("right finger must have enabled velocity limits")
    minimum = require_finite(right.get("min_position"), "right min_position")
    maximum = require_finite(right.get("max_position"), "right max_position", positive=True)
    velocity = require_finite(right.get("max_velocity"), "right max_velocity", positive=True)
    if minimum < 0.0 or maximum <= minimum:
        raise VerificationError("right finger position limits are invalid")
    initial = require_finite(
        require_mapping(root.get("initial_positions"), "initial_positions").get(RIGHT_FINGER),
        "initial right finger position",
    )
    commands = require_mapping(root.get("safe_commands"), "safe_commands")
    close = require_finite(
        commands.get("closed_clearance_position"), "closed_clearance_position"
    )
    open_position = require_finite(commands.get("open_position"), "open_position")
    for name, value in (
        ("initial right finger position", initial),
        ("closed_clearance_position", close),
        ("open_position", open_position),
    ):
        if not minimum <= value <= maximum:
            raise VerificationError(
                f"{name} {value} is outside [{minimum}, {maximum}]"
            )
    if not close < initial < open_position:
        raise VerificationError(
            "profile must provide a non-degenerate close < initial < open sequence"
        )
    return GripperProfile(
        minimum_position_m=minimum,
        maximum_position_m=maximum,
        maximum_velocity_m_per_s=velocity,
        initial_position_m=initial,
        close_position_m=close,
        open_position_m=open_position,
    )


def map_joint_state(message: JointState) -> tuple[dict[str, float], dict[str, float], int]:
    """Extract exactly the live values required for P3.1 evidence."""

    if len(message.name) != len(message.position):
        raise VerificationError("/joint_states name and position arrays have different sizes")
    names = list(message.name)
    if len(set(names)) != len(names):
        raise VerificationError("/joint_states contains duplicate joint names")
    positions_by_name = dict(zip(names, message.position, strict=True))
    missing = sorted(set(REQUIRED_JOINTS) - set(positions_by_name))
    if missing:
        raise VerificationError(f"/joint_states is missing required joints: {missing}")
    positions: dict[str, float] = {}
    for name in REQUIRED_JOINTS:
        positions[name] = require_finite(
            positions_by_name[name], f"/joint_states position for {name}"
        )

    if len(message.velocity) != len(message.name):
        raise VerificationError(
            "/joint_states must expose one finite velocity for every named joint"
        )
    velocities_by_name = dict(zip(names, message.velocity, strict=True))
    velocities: dict[str, float] = {}
    for name in REQUIRED_JOINTS:
        velocities[name] = require_finite(
            velocities_by_name[name], f"/joint_states velocity for {name}"
        )

    stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
    if stamp_ns <= 0:
        raise VerificationError("/joint_states has a non-positive simulation timestamp")
    return positions, velocities, stamp_ns


class GripperRuntimeVerifier(Node):
    """A wall-clock-bounded client for the real gripper action and state feed."""

    def __init__(self, profile: GripperProfile, timeout_sec: float) -> None:
        super().__init__(
            "abb_lab_gripper_runtime_verifier",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.profile = profile
        self.deadline = time.monotonic() + timeout_sec
        self.action_client = ActionClient(self, ParallelGripperCommand, ACTION_NAME)
        self.samples: list[JointSample] = []
        self.invalid_frames = 0
        self.last_joint_state_error = ""
        self._last_stamp_ns = 0
        self._sequence = 0
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, qos_profile_sensor_data
        )

    def remaining(self) -> float:
        return self.deadline - time.monotonic()

    def bounded_deadline(self, maximum_sec: float, label: str) -> float:
        if maximum_sec <= 0.0:
            raise VerificationError(f"{label} timeout must be positive")
        if self.remaining() <= 0.0:
            raise VerificationError(f"global verifier timeout expired before {label}")
        return min(self.deadline, time.monotonic() + min(maximum_sec, self.remaining()))

    def spin_until(
        self, predicate: Callable[[], bool], label: str, maximum_sec: float
    ) -> None:
        deadline = self.bounded_deadline(maximum_sec, label)
        while time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self, timeout_sec=max(0.0, min(0.1, deadline - time.monotonic())))
        if not predicate():
            raise VerificationError(f"timed out waiting for {label}")

    def _on_joint_state(self, message: JointState) -> None:
        try:
            positions, velocities, stamp_ns = map_joint_state(message)
            if stamp_ns <= self._last_stamp_ns:
                raise VerificationError("/joint_states timestamps are not strictly increasing")
            right = positions[RIGHT_FINGER]
            left = positions[LEFT_FINGER]
            tolerance = self.profile.endpoint_tolerance_m
            if not (
                self.profile.minimum_position_m - tolerance
                <= right
                <= self.profile.maximum_position_m + tolerance
            ):
                raise VerificationError(f"right finger {right} is outside its safe position bound")
            if not (
                -self.profile.maximum_position_m - tolerance
                <= left
                <= -self.profile.minimum_position_m + tolerance
            ):
                raise VerificationError(f"left finger {left} is outside its mirrored bound")
            if abs(left + right) > self.profile.mirror_tolerance_m:
                raise VerificationError(
                    f"finger states are not mirrored: left + right = {left + right}"
                )
        except VerificationError as exc:
            self.invalid_frames += 1
            self.last_joint_state_error = str(exc)
            return
        self._last_stamp_ns = stamp_ns
        self._sequence += 1
        self.samples.append(
            JointSample(self._sequence, stamp_ns, positions, velocities)
        )
        if len(self.samples) > 512:
            del self.samples[:-256]

    def wait_for_action_server(self) -> None:
        deadline = self.bounded_deadline(self.profile.action_timeout_sec, ACTION_NAME)
        while time.monotonic() < deadline:
            if self.action_client.wait_for_server(
                timeout_sec=max(0.0, min(0.2, deadline - time.monotonic()))
            ):
                return
        raise VerificationError(f"action server is not ready: {ACTION_NAME}")

    def wait_for_initial_samples(self) -> JointSample:
        try:
            self.spin_until(
                lambda: len(self.samples) >= self.profile.initial_sample_count,
                f"{self.profile.initial_sample_count} valid /joint_states samples",
                self.profile.settle_timeout_sec,
            )
        except VerificationError as exc:
            if self.invalid_frames:
                raise VerificationError(
                    f"{exc}; observed {self.invalid_frames} invalid /joint_states "
                    f"frames (last: {self.last_joint_state_error})"
                ) from exc
            raise
        if self.invalid_frames:
            raise VerificationError(
                "invalid /joint_states frames were observed before the first gripper action: "
                f"{self.last_joint_state_error}"
            )
        return self.samples[-1]

    def _await_future(self, future: Any, label: str, timeout_sec: float) -> Any:
        self.spin_until(future.done, label, timeout_sec)
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001 - preserve ROS future context
            raise VerificationError(f"{label} failed: {exc}") from exc
        if result is None:
            raise VerificationError(f"{label} returned no result")
        return result

    def command(self, label: str, target_m: float) -> dict[str, Any]:
        """Send one real position action and require endpoint state evidence."""

        before = self.samples[-1]
        goal = ParallelGripperCommand.Goal()
        # The installed controller requires exactly one position.  Velocity and
        # effort arrays stay empty because this profile has position-only
        # hardware; treating those optional goal fields as force/velocity proof
        # would be misleading.
        goal.command.name = [RIGHT_FINGER]
        goal.command.position = [target_m]
        goal.command.velocity = []
        goal.command.effort = []
        goal_handle = self._await_future(
            self.action_client.send_goal_async(goal),
            f"goal acceptance for {label}",
            self.profile.action_timeout_sec,
        )
        if not goal_handle.accepted:
            raise VerificationError(f"gripper action goal was rejected: {label}")
        wrapped_result = self._await_future(
            goal_handle.get_result_async(),
            f"action result for {label}",
            self.profile.action_timeout_sec,
        )
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise VerificationError(
                f"gripper action {label} did not succeed: status={wrapped_result.status}"
            )
        action_result = wrapped_result.result
        if not action_result.reached_goal or action_result.stalled:
            raise VerificationError(
                f"gripper action {label} did not reach a free-space goal: "
                f"reached_goal={action_result.reached_goal}, stalled={action_result.stalled}"
            )

        self.spin_until(
            lambda: bool(self.samples)
            and self.samples[-1].sequence > before.sequence
            and abs(self.samples[-1].positions[RIGHT_FINGER] - target_m)
            <= self.profile.endpoint_tolerance_m,
            f"observed {label} endpoint",
            self.profile.settle_timeout_sec,
        )
        after = self.samples[-1]
        segment = [sample for sample in self.samples if sample.sequence > before.sequence]
        motion = max(
            abs(sample.positions[RIGHT_FINGER] - before.positions[RIGHT_FINGER])
            for sample in segment
        )
        if motion < self.profile.minimum_action_motion_m:
            raise VerificationError(
                f"gripper action {label} observed only {motion} m motion; "
                f"requires at least {self.profile.minimum_action_motion_m} m"
            )
        return {
            "target_position_m": target_m,
            "start_position_m": before.positions[RIGHT_FINGER],
            "observed_position_m": after.positions[RIGHT_FINGER],
            "observed_left_position_m": after.positions[LEFT_FINGER],
            "observed_motion_m": motion,
            "result_status": int(wrapped_result.status),
            "reached_goal": bool(action_result.reached_goal),
            "stalled": bool(action_result.stalled),
            "result_state_names": list(action_result.state.name),
            "result_state_positions": list(action_result.state.position),
        }

    def verify(self) -> dict[str, Any]:
        started_at_utc = datetime.now(timezone.utc).isoformat()
        self.wait_for_action_server()
        initial = self.wait_for_initial_samples()
        initial_deviation = abs(
            initial.positions[RIGHT_FINGER] - self.profile.initial_position_m
        )
        # ``initial_value`` seeds the ros2_control state interface; it is not
        # a Gazebo pose-set command.  Physics can therefore settle a spawned
        # mechanism before the first controller command.  The callback has
        # already required the observed pose to be finite, bounded, and
        # mirrored, and the three real actions below prove physical control.
        # Keep the configured-vs-observed deviation as evidence, not a false
        # claim that ros2_control teleports a physical joint at spawn time.
        actions = {
            "open": self.command("open", self.profile.open_position_m),
            "close_clearance": self.command(
                "closed clearance", self.profile.close_position_m
            ),
            "reopen": self.command("reopen", self.profile.open_position_m),
        }
        observed_positions = [sample.positions[RIGHT_FINGER] for sample in self.samples]
        if max(observed_positions) - min(observed_positions) < self.profile.minimum_action_motion_m:
            raise VerificationError("joint-state history does not show a real open/close motion range")
        if self.invalid_frames:
            raise VerificationError(
                "invalid /joint_states frames were observed during gripper actions: "
                f"{self.last_joint_state_error}"
            )
        final = self.samples[-1]
        return {
            "schema_version": 1,
            "scope": "SIMULATION_ONLY",
            "started_at_utc": started_at_utc,
            "action_name": ACTION_NAME,
            "joint_names": list(REQUIRED_JOINTS),
            "profile": {
                "minimum_position_m": self.profile.minimum_position_m,
                "maximum_position_m": self.profile.maximum_position_m,
                "maximum_velocity_m_per_s": self.profile.maximum_velocity_m_per_s,
                "initial_position_m": self.profile.initial_position_m,
                "close_position_m": self.profile.close_position_m,
                "open_position_m": self.profile.open_position_m,
                "endpoint_tolerance_m": self.profile.endpoint_tolerance_m,
                "mirror_tolerance_m": self.profile.mirror_tolerance_m,
            },
            "initial_state": {
                "right_position_m": initial.positions[RIGHT_FINGER],
                "left_position_m": initial.positions[LEFT_FINGER],
                "initial_deviation_m": initial_deviation,
                "stamp_ns": initial.stamp_ns,
            },
            "actions": actions,
            "final_state": {
                "right_position_m": final.positions[RIGHT_FINGER],
                "left_position_m": final.positions[LEFT_FINGER],
                "stamp_ns": final.stamp_ns,
            },
            "joint_state_samples": len(self.samples),
            "joint_state_invalid_frames": self.invalid_frames,
            "status": {
                "gripper_action_server": "PASS",
                "gripper_open_close_open": "PASS",
                "gripper_mirror_state": "PASS",
                "grasp_state": "NOT_DETERMINED_NO_CONTACT_EVIDENCE",
                "object_attachment": "NOT_IMPLEMENTED",
                "force_control": "NOT_CONFIGURED",
            },
        }


def write_evidence(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--limits", required=True, type=Path)
    parser.add_argument("--evidence", required=True, type=Path)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    evidence: dict[str, Any] = {
        "schema_version": 1,
        "scope": "SIMULATION_ONLY",
        "status": {
            "gripper_action_server": "NOT_RUN",
            "gripper_open_close_open": "NOT_RUN",
            "gripper_mirror_state": "NOT_RUN",
            "grasp_state": "NOT_DETERMINED_NO_CONTACT_EVIDENCE",
        },
    }
    verifier: GripperRuntimeVerifier | None = None
    try:
        timeout_sec = require_finite(args.timeout, "--timeout", positive=True)
        profile = load_profile(args.limits)
        rclpy.init(args=None)
        verifier = GripperRuntimeVerifier(profile, timeout_sec)
        evidence = verifier.verify()
    except (VerificationError, OSError, ValueError) as exc:
        evidence["error"] = str(exc)
        try:
            write_evidence(args.evidence, evidence)
        except OSError as evidence_exc:
            print(f"ERROR: could not write evidence: {evidence_exc}", file=sys.stderr)
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if verifier is not None:
            verifier.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    write_evidence(args.evidence, evidence)
    print("GRIPPER_ACTION_SERVER=PASS")
    print("GRIPPER_OPEN_CLOSE_OPEN=PASS")
    print("GRIPPER_MIRROR_STATE=PASS")
    print("GRASP_STATE=NOT_DETERMINED_NO_CONTACT_EVIDENCE")
    print(f"GRIPPER_RUNTIME_EVIDENCE={args.evidence}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
