#!/usr/bin/env python3
"""Observe the project ABB simulation through ROS messages and saved evidence.

This verifier never fabricates samples.  A PASS is emitted only after the
required ROS messages or Gazebo pose files have been observed and validated.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import sys
import time
from typing import Any

import yaml


EXPECTED_JOINTS = [f"joint_{index}" for index in range(1, 7)]
FLOAT_PATTERN = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


def load_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise ValueError(f"required YAML does not exist: {path}")
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return document


def require_finite(value: Any, label: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} is not finite: {value!r}")
    return number


def validate_contract(
    smoke_path: Path, limits_path: Path
) -> tuple[dict[str, Any], dict[str, Any]]:
    smoke = load_yaml(smoke_path)
    limits_document = load_yaml(limits_path)
    limits = limits_document.get("joint_limits")
    if smoke.get("scope") != "SIMULATION_ONLY":
        raise ValueError("bounded smoke scope must be SIMULATION_ONLY")
    if smoke.get("joint_order") != EXPECTED_JOINTS:
        raise ValueError("bounded smoke joint_order is not joint_1..joint_6")
    if not isinstance(limits, dict) or set(limits) != set(EXPECTED_JOINTS):
        raise ValueError("canonical limits must contain exactly joint_1..joint_6")

    neutral = smoke.get("neutral_reference")
    target = smoke.get("target")
    guards = smoke.get("guards")
    if not isinstance(neutral, dict) or set(neutral) != set(EXPECTED_JOINTS):
        raise ValueError("neutral_reference must contain exactly six joints")
    if not isinstance(target, dict) or set(target) != set(EXPECTED_JOINTS):
        raise ValueError("target must contain exactly six joints")
    if not isinstance(guards, dict):
        raise ValueError("guards must be a mapping")

    margin = require_finite(guards["limit_margin_rad"], "limit_margin_rad")
    max_delta = require_finite(guards["max_delta_rad"], "max_delta_rad")
    for joint_name in EXPECTED_JOINTS:
        lower = require_finite(
            limits[joint_name]["min_position"], f"{joint_name}.min_position"
        )
        upper = require_finite(
            limits[joint_name]["max_position"], f"{joint_name}.max_position"
        )
        reference = require_finite(neutral[joint_name], f"{joint_name}.neutral")
        goal = require_finite(target[joint_name], f"{joint_name}.target")
        if lower >= upper:
            raise ValueError(f"invalid canonical limits for {joint_name}")
        if abs(goal - reference) > max_delta:
            raise ValueError(f"{joint_name} target exceeds max_delta_rad")
        if goal < lower + margin or goal > upper - margin:
            raise ValueError(f"{joint_name} target violates canonical margin")
    return smoke, limits


def import_ros() -> tuple[Any, Any, Any, Any, Any]:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Python messages are unavailable; source the Jazzy environment"
        ) from exc
    return rclpy, Node, qos_profile_sensor_data, Clock, JointState


def observe_clock(timeout_sec: float, minimum_advance_sec: float) -> None:
    rclpy, Node, qos_profile_sensor_data, Clock, _ = import_ros()
    rclpy.init(args=None)
    node = Node("abb_lab_clock_evidence")
    samples: list[float] = []

    def callback(message: Any) -> None:
        samples.append(float(message.clock.sec) + float(message.clock.nanosec) / 1e9)

    node.create_subscription(Clock, "/clock", callback, qos_profile_sensor_data)
    deadline = time.monotonic() + timeout_sec
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(
                node, timeout_sec=max(0.0, min(0.1, deadline - time.monotonic()))
            )
            if len(samples) >= 2 and samples[-1] - samples[0] >= minimum_advance_sec:
                print("CLOCK_PROGRESS=PASS")
                print(f"CLOCK_SAMPLE_COUNT={len(samples)}")
                print(f"CLOCK_ADVANCE_SEC={samples[-1] - samples[0]:.9f}")
                return
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise RuntimeError(
        f"/clock did not advance by {minimum_advance_sec} s within {timeout_sec} s"
    )


def observe_joint_state(
    phase: str,
    smoke_path: Path,
    limits_path: Path,
    timeout_sec: float,
) -> None:
    smoke, limits = validate_contract(smoke_path, limits_path)
    guards = smoke["guards"]
    if phase == "initial":
        expected = smoke["neutral_reference"]
        tolerance = require_finite(
            guards["initial_neutral_tolerance_rad"],
            "initial_neutral_tolerance_rad",
        )
        required_consecutive = int(guards.get("consecutive_initial_samples", 3))
    else:
        expected = smoke["target"]
        tolerance = require_finite(
            guards["final_position_tolerance_rad"],
            "final_position_tolerance_rad",
        )
        required_consecutive = int(guards["consecutive_target_samples"])
    if required_consecutive < 1:
        raise ValueError("consecutive sample requirement must be positive")

    rclpy, Node, qos_profile_sensor_data, _, JointState = import_ros()
    rclpy.init(args=None)
    node = Node(f"abb_lab_joint_state_{phase}_evidence")
    consecutive = 0
    valid_frames = 0
    total_frames = 0
    last_positions: dict[str, float] = {}
    last_velocities: dict[str, float] = {}
    first_stamp_ns: int | None = None
    last_stamp_ns: int | None = None
    failure: list[str] = []

    def callback(message: Any) -> None:
        nonlocal consecutive, total_frames, valid_frames, last_positions
        nonlocal last_velocities, first_stamp_ns, last_stamp_ns
        total_frames += 1
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        if last_stamp_ns is not None and stamp_ns <= last_stamp_ns:
            failure.append(
                f"joint state timestamp did not increase: {stamp_ns} <= {last_stamp_ns}"
            )
            return
        if first_stamp_ns is None:
            first_stamp_ns = stamp_ns
        last_stamp_ns = stamp_ns
        names = list(message.name)
        if len(names) != len(set(names)):
            failure.append("joint state contains duplicate names")
            return
        if set(names) != set(EXPECTED_JOINTS) or len(names) != len(EXPECTED_JOINTS):
            failure.append(f"joint names are not exactly {EXPECTED_JOINTS}: {names}")
            return
        if len(message.position) != len(names) or len(message.velocity) != len(names):
            failure.append("joint state position/velocity lengths do not match names")
            return
        positions = {name: float(message.position[index]) for index, name in enumerate(names)}
        velocities = {name: float(message.velocity[index]) for index, name in enumerate(names)}
        if not all(math.isfinite(value) for value in positions.values()):
            failure.append("joint state contains a non-finite position")
            return
        if not all(math.isfinite(value) for value in velocities.values()):
            failure.append("joint state contains a non-finite velocity")
            return
        for joint_name in EXPECTED_JOINTS:
            lower = float(limits[joint_name]["min_position"])
            upper = float(limits[joint_name]["max_position"])
            if positions[joint_name] < lower or positions[joint_name] > upper:
                failure.append(
                    f"{joint_name} observed outside canonical bounds: "
                    f"{positions[joint_name]} not in [{lower}, {upper}]"
                )
                return
            velocity_limit = float(limits[joint_name]["max_velocity"])
            if abs(velocities[joint_name]) > velocity_limit:
                failure.append(
                    f"{joint_name} observed velocity {velocities[joint_name]} "
                    f"exceeds canonical magnitude {velocity_limit}"
                )
                return
        valid_frames += 1
        last_positions = positions
        last_velocities = velocities
        if all(
            abs(positions[joint_name] - float(expected[joint_name])) <= tolerance
            for joint_name in EXPECTED_JOINTS
        ):
            consecutive += 1
        else:
            consecutive = 0

    node.create_subscription(
        JointState, "/joint_states", callback, qos_profile_sensor_data
    )
    deadline = time.monotonic() + timeout_sec
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(
                node, timeout_sec=max(0.0, min(0.1, deadline - time.monotonic()))
            )
            if failure:
                raise RuntimeError(failure[0])
            if consecutive >= required_consecutive:
                print("JOINT_STATE_OBSERVED=PASS")
                print(f"JOINT_STATE_PHASE={phase}")
                print(f"JOINT_STATE_TOTAL_FRAMES={total_frames}")
                print(f"JOINT_STATE_VALID_FRAMES={valid_frames}")
                print(f"JOINT_STATE_CONSECUTIVE_MATCHES={consecutive}")
                print(f"JOINT_STATE_FIRST_STAMP_NS={first_stamp_ns}")
                print(f"JOINT_STATE_LAST_STAMP_NS={last_stamp_ns}")
                print(
                    "JOINT_STATE_POSITION_MAP="
                    + json.dumps(last_positions, sort_keys=True, separators=(",", ":"))
                )
                print(
                    "JOINT_STATE_VELOCITY_MAP="
                    + json.dumps(last_velocities, sort_keys=True, separators=(",", ":"))
                )
                return
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise RuntimeError(
        f"joint state phase {phase!r} did not match for {required_consecutive} "
        f"consecutive frames within {timeout_sec} s; valid_frames={valid_frames}, "
        f"last_positions={last_positions}"
    )


def emit_goal(smoke_path: Path, limits_path: Path) -> None:
    smoke, _ = validate_contract(smoke_path, limits_path)
    duration = require_finite(
        smoke["guards"]["time_from_start_sec"], "time_from_start_sec"
    )
    seconds = int(duration)
    nanoseconds = int(round((duration - seconds) * 1e9))
    if nanoseconds == 1_000_000_000:
        seconds += 1
        nanoseconds = 0
    goal = {
        "trajectory": {
            "joint_names": EXPECTED_JOINTS,
            "points": [
                {
                    "positions": [
                        float(smoke["target"][joint_name])
                        for joint_name in EXPECTED_JOINTS
                    ],
                    "time_from_start": {"sec": seconds, "nanosec": nanoseconds},
                }
            ],
        }
    }
    print(json.dumps(goal, separators=(",", ":")))


def parse_numeric_list(block: str, label: str) -> list[float]:
    values: list[float] = []
    for line in block.splitlines():
        match = re.fullmatch(rf"\s*-\s*({FLOAT_PATTERN})\s*", line)
        if match is None:
            raise ValueError(f"could not parse {label} line: {line!r}")
        values.append(require_finite(match.group(1), label))
    return values


def verify_action_feedback(
    action_log_path: Path, smoke_path: Path, limits_path: Path
) -> None:
    smoke, limits = validate_contract(smoke_path, limits_path)
    if not action_log_path.is_file():
        raise ValueError(f"action evidence does not exist: {action_log_path}")
    text = action_log_path.read_text(encoding="utf-8")
    if "Goal accepted" not in text:
        raise ValueError("action evidence does not contain goal acceptance")
    if "Goal finished with status: SUCCEEDED" not in text:
        raise ValueError("action evidence does not contain SUCCEEDED status")
    if re.search(r"error_code:\s+0(?:\s|$)", text) is None:
        raise ValueError("action evidence does not contain controller error_code 0")

    joint_blocks = re.findall(
        r"(?m)^joint_names:\s*\n((?:-\s+[^\n]+\n)+)", text
    )
    feedback_joint_orders = [
        [line.removeprefix("-").strip() for line in block.splitlines()]
        for block in joint_blocks
    ]
    if not feedback_joint_orders:
        raise ValueError("action evidence contains no feedback joint_names")
    if any(order != EXPECTED_JOINTS for order in feedback_joint_orders):
        raise ValueError("action feedback joint order is not joint_1..joint_6")

    section_pattern = re.compile(
        rf"(?m)^actual:\s*\n"
        rf"\s{{2}}positions:\s*\n"
        rf"(?P<positions>(?:\s{{2}}-\s*{FLOAT_PATTERN}\s*\n)+)"
        rf"\s{{2}}velocities:\s*\n"
        rf"(?P<velocities>(?:\s{{2}}-\s*{FLOAT_PATTERN}\s*\n)+)"
    )
    frames: list[tuple[list[float], list[float]]] = []
    for match in section_pattern.finditer(text):
        positions = parse_numeric_list(match.group("positions"), "feedback position")
        velocities = parse_numeric_list(match.group("velocities"), "feedback velocity")
        if len(positions) != len(EXPECTED_JOINTS):
            raise ValueError("action feedback position vector is not six-axis")
        if len(velocities) != len(EXPECTED_JOINTS):
            raise ValueError("action feedback velocity vector is not six-axis")
        for index, joint_name in enumerate(EXPECTED_JOINTS):
            lower = float(limits[joint_name]["min_position"])
            upper = float(limits[joint_name]["max_position"])
            if positions[index] < lower or positions[index] > upper:
                raise ValueError(
                    f"feedback {joint_name} position {positions[index]} is outside "
                    f"[{lower}, {upper}]"
                )
            velocity_limit = float(limits[joint_name]["max_velocity"])
            if abs(velocities[index]) > velocity_limit:
                raise ValueError(
                    f"feedback {joint_name} velocity {velocities[index]} exceeds "
                    f"canonical magnitude {velocity_limit}"
                )
        frames.append((positions, velocities))

    minimum_frames = int(smoke["guards"]["minimum_action_feedback_samples"])
    if len(frames) < minimum_frames:
        raise ValueError(
            f"observed {len(frames)} action feedback frames; require {minimum_frames}"
        )
    if len(feedback_joint_orders) != len(frames):
        raise ValueError(
            "feedback joint-name and actual-state frame counts do not agree: "
            f"{len(feedback_joint_orders)} vs {len(frames)}"
        )

    minima = [min(frame[0][index] for frame in frames) for index in range(6)]
    maxima = [max(frame[0][index] for frame in frames) for index in range(6)]
    maximum_abs_velocity = max(abs(value) for frame in frames for value in frame[1])
    print("TRAJECTORY_FEEDBACK_BOUNDS=PASS")
    print(f"ACTION_FEEDBACK_FRAME_COUNT={len(frames)}")
    print(
        "ACTION_FEEDBACK_POSITION_MIN="
        + json.dumps(dict(zip(EXPECTED_JOINTS, minima)), separators=(",", ":"))
    )
    print(
        "ACTION_FEEDBACK_POSITION_MAX="
        + json.dumps(dict(zip(EXPECTED_JOINTS, maxima)), separators=(",", ":"))
    )
    print(f"ACTION_FEEDBACK_MAX_ABS_VELOCITY={maximum_abs_velocity:.12g}")


def parse_gz_pose(path: Path) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    if not path.is_file():
        raise ValueError(f"Gazebo pose evidence does not exist: {path}")
    text = re.sub(r"\x1b\[[0-9;]*m", "", path.read_text(encoding="utf-8"))
    triple_pattern = re.compile(
        rf"\[\s*({FLOAT_PATTERN})\s+({FLOAT_PATTERN})\s+({FLOAT_PATTERN})\s*\]"
    )
    triples = [tuple(float(value) for value in match) for match in triple_pattern.findall(text)]
    if len(triples) < 2:
        raise ValueError(f"could not parse XYZ and RPY triples from {path}")
    xyz, rpy = triples[0], triples[1]
    if not all(math.isfinite(value) for value in (*xyz, *rpy)):
        raise ValueError(f"Gazebo pose contains non-finite values: {path}")
    return xyz, rpy


def angular_distance(left: float, right: float) -> float:
    return abs((left - right + math.pi) % (2.0 * math.pi) - math.pi)


def compare_pose(
    before_path: Path,
    after_path: Path,
    translation_tolerance: float,
    rotation_tolerance: float,
) -> None:
    translation_tolerance = require_finite(
        translation_tolerance, "translation_tolerance"
    )
    rotation_tolerance = require_finite(rotation_tolerance, "rotation_tolerance")
    if translation_tolerance < 0.0 or rotation_tolerance < 0.0:
        raise ValueError("pose tolerances must be non-negative")
    before_xyz, before_rpy = parse_gz_pose(before_path)
    after_xyz, after_rpy = parse_gz_pose(after_path)
    translation_delta = math.sqrt(
        sum((after_xyz[index] - before_xyz[index]) ** 2 for index in range(3))
    )
    rotation_delta = max(
        angular_distance(after_rpy[index], before_rpy[index]) for index in range(3)
    )
    if translation_delta > translation_tolerance:
        raise RuntimeError(
            f"model translation drift {translation_delta} exceeds {translation_tolerance}"
        )
    if rotation_delta > rotation_tolerance:
        raise RuntimeError(
            f"model rotation drift {rotation_delta} exceeds {rotation_tolerance}"
        )
    print("FIXED_BASE_POSE=PASS")
    print(f"MODEL_TRANSLATION_DRIFT_M={translation_delta:.12g}")
    print(f"MODEL_ROTATION_DRIFT_RAD={rotation_delta:.12g}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    clock_parser = subparsers.add_parser("clock", help="observe advancing /clock")
    clock_parser.add_argument("--timeout", type=float, required=True)
    clock_parser.add_argument("--min-advance", type=float, required=True)

    joint_parser = subparsers.add_parser(
        "joint-state", help="observe exact six-axis JointState frames"
    )
    joint_parser.add_argument("--phase", choices=("initial", "target"), required=True)
    joint_parser.add_argument("--smoke-config", type=Path, required=True)
    joint_parser.add_argument("--limits-config", type=Path, required=True)
    joint_parser.add_argument("--timeout", type=float, required=True)

    goal_parser = subparsers.add_parser("goal", help="emit the reviewed bounded goal")
    goal_parser.add_argument("--smoke-config", type=Path, required=True)
    goal_parser.add_argument("--limits-config", type=Path, required=True)

    feedback_parser = subparsers.add_parser(
        "feedback", help="verify recorded real action feedback frames"
    )
    feedback_parser.add_argument("--action-log", type=Path, required=True)
    feedback_parser.add_argument("--smoke-config", type=Path, required=True)
    feedback_parser.add_argument("--limits-config", type=Path, required=True)

    pose_parser = subparsers.add_parser(
        "pose", help="compare two observed gz model pose responses"
    )
    pose_parser.add_argument("--before", type=Path, required=True)
    pose_parser.add_argument("--after", type=Path, required=True)
    pose_parser.add_argument("--translation-tolerance", type=float, required=True)
    pose_parser.add_argument("--rotation-tolerance", type=float, required=True)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        if args.command == "clock":
            observe_clock(args.timeout, args.min_advance)
        elif args.command == "joint-state":
            observe_joint_state(
                args.phase,
                args.smoke_config,
                args.limits_config,
                args.timeout,
            )
        elif args.command == "goal":
            emit_goal(args.smoke_config, args.limits_config)
        elif args.command == "feedback":
            verify_action_feedback(
                args.action_log, args.smoke_config, args.limits_config
            )
        elif args.command == "pose":
            compare_pose(
                args.before,
                args.after,
                args.translation_tolerance,
                args.rotation_tolerance,
            )
        else:  # pragma: no cover - argparse enforces this
            raise AssertionError(args.command)
    except (KeyError, TypeError, ValueError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
