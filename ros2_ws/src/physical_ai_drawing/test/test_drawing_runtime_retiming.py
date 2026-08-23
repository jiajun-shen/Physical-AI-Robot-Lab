from types import SimpleNamespace

import pytest
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from physical_ai_drawing.drawing_runtime import DrawingRuntime


JOINTS = tuple(f"joint_{index}" for index in range(1, 7))


def _seconds(point: JointTrajectoryPoint) -> float:
    return float(point.time_from_start.sec) + float(point.time_from_start.nanosec) / 1_000_000_000.0


def test_cartesian_retime_appends_a_same_endpoint_terminal_hold():
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = list(JOINTS)
    trajectory.joint_trajectory.points = [
        JointTrajectoryPoint(positions=[0.0] * len(JOINTS)),
        JointTrajectoryPoint(positions=[0.02] * len(JOINTS)),
    ]
    runtime = SimpleNamespace(
        arm_joints=JOINTS,
        velocity_limits={name: 1.0 for name in JOINTS},
        planner={
            "cartesian_velocity_scaling": 0.1,
            "cartesian_retime_safety_factor": 2.0,
            "cartesian_minimum_segment_duration_sec": 0.2,
            "cartesian_terminal_hold_sec": 1.0,
        },
        _set_time=DrawingRuntime._set_time,
    )

    result, evidence = DrawingRuntime.retime(runtime, trajectory, "unit_cartesian", cartesian=True)

    points = result.joint_trajectory.points
    assert len(points) == 3
    assert list(points[-1].positions) == pytest.approx(points[-2].positions)
    assert _seconds(points[-1]) - _seconds(points[-2]) == pytest.approx(1.0)
    assert evidence["terminal_hold"] == {
        "duration_sec": 1.0,
        "repeats_collision_checked_endpoint": True,
    }
