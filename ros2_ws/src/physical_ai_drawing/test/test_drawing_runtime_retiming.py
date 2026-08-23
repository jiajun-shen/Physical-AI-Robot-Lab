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


def test_live_cartesian_execution_uses_only_forward_generated_targets():
    seen_waypoints = []
    runtime = SimpleNamespace(
        cartesian_path=lambda label, waypoints, orientation, scene_ids: (
            seen_waypoints.append(list(waypoints))
            or {
                "trajectory": object(),
                "endpoint": {name: 0.0 for name in JOINTS},
                "cartesian": {
                    "fraction": 1.0,
                    "avoid_collisions": True,
                    "trajectory_waypoint_count": 2,
                },
            }
        ),
        execute=lambda trajectory, label: {"action_status": 4},
        observe_endpoint=lambda endpoint, label: endpoint,
        planner={
            "cartesian_segment_waypoints": 2,
            "observed_joint_tolerance_rad": 0.01,
            "endpoint_samples": 3,
        },
    )
    planned = [
        {
            "label": "draw_segment_00",
            "waypoints": [(1.0, 2.0, 3.0), (1.0, 2.1, 3.0)],
            "cartesian": {"fraction": 1.0, "avoid_collisions": True},
        }
    ]

    result = DrawingRuntime.execute_preflighted_cartesian_segments(
        runtime,
        planned,
        (0.0, 0.0, 0.0, 1.0),
        ["p6_live_vision_drawing_board"],
        requested_waypoint_count=2,
    )

    assert seen_waypoints == [[(1.0, 2.1, 3.0)]]
    assert result["segments"][0]["live_execution_targets"] == {
        "strategy": "live_state_to_forward_generated_waypoints",
        "preflight_waypoint_count": 2,
        "live_forward_target_count": 1,
    }
    assert result["segments"][0]["endpoint_gate"] == {
        "tolerance_rad": 0.01,
        "consecutive_samples": 3,
    }
