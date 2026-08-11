#!/usr/bin/env python3
"""Source contracts for the minimal six-axis ABB simulation controllers."""

from __future__ import annotations

import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONTROLLERS_PATH = PACKAGE_ROOT / "config" / "irb1200_controllers.yaml"
BOUNDED_SMOKE_PATH = PACKAGE_ROOT / "config" / "irb1200_bounded_smoke.yaml"
CANONICAL_LIMITS_PATH = (
    PACKAGE_ROOT.parent
    / "abb_lab_description"
    / "config"
    / "irb1200_5_90_control_limits.yaml"
)
EXPECTED_JOINTS = [f"joint_{index}" for index in range(1, 7)]


def required_yaml(path: Path) -> dict:
    if not path.is_file():
        raise AssertionError(f"required YAML is missing: {path}")
    return yaml.safe_load(path.read_text(encoding="utf-8"))


class ABBLabControlSourceTests(unittest.TestCase):
    def test_package_files_exist(self) -> None:
        for path in (
            PACKAGE_ROOT / "CMakeLists.txt",
            PACKAGE_ROOT / "package.xml",
            CONTROLLERS_PATH,
            BOUNDED_SMOKE_PATH,
        ):
            with self.subTest(path=path.name):
                self.assertTrue(path.is_file(), path)

    def test_controller_manager_safety_and_rate_are_explicit(self) -> None:
        config = required_yaml(CONTROLLERS_PATH)
        self.assertSetEqual(
            set(config), {"controller_manager", "joint_trajectory_controller"}
        )
        manager = config["controller_manager"]["ros__parameters"]
        self.assertEqual(manager["update_rate"], 100)
        self.assertNotIn("enforce_command_limits", manager)
        self.assertEqual(
            manager["joint_state_broadcaster"],
            {"type": "joint_state_broadcaster/JointStateBroadcaster"},
        )

    def test_trajectory_controller_owns_exact_six_position_joints(self) -> None:
        config = required_yaml(CONTROLLERS_PATH)
        parameters = config["joint_trajectory_controller"]["ros__parameters"]
        self.assertEqual(
            parameters["type"],
            "joint_trajectory_controller/JointTrajectoryController",
        )
        self.assertEqual(parameters["joints"], EXPECTED_JOINTS)
        self.assertEqual(parameters["command_interfaces"], ["position"])
        self.assertEqual(parameters["state_interfaces"], ["position", "velocity"])
        self.assertIs(parameters["allow_partial_joints_goal"], False)
        self.assertIs(parameters["open_loop_control"], False)
        constraints = parameters["constraints"]
        self.assertEqual(constraints["goal_time"], 2.0)
        self.assertEqual(constraints["stopped_velocity_tolerance"], 0.05)
        for joint_name in EXPECTED_JOINTS:
            self.assertEqual(
                constraints[joint_name], {"trajectory": 0.1, "goal": 0.03}
            )

    def test_controller_config_contains_no_invented_physical_limits(self) -> None:
        config = required_yaml(CONTROLLERS_PATH)
        text = CONTROLLERS_PATH.read_text(encoding="utf-8").lower()
        for forbidden in (
            "max_acceleration",
            "max_deceleration",
            "max_jerk",
            "max_effort",
            "joint_limits:",
            "initial_value",
        ):
            self.assertNotIn(forbidden, text)

        manager = config["controller_manager"]["ros__parameters"]
        self.assertTrue(math.isfinite(float(manager["update_rate"])))
        self.assertGreater(manager["update_rate"], 0)

    def test_bounded_goal_is_small_neutral_and_inside_canonical_margins(self) -> None:
        smoke = required_yaml(BOUNDED_SMOKE_PATH)
        canonical = required_yaml(CANONICAL_LIMITS_PATH)["joint_limits"]
        self.assertEqual(smoke["schema_version"], 1)
        self.assertEqual(smoke["scope"], "SIMULATION_ONLY")
        self.assertEqual(smoke["joint_order"], EXPECTED_JOINTS)
        self.assertSetEqual(set(smoke["neutral_reference"]), set(EXPECTED_JOINTS))
        self.assertSetEqual(set(smoke["target"]), set(EXPECTED_JOINTS))

        guards = smoke["guards"]
        self.assertLessEqual(guards["max_delta_rad"], 0.1)
        self.assertGreaterEqual(guards["limit_margin_rad"], 0.05)
        self.assertGreaterEqual(guards["time_from_start_sec"], 3)
        self.assertLessEqual(guards["final_position_tolerance_rad"], 0.03)
        self.assertGreaterEqual(guards["consecutive_target_samples"], 3)
        self.assertGreaterEqual(guards["minimum_action_feedback_samples"], 10)

        for joint_name in EXPECTED_JOINTS:
            neutral = float(smoke["neutral_reference"][joint_name])
            target = float(smoke["target"][joint_name])
            limits = canonical[joint_name]
            self.assertEqual(neutral, 0.0)
            self.assertLessEqual(abs(target), 0.08)
            self.assertLessEqual(abs(target - neutral), guards["max_delta_rad"])
            self.assertGreaterEqual(
                target - limits["min_position"], guards["limit_margin_rad"]
            )
            self.assertGreaterEqual(
                limits["max_position"] - target, guards["limit_margin_rad"]
            )

        safety = smoke["safety_status"]
        self.assertEqual(safety["collision_free"], "NOT_VERIFIED")
        self.assertEqual(safety["clamp_probe"], "NOT_RUN")
        self.assertIs(safety["real_robot_ready"], False)

    def test_manifest_declares_only_the_runtime_control_stack(self) -> None:
        manifest = PACKAGE_ROOT / "package.xml"
        self.assertTrue(manifest.is_file(), manifest)
        root = ET.parse(manifest).getroot()
        self.assertEqual(root.findtext("name"), "abb_lab_control")
        dependencies = {element.text for element in root.findall("exec_depend")}
        self.assertTrue(
            {
                "controller_manager",
                "joint_state_broadcaster",
                "joint_trajectory_controller",
            }
            <= dependencies
        )
        self.assertTrue(
            {
                "abb_hardware_interface",
                "abb_rws",
                "abb_egm",
                "moveit_ros_move_group",
            }.isdisjoint(dependencies)
        )


if __name__ == "__main__":
    unittest.main()
