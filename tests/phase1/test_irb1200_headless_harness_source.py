#!/usr/bin/env python3
"""Repository-level source contract for the destructive-safe runtime harness."""

from __future__ import annotations

from pathlib import Path
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
RUNTIME_HARNESS_PATH = REPOSITORY_ROOT / "scripts" / "run_and_verify_irb1200_headless.sh"


class IRB1200HeadlessHarnessSourceTests(unittest.TestCase):
    def test_runtime_harness_has_isolation_evidence_and_cleanup(self) -> None:
        self.assertTrue(RUNTIME_HARNESS_PATH.is_file(), RUNTIME_HARNESS_PATH)
        source = RUNTIME_HARNESS_PATH.read_text(encoding="utf-8")
        required_tokens = (
            "set -eo pipefail",
            "timeout --signal=TERM --kill-after=30s 180s",
            "could not prove selected ROS domain",
            "could not re-prove selected ROS domain",
            "ROS_DOMAIN_ID",
            "GZ_PARTITION",
            "ROS_HOME",
            "ROS_AUTOMATIC_DISCOVERY_RANGE",
            "flock",
            "mktemp -d",
            ".runtime-gate.lock",
            "output directory is already in use",
            "setsid ros2 launch abb_lab_gazebo irb1200_headless.launch.py",
            "trap cleanup EXIT",
            "trap 'cleanup 130' INT",
            "trap 'cleanup 143' TERM",
            "trap '' INT TERM",
            "kill -INT -- \"-${launch_pgid}\"",
            "kill -TERM -- \"-${launch_pgid}\"",
            "kill -KILL -- \"-${launch_pgid}\"",
            "ros2 control list_hardware_components",
            "ros2 control list_hardware_interfaces",
            "ros2 control list_controllers",
            "ros2 param get /controller_manager enforce_command_limits",
            "Creating JointSaturationLimiter for joint",
            "gz model --list",
            "gz model -m abb_irb1200 -p",
            "gz model -m abb_irb1200 -j",
            "control_msgs/action/FollowJointTrajectory",
            "verify_irb1200_runtime.py",
            "PROJECT_ABB_GAZEBO_SPAWN=PASS",
            "HARDWARE_RESOURCES=PASS",
            "CONTROLLERS_ACTIVE=PASS",
            "LIMITER_RUNTIME_INITIALIZED=PASS",
            "BOUNDED_TRAJECTORY=PASS",
            "CLAMP_PROBE=NOT_RUN",
            "CLAMP_BEHAVIOR_VERIFIED=false",
            "GUI_RENDERING=NOT_RUN",
            "ABB_HEADLESS_RUNTIME=PASS",
            "CLEANUP_VERIFIED=PASS",
            "working_source_before.sha256",
            "working_source_after.sha256",
            "cmp --silent",
        )
        for token in required_tokens:
            with self.subTest(token=token):
                self.assertIn(token, source)
        self.assertIn("Polling delay is test orchestration", source)
        self.assertNotIn("killall", source)
        self.assertNotIn("pkill", source)
        self.assertNotIn("rm -rf", source)


if __name__ == "__main__":
    unittest.main()
