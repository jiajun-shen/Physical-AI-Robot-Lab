#!/usr/bin/env python3
"""Source contracts for the first real headless ABB Gazebo runtime slice."""

from __future__ import annotations

import ast
import importlib.util
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION_ROOT = PACKAGE_ROOT.parent / "abb_lab_description"
CONTROL_ROOT = PACKAGE_ROOT.parent / "abb_lab_control"
WORLD_PATH = PACKAGE_ROOT / "worlds" / "irb1200_minimal.sdf"
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "irb1200_headless.launch.py"
RUNTIME_VERIFIER_PATH = PACKAGE_ROOT / "scripts" / "verify_irb1200_runtime.py"
DESCRIPTION_PATH = (
    DESCRIPTION_ROOT / "urdf" / "irb1200_5_90_gz_control.urdf.xacro"
)
EXPECTED_JOINTS = [f"joint_{index}" for index in range(1, 7)]
XACRO_NAMESPACE = "http://ros.org/wiki/xacro"


def required_text(path: Path) -> str:
    if not path.is_file():
        raise AssertionError(f"required file is missing: {path}")
    return path.read_text(encoding="utf-8")


def required_xml(path: Path) -> ET.Element:
    if not path.is_file():
        raise AssertionError(f"required XML is missing: {path}")
    return ET.parse(path).getroot()


class ABBLabGazeboSourceTests(unittest.TestCase):
    def test_runtime_files_exist(self) -> None:
        for path in (
            PACKAGE_ROOT / "CMakeLists.txt",
            PACKAGE_ROOT / "package.xml",
            WORLD_PATH,
            LAUNCH_PATH,
            RUNTIME_VERIFIER_PATH,
        ):
            with self.subTest(path=path.name):
                self.assertTrue(path.is_file(), path)

    def test_world_is_minimal_headless_and_spawn_capable(self) -> None:
        root = required_xml(WORLD_PATH)
        self.assertEqual(root.tag, "sdf")
        worlds = root.findall("world")
        self.assertEqual(len(worlds), 1)
        world = worlds[0]
        self.assertEqual(world.attrib["name"], "abb_lab")
        self.assertEqual(world.findtext("gravity"), "0 0 -9.81")

        physics = world.find("physics")
        self.assertEqual(float(physics.findtext("max_step_size")), 0.001)
        self.assertEqual(float(physics.findtext("real_time_factor")), 1.0)

        plugins = {
            plugin.attrib["name"]: plugin.attrib["filename"]
            for plugin in world.findall("plugin")
        }
        self.assertEqual(
            plugins,
            {
                "gz::sim::systems::Physics": "gz-sim-physics-system",
                "gz::sim::systems::UserCommands": "gz-sim-user-commands-system",
                "gz::sim::systems::SceneBroadcaster": (
                    "gz-sim-scene-broadcaster-system"
                ),
            },
        )
        models = world.findall("model")
        self.assertEqual([model.attrib["name"] for model in models], ["ground_plane"])
        self.assertEqual(models[0].findtext("static"), "true")
        self.assertEqual(world.findall("gui"), [])
        self.assertEqual(world.findall("sensor"), [])

    def test_description_has_explicit_runtime_parameter_override(self) -> None:
        root = required_xml(DESCRIPTION_PATH)
        args = {
            element.attrib["name"]: element.attrib["default"]
            for element in root.findall(f"{{{XACRO_NAMESPACE}}}arg")
        }
        self.assertEqual(args["controller_config_file"], "")
        plugins = root.findall("gazebo/plugin")
        self.assertEqual(len(plugins), 1)
        self.assertEqual(
            plugins[0].findtext("parameters"),
            "$(find abb_lab_description)/config/controller_manager_safety.yaml",
        )
        conditionals = plugins[0].findall(f"{{{XACRO_NAMESPACE}}}if")
        self.assertEqual(len(conditionals), 1)
        self.assertEqual(
            conditionals[0].attrib["value"], "${controller_config_file != ''}"
        )
        self.assertEqual(
            conditionals[0].findtext("parameters"), "$(arg controller_config_file)"
        )

    def test_launch_wires_real_server_spawn_and_ordered_controllers(self) -> None:
        source = required_text(LAUNCH_PATH)
        ast.parse(source, filename=str(LAUNCH_PATH))
        required_tokens = (
            "ros_gz_sim",
            "gz_sim.launch.py",
            "irb1200_minimal.sdf",
            "-s",
            "-r",
            "robot_state_publisher",
            "parameter_bridge",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "irb1200_5_90_gz_control.urdf.xacro",
            "controller_config_file:=",
            "irb1200_controllers.yaml",
            "create",
            "-world",
            "abb_lab",
            "-name",
            "abb_irb1200",
            "joint_state_broadcaster",
            "joint_trajectory_controller",
            "OnProcessExit",
            "event.returncode",
            "Shutdown",
            "-allow_renaming=false",
        )
        for token in required_tokens:
            with self.subTest(token=token):
                self.assertIn(token, source)
        for forbidden in ("rviz", "gzclient", "allow_renaming', 'true"):
            self.assertNotIn(forbidden, source.lower())

    def test_runtime_verifier_requires_real_messages_and_multi_frame_evidence(self) -> None:
        source = required_text(RUNTIME_VERIFIER_PATH)
        ast.parse(source, filename=str(RUNTIME_VERIFIER_PATH))
        for token in (
            "rclpy",
            "sensor_msgs.msg",
            "JointState",
            "rosgraph_msgs.msg",
            "Clock",
            "math.isfinite",
            "joint_1",
            "joint_6",
            "--phase",
            "initial",
            "target",
            "consecutive_target_samples",
            "--timeout",
            "CLOCK_PROGRESS=PASS",
            "JOINT_STATE_OBSERVED=PASS",
            "TRAJECTORY_FEEDBACK_BOUNDS=PASS",
        ):
            self.assertIn(token, source)
        self.assertNotIn("random", source.lower())

    def test_pose_verifier_rejects_nonfinite_or_negative_tolerances(self) -> None:
        spec = importlib.util.spec_from_file_location(
            "verify_irb1200_runtime_for_test", RUNTIME_VERIFIER_PATH
        )
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory(prefix="abb-pose-verifier-") as directory:
            pose_path = Path(directory) / "pose.txt"
            pose_path.write_text(
                "Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
                "[0.0 0.0 0.0]\n[0.0 0.0 0.0]\n",
                encoding="utf-8",
            )
            for translation_tolerance, rotation_tolerance in (
                (math.nan, 0.1),
                (0.1, math.inf),
                (-0.1, 0.1),
                (0.1, -0.1),
            ):
                with self.subTest(
                    translation=translation_tolerance,
                    rotation=rotation_tolerance,
                ):
                    with self.assertRaises(ValueError):
                        module.compare_pose(
                            pose_path,
                            pose_path,
                            translation_tolerance,
                            rotation_tolerance,
                        )

    def test_manifest_has_no_moveit_gripper_or_real_driver_dependency(self) -> None:
        manifest = PACKAGE_ROOT / "package.xml"
        self.assertTrue(manifest.is_file(), manifest)
        root = ET.parse(manifest).getroot()
        self.assertEqual(root.findtext("name"), "abb_lab_gazebo")
        dependencies = {element.text for element in root.findall("exec_depend")}
        self.assertTrue(
            {
                "abb_lab_control",
                "abb_lab_description",
                "controller_manager",
                "robot_state_publisher",
                "ros_gz_bridge",
                "ros_gz_sim",
                "xacro",
            }
            <= dependencies
        )
        self.assertTrue(
            {
                "abb_hardware_interface",
                "abb_rws",
                "abb_egm",
                "moveit_ros_move_group",
                "rviz2",
            }.isdisjoint(dependencies)
        )


if __name__ == "__main__":
    unittest.main()
