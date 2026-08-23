#!/usr/bin/env python3
"""Static architecture contracts for the ABB arm-and-gripper lab bringup."""

from __future__ import annotations

import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
GRIPPER_ROOT = PACKAGE_ROOT.parent / "abb_lab_gripper"
LAB_ROOT = PACKAGE_ROOT.parent / "abb_lab_lab"
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "irb1200_gripper_lab_headless.launch.py"
RGBD_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "irb1200_gripper_lab_rgbd_headless.launch.py"
RGBD_WORLD_PATH = LAB_ROOT / "worlds" / "manipulation_lab_rgbd.sdf"


def read_required(path: Path) -> str:
    """Read a required text artifact with a useful assertion error."""
    if not path.is_file():
        raise AssertionError(f"required file is missing: {path}")
    return path.read_text(encoding="utf-8")


def literal_keyword(call: ast.Call, keyword: str) -> str | None:
    """Return a literal string keyword value from a call when present."""
    for item in call.keywords:
        if (
            item.arg == keyword
            and isinstance(item.value, ast.Constant)
            and isinstance(item.value.value, str)
        ):
            return item.value.value
    return None


class ABBLabBringupSourceContracts(unittest.TestCase):
    """Lock the single-GazeboSimSystem controller-manager topology."""

    def test_package_artifacts_and_upstream_assets_exist(self) -> None:
        required_paths = (
            PACKAGE_ROOT / "CMakeLists.txt",
            PACKAGE_ROOT / "package.xml",
            LAUNCH_PATH,
            RGBD_LAUNCH_PATH,
            GRIPPER_ROOT / "urdf" / "irb1200_5_90_gz_gripper_control.urdf.xacro",
            GRIPPER_ROOT / "config" / "irb1200_gripper_controllers.yaml",
            LAB_ROOT / "worlds" / "manipulation_lab.sdf",
            RGBD_WORLD_PATH,
        )
        for path in required_paths:
            with self.subTest(path=path):
                self.assertTrue(path.is_file(), path)

    def test_headless_launch_uses_lab_world_and_combined_robot_description(self) -> None:
        source = read_required(LAUNCH_PATH)
        ast.parse(source, filename=str(LAUNCH_PATH))

        required_tokens = (
            'LaunchConfiguration("world_path")',
            'DeclareLaunchArgument(\n                "world_path"',
            "manipulation_lab.sdf",
            "irb1200_5_90_gz_gripper_control.urdf.xacro",
            "irb1200_gripper_controllers.yaml",
            "controller_config_file:=",
            "position_proportional_gain:=",
            "gz_sim.launch.py",
            "gz-physics-bullet-featherstone-plugin",
            '"-s -r -v 1 --physics-engine "',
            "robot_state_publisher",
            "parameter_bridge",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "ros_gz_sim",
            '"create"',
            '"abb_irb1200_gripper"',
            '"-allow_renaming=false"',
        )
        for token in required_tokens:
            with self.subTest(token=token):
                self.assertIn(token, source)

        for forbidden_token in (
            "abb_lab_gazebo",
            "irb1200_headless.launch.py",
            "ros2_control_node",
            "fake_components",
            "mock_components",
            "rviz",
            "gzclient",
        ):
            with self.subTest(forbidden=forbidden_token):
                self.assertNotIn(forbidden_token, source.lower())

        self.assertEqual(source.count('executable="create"'), 1)
        self.assertEqual(source.count('"-allow_renaming=false"'), 1)

    def test_controller_spawners_are_strictly_ordered_and_return_code_gated(self) -> None:
        source = read_required(LAUNCH_PATH)
        expected_order = (
            "joint_state_broadcaster",
            "joint_trajectory_controller",
            "two_finger_gripper_controller",
        )
        positions = [source.index(controller) for controller in expected_order]
        self.assertEqual(positions, sorted(positions))
        self.assertEqual(source.count('package="controller_manager"'), 3)
        self.assertEqual(source.count('executable="spawner"'), 3)
        self.assertGreaterEqual(source.count("OnProcessExit"), 3)
        self.assertIn("event.returncode == 0", source)
        self.assertIn("EmitEvent(event=Shutdown", source)
        self.assertIn('"ABB-with-gripper entity creation"', source)
        self.assertIn('"joint_state_broadcaster spawner"', source)
        self.assertIn('"joint_trajectory_controller spawner"', source)
        self.assertIn('"two_finger_gripper_controller spawner"', source)
        self.assertIn("target_action=spawn_robot", source)
        self.assertIn("target_action=joint_state_broadcaster_spawner", source)
        self.assertIn("target_action=joint_trajectory_controller_spawner", source)
        self.assertIn(
            "target_action=two_finger_gripper_controller_spawner", source
        )

        tree = ast.parse(source, filename=str(LAUNCH_PATH))
        nodes = [
            call
            for call in ast.walk(tree)
            if isinstance(call, ast.Call)
            and isinstance(call.func, ast.Name)
            and call.func.id == "Node"
        ]
        controller_nodes = [
            node
            for node in nodes
            if literal_keyword(node, "package") == "controller_manager"
        ]
        self.assertEqual(len(controller_nodes), 3)
        self.assertTrue(
            all(literal_keyword(node, "executable") == "spawner" for node in controller_nodes)
        )
        self.assertNotIn(
            "ros2_control_node",
            {literal_keyword(node, "executable") for node in nodes},
        )

    def test_manifest_declares_only_bringup_runtime_dependencies(self) -> None:
        manifest = PACKAGE_ROOT / "package.xml"
        self.assertTrue(manifest.is_file(), manifest)
        root = ET.parse(manifest).getroot()
        self.assertEqual(root.findtext("name"), "abb_lab_bringup")
        dependencies = {element.text for element in root.findall("exec_depend")}
        self.assertTrue(
            {
                "abb_lab_gripper",
                "abb_lab_lab",
                "controller_manager",
                "joint_state_broadcaster",
                "joint_trajectory_controller",
                "parallel_gripper_controller",
                "robot_state_publisher",
                "ros_gz_bridge",
                "ros_gz_sim",
                "sensor_msgs",
                "tf2_ros",
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

    def test_cmake_installs_launch_and_registers_only_source_contract_test(self) -> None:
        cmake = read_required(PACKAGE_ROOT / "CMakeLists.txt")
        self.assertIn("install(DIRECTORY launch", cmake)
        self.assertIn('PATTERN "__pycache__" EXCLUDE', cmake)
        self.assertIn('PATTERN "*.pyc" EXCLUDE', cmake)
        self.assertIn("ament_add_pytest_test(abb_lab_bringup_source_contract", cmake)
        self.assertNotIn("verify_", cmake)

    def test_rgbd_wrapper_reuses_the_p3_control_chain_and_bridges_only_sensors(self) -> None:
        source = read_required(RGBD_LAUNCH_PATH)
        ast.parse(source, filename=str(RGBD_LAUNCH_PATH))
        required_tokens = (
            "irb1200_gripper_lab_headless.launch.py",
            "manipulation_lab_rgbd.sdf",
            'package="ros_gz_bridge"',
            'executable="parameter_bridge"',
            "/lab_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/lab_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/lab_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "override_frame_id:=lab_camera_optical_frame",
            'package="tf2_ros"',
            "lab_camera_link",
            "lab_camera_optical_frame",
            "controller_config_file",
            "position_proportional_gain",
        )
        for token in required_tokens:
            with self.subTest(token=token):
                self.assertIn(token, source)
        for forbidden in (
            "ros2_control_node",
            "controller_manager",
            "gz model",
            "model_pose",
            "ground_truth",
            "gazebo_msgs",
            "ros_gz_interfaces",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, source.lower())
        self.assertEqual(source.count('executable="parameter_bridge"'), 1)
        self.assertEqual(source.count('static_transform_publisher'), 2)

    def test_rgbd_world_is_separate_and_has_one_static_sensor_camera(self) -> None:
        original = read_required(LAB_ROOT / "worlds" / "manipulation_lab.sdf")
        rgbd_root = ET.parse(RGBD_WORLD_PATH).getroot()
        self.assertEqual(rgbd_root.tag, "sdf")
        world = rgbd_root.find("world")
        self.assertIsNotNone(world)
        assert world is not None
        self.assertEqual(world.attrib.get("name"), "manipulation_lab")
        self.assertIn("gz-sim-sensors-system", read_required(RGBD_WORLD_PATH))
        self.assertIn("ogre2", read_required(RGBD_WORLD_PATH))
        self.assertNotIn("gz-sim-sensors-system", original)
        models = {model.attrib.get("name"): model for model in world.findall("model")}
        camera = models.get("lab_rgbd_camera")
        self.assertIsNotNone(camera)
        assert camera is not None
        self.assertEqual(camera.findtext("static"), "true")
        sensor = camera.find("link/sensor")
        self.assertIsNotNone(sensor)
        assert sensor is not None
        self.assertEqual(sensor.attrib.get("type"), "rgbd_camera")
        self.assertEqual(sensor.findtext("topic"), "/lab_camera")
        self.assertEqual(sensor.findtext("camera/image/width"), "640")
        self.assertEqual(sensor.findtext("camera/image/height"), "480")


if __name__ == "__main__":
    unittest.main()
