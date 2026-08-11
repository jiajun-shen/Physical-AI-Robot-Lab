"""Static contracts for the project-owned ABB MoveIt configuration.

These tests deliberately validate package data only.  They do not launch
Gazebo, move_group, a controller, or an execution action; P2.2 must provide
that separate runtime evidence before A02 can change state.
"""

from __future__ import annotations

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import xacro
import yaml


PACKAGE_SHARE = Path(get_package_share_directory("abb_lab_moveit_config"))
DESCRIPTION_SHARE = Path(get_package_share_directory("abb_lab_description"))
CONTROL_SHARE = Path(get_package_share_directory("abb_lab_control"))

SRDF_PATH = PACKAGE_SHARE / "config" / "irb1200_5_90_gz_control.srdf"
RUNTIME_URDF_XACRO_PATH = (
    DESCRIPTION_SHARE / "urdf" / "irb1200_5_90_gz_control.urdf.xacro"
)
LIMITS_PATH = PACKAGE_SHARE / "config" / "joint_limits.yaml"
KINEMATICS_PATH = PACKAGE_SHARE / "config" / "kinematics.yaml"
OMPL_PATH = PACKAGE_SHARE / "config" / "ompl_planning.yaml"
CONTROLLERS_PATH = PACKAGE_SHARE / "config" / "moveit_controllers.yaml"
SETUP_ASSISTANT_PATH = PACKAGE_SHARE / ".setup_assistant"
CANONICAL_LIMITS_PATH = (
    DESCRIPTION_SHARE / "config" / "irb1200_5_90_control_limits.yaml"
)
RUNTIME_CONTROLLERS_PATH = CONTROL_SHARE / "config" / "irb1200_controllers.yaml"

JOINTS = [f"joint_{index}" for index in range(1, 7)]
LIMIT_KEYS = {
    "has_position_limits",
    "min_position",
    "max_position",
    "has_velocity_limits",
    "max_velocity",
}


def load_yaml(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise AssertionError(f"{path} must contain a YAML mapping")
    return value


def collect_keys(value: object) -> set[str]:
    if isinstance(value, dict):
        return {
            str(key)
            for key in value
        }.union(*(collect_keys(item) for item in value.values()))
    if isinstance(value, list):
        return set().union(*(collect_keys(item) for item in value))
    return set()


class TestAbbLabMoveItConfigSource(unittest.TestCase):
    def test_setup_assistant_binds_project_description_and_semantics(self) -> None:
        config = load_yaml(SETUP_ASSISTANT_PATH)["moveit_setup_assistant_config"]
        self.assertEqual(
            config["URDF"],
            {
                "package": "abb_lab_description",
                "relative_path": "urdf/irb1200_5_90_gz_control.urdf.xacro",
            },
        )
        self.assertEqual(
            config["SRDF"],
            {"relative_path": "config/irb1200_5_90_gz_control.srdf"},
        )
        self.assertEqual(
            config["CONFIG"],
            {
                "author_name": "Physical-AI-Robot-Lab maintainers",
                "author_email": "maintainers@example.invalid",
                "generated_timestamp": "2026-08-11",
            },
        )

    def test_arm_only_semantic_chain_has_canonical_base_and_tool(self) -> None:
        root = ET.parse(SRDF_PATH).getroot()
        self.assertEqual(root.tag, "robot")
        self.assertEqual(root.attrib, {"name": "abb_irb1200_5_90_gz_control"})

        groups = root.findall("group")
        self.assertEqual(len(groups), 1)
        group = groups[0]
        self.assertEqual(group.attrib, {"name": "manipulator"})
        chain = group.find("chain")
        self.assertIsNotNone(chain)
        self.assertEqual(chain.attrib, {"base_link": "base_link", "tip_link": "tool0"})
        self.assertIsNone(root.find("end_effector"))

    def test_installed_runtime_urdf_expands_and_resolves_semantic_chain(self) -> None:
        srdf_root = ET.parse(SRDF_PATH).getroot()
        expanded_root = ET.fromstring(xacro.process_file(RUNTIME_URDF_XACRO_PATH).toxml())
        self.assertEqual(expanded_root.tag, "robot")
        self.assertEqual(expanded_root.attrib["name"], srdf_root.attrib["name"])

        links = {link.attrib["name"] for link in expanded_root.findall("link")}
        self.assertTrue({"base_link", "tool0"}.issubset(links))
        joints = {joint.attrib["name"]: joint for joint in expanded_root.findall("joint")}
        self.assertTrue(set(JOINTS).issubset(joints))
        self.assertEqual(
            [joints[name].attrib["type"] for name in JOINTS],
            ["revolute"] * len(JOINTS),
        )

        parent_by_child = {
            joint.find("child").attrib["link"]: joint
            for joint in joints.values()
        }
        active_joints: list[str] = []
        current_link = "tool0"
        while current_link != "base_link":
            joint = parent_by_child[current_link]
            if joint.attrib["type"] != "fixed":
                active_joints.append(joint.attrib["name"])
            current_link = joint.find("parent").attrib["link"]
        self.assertEqual(active_joints, list(reversed(JOINTS)))

    def test_home_state_is_exact_six_joint_neutral_state_within_canonical_limits(self) -> None:
        root = ET.parse(SRDF_PATH).getroot()
        home = root.find("group_state[@name='home']")
        self.assertIsNotNone(home)
        self.assertEqual(home.attrib, {"name": "home", "group": "manipulator"})
        home_positions = {
            joint.attrib["name"]: float(joint.attrib["value"])
            for joint in home.findall("joint")
        }
        self.assertEqual(list(home_positions), JOINTS)
        self.assertEqual(set(home_positions.values()), {0.0})

        canonical = load_yaml(CANONICAL_LIMITS_PATH)["joint_limits"]
        for name, position in home_positions.items():
            self.assertGreaterEqual(position, canonical[name]["min_position"])
            self.assertLessEqual(position, canonical[name]["max_position"])

    def test_moveit_limits_exactly_mirror_canonical_position_and_velocity_limits(self) -> None:
        actual = load_yaml(LIMITS_PATH)
        canonical = load_yaml(CANONICAL_LIMITS_PATH)
        self.assertEqual(set(actual), {"joint_limits"})
        self.assertEqual(set(actual["joint_limits"]), set(JOINTS))
        self.assertEqual(set(canonical["joint_limits"]), set(JOINTS))
        for name in JOINTS:
            self.assertEqual(set(actual["joint_limits"][name]), LIMIT_KEYS)
            self.assertEqual(actual["joint_limits"][name], canonical["joint_limits"][name])

    def test_moveit_limits_do_not_invent_acceleration_or_jerk_constraints(self) -> None:
        prohibited_fragments = ("acceleration", "deceleration", "jerk")
        keys = collect_keys(load_yaml(LIMITS_PATH))
        self.assertFalse(
            any(fragment in key.lower() for fragment in prohibited_fragments for key in keys),
            f"unverified dynamic limit key found: {sorted(keys)}",
        )

    def test_kinematics_is_configured_for_the_manipulator_group(self) -> None:
        kinematics = load_yaml(KINEMATICS_PATH)
        self.assertEqual(set(kinematics), {"manipulator"})
        self.assertEqual(
            kinematics["manipulator"]["kinematics_solver"],
            "kdl_kinematics_plugin/KDLKinematicsPlugin",
        )

    def test_ompl_pipeline_selects_collision_checking_rrt_connect_for_manipulator(self) -> None:
        ompl = load_yaml(OMPL_PATH)
        self.assertEqual(ompl["planning_plugins"], ["ompl_interface/OMPLPlanner"])
        self.assertIn(
            "default_planning_request_adapters/CheckStartStateCollision",
            ompl["request_adapters"],
        )
        self.assertIn(
            "default_planning_response_adapters/ValidateSolution",
            ompl["response_adapters"],
        )
        self.assertEqual(
            ompl["manipulator"]["planner_configs"], ["RRTConnectkConfigDefault"]
        )
        self.assertEqual(
            ompl["planner_configs"]["RRTConnectkConfigDefault"],
            {"type": "geometric::RRTConnect", "range": 0.0},
        )

    def test_moveit_controller_mapping_matches_project_trajectory_controller(self) -> None:
        moveit = load_yaml(CONTROLLERS_PATH)
        runtime = load_yaml(RUNTIME_CONTROLLERS_PATH)[
            "joint_trajectory_controller"
        ]["ros__parameters"]

        self.assertEqual(
            moveit["moveit_controller_manager"],
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        )
        self.assertFalse(moveit["moveit_manage_controllers"])
        self.assertEqual(
            moveit["trajectory_execution"],
            {
                "execution_duration_monitoring": True,
                "allowed_execution_duration_scaling": 1.2,
                "allowed_goal_duration_margin": 0.5,
                "allowed_start_tolerance": 0.01,
            },
        )
        controllers = moveit["moveit_simple_controller_manager"]
        self.assertEqual(controllers["controller_names"], ["joint_trajectory_controller"])
        controller = controllers["joint_trajectory_controller"]
        self.assertEqual(controller["type"], "FollowJointTrajectory")
        self.assertEqual(controller["action_ns"], "follow_joint_trajectory")
        self.assertTrue(controller["default"])
        self.assertEqual(controller["joints"], JOINTS)
        self.assertEqual(controller["joints"], runtime["joints"])
        self.assertEqual(
            f"/{controllers['controller_names'][0]}/{controller['action_ns']}",
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

    def test_moveit_configs_builder_loads_the_installed_configuration(self) -> None:
        configs = (
            MoveItConfigsBuilder(
                "irb1200_5_90_gz_control", package_name="abb_lab_moveit_config"
            )
            .robot_description()
            .robot_description_semantic()
            .robot_description_kinematics()
            .joint_limits()
            .planning_pipelines(pipelines=["ompl"], load_all=False)
            .trajectory_execution()
            .to_moveit_configs()
            .to_dict()
        )
        required = {
            "robot_description",
            "robot_description_semantic",
            "robot_description_kinematics",
            "robot_description_planning",
            "planning_pipelines",
            "default_planning_pipeline",
            "ompl",
            "moveit_controller_manager",
            "moveit_simple_controller_manager",
        }
        self.assertFalse(required.difference(configs))
        self.assertEqual(configs["default_planning_pipeline"], "ompl")
        self.assertEqual(
            configs["ompl"]["manipulator"]["planner_configs"],
            ["RRTConnectkConfigDefault"],
        )

    def test_package_contains_no_runtime_launch_file(self) -> None:
        self.assertFalse((PACKAGE_SHARE / "launch").exists())


if __name__ == "__main__":
    unittest.main()
