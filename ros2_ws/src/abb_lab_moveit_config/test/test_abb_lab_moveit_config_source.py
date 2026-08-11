"""Static contracts for the project-owned ABB MoveIt configuration.

P2.1 validates the declarative configuration.  P2.2 adds source contracts for
the *separate* headless launch, verifier, and bounded root harness.  These
tests never claim that a plan or trajectory has run; only a clean P2.2 runtime
gate may change A02.
"""

from __future__ import annotations

import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import xacro
import yaml


PACKAGE_SHARE = Path(get_package_share_directory("abb_lab_moveit_config"))
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
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
SMOKE_PATH = PACKAGE_SHARE / "config" / "irb1200_moveit_smoke.yaml"
SETUP_ASSISTANT_PATH = PACKAGE_SHARE / ".setup_assistant"
CANONICAL_LIMITS_PATH = (
    DESCRIPTION_SHARE / "config" / "irb1200_5_90_control_limits.yaml"
)
RUNTIME_CONTROLLERS_PATH = CONTROL_SHARE / "config" / "irb1200_controllers.yaml"
P22_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "irb1200_moveit_headless.launch.py"
P22_VERIFIER_PATH = PACKAGE_ROOT / "scripts" / "verify_moveit_runtime.py"
P22_HARNESS_PATH = REPOSITORY_ROOT / "scripts" / "run_and_verify_irb1200_moveit_headless.sh"

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


def required_source(path: Path) -> str:
    if not path.is_file():
        raise AssertionError(f"required P2.2 source file is missing: {path}")
    return path.read_text(encoding="utf-8")


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

        # The only disabled pairs are physical neighbours that share a joint.
        # This prevents the vendor meshes from reporting permanent self-contact
        # without hiding any non-adjacent collision risk from planning.
        disabled_pairs = {
            (
                item.attrib["link1"],
                item.attrib["link2"],
                item.attrib["reason"],
            )
            for item in root.findall("disable_collisions")
        }
        self.assertEqual(
            disabled_pairs,
            {
                ("base_link", "link_1", "Adjacent"),
                ("link_1", "link_2", "Adjacent"),
                ("link_2", "link_3", "Adjacent"),
                ("link_3", "link_4", "Adjacent"),
                ("link_4", "link_5", "Adjacent"),
                ("link_5", "link_6", "Adjacent"),
            },
        )

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
        # TOTG requires acceleration limits, which this project deliberately
        # does not invent. P2.2 schedules the returned geometric trajectory
        # conservatively from the verified velocity limits instead.
        self.assertNotIn(
            "default_planning_response_adapters/AddTimeOptimalParameterization",
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

    def test_p22_smoke_contract_is_bounded_home_pose_home_without_dynamic_fiction(self) -> None:
        smoke = load_yaml(SMOKE_PATH)
        self.assertEqual(
            set(smoke), {"schema_version", "scope", "robot", "planner", "targets", "guards", "collision"}
        )
        self.assertEqual(smoke["schema_version"], 1)
        self.assertEqual(smoke["scope"], "SIMULATION_ONLY")
        self.assertEqual(
            smoke["robot"],
            {
                "group_name": "manipulator",
                "base_frame": "base_link",
                "tool_link": "tool0",
                "joint_names": JOINTS,
            },
        )
        self.assertEqual(
            set(smoke["planner"]),
            {"pipeline_id", "planner_id", "planning_attempts", "allowed_planning_time_sec", "velocity_scaling"},
        )
        self.assertEqual(smoke["planner"]["pipeline_id"], "ompl")
        self.assertEqual(smoke["planner"]["planner_id"], "RRTConnectkConfigDefault")
        self.assertGreater(smoke["planner"]["planning_attempts"], 0)
        self.assertGreater(smoke["planner"]["allowed_planning_time_sec"], 0.0)
        self.assertGreater(smoke["planner"]["velocity_scaling"], 0.0)
        self.assertLessEqual(smoke["planner"]["velocity_scaling"], 1.0)
        self.assertFalse(
            any(
                fragment in key.lower()
                for fragment in ("acceleration", "deceleration", "jerk")
                for key in collect_keys(smoke)
            )
        )

        canonical = load_yaml(CANONICAL_LIMITS_PATH)["joint_limits"]
        targets = smoke["targets"]
        self.assertEqual(
            set(targets), {"home", "joint_execution", "joint_alternate", "fk_pose_seed"}
        )
        self.assertEqual(targets["home"], {name: 0.0 for name in JOINTS})
        for target_name, positions in targets.items():
            with self.subTest(target=target_name):
                self.assertEqual(set(positions), set(JOINTS))
                for name in JOINTS:
                    self.assertGreaterEqual(positions[name], canonical[name]["min_position"])
                    self.assertLessEqual(positions[name], canonical[name]["max_position"])

        guards = smoke["guards"]
        self.assertEqual(guards["velocity_only_retime_safety_factor"], 2.0)
        self.assertGreater(guards["minimum_segment_duration_sec"], 0.0)
        self.assertGreater(guards["observed_home_tolerance_rad"], 0.0)
        self.assertEqual(smoke["collision"]["object_id"], "p22_tool0_goal_block")
        self.assertEqual(len(smoke["collision"]["box_dimensions_m"]), 3)
        self.assertTrue(all(value > 0.0 for value in smoke["collision"]["box_dimensions_m"]))

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

    def test_p22_runtime_sources_exist_and_are_installed(self) -> None:
        """P2.2 must own an installed launch and verifier, not a README claim."""
        for path in (P22_LAUNCH_PATH, P22_VERIFIER_PATH, P22_HARNESS_PATH):
            with self.subTest(path=path):
                self.assertTrue(path.is_file(), path)

        cmake = required_source(PACKAGE_ROOT / "CMakeLists.txt")
        self.assertRegex(
            cmake,
            r"install\s*\(\s*DIRECTORY\s+config\s+launch\s+scripts\b",
        )

    def test_p22_launch_reuses_the_owned_gazebo_control_chain_before_move_group(self) -> None:
        """The MoveIt layer must attach to P1.3b, never create another control stack."""
        source = required_source(P22_LAUNCH_PATH)
        ast.parse(source, filename=str(P22_LAUNCH_PATH))

        self.assertRegex(
            source,
            r"FindPackageShare\(\s*['\"]abb_lab_gazebo['\"]\s*\)",
        )
        for token in (
            "IncludeLaunchDescription",
            "PythonLaunchDescriptionSource",
            "irb1200_headless.launch.py",
            "moveit_ros_move_group",
            "move_group",
            "robot_description",
            "robot_description_semantic",
            "robot_description_kinematics",
            "robot_description_planning",
            "use_sim_time",
        ):
            with self.subTest(token=token):
                self.assertIn(token, source)

        self.assertLess(
            source.index("abb_lab_gazebo"),
            source.index("moveit_ros_move_group"),
            "the inherited Gazebo/control bringup must be constructed before move_group",
        )
        self.assertNotIn("ros2_control_node", source)
        self.assertNotRegex(
            source,
            r"package\s*=\s*['\"]controller_manager['\"]",
        )
        self.assertNotRegex(
            source,
            r"executable\s*=\s*['\"]spawner['\"]",
        )

    def test_p22_launch_uses_the_explicit_ompl_only_and_existing_jtc_contract(self) -> None:
        """No planner or controller may be silently substituted at runtime."""
        source = required_source(P22_LAUNCH_PATH)
        self.assertIn("MoveItConfigsBuilder", source)
        self.assertRegex(
            source,
            r"\.planning_pipelines\(\s*pipelines\s*=\s*\[\s*['\"]ompl['\"]\s*\]"
            r"\s*,\s*load_all\s*=\s*False\s*\)",
        )
        self.assertIn("moveit_controllers.yaml", source)

        moveit = load_yaml(CONTROLLERS_PATH)
        self.assertFalse(moveit["moveit_manage_controllers"])
        manager = moveit["moveit_simple_controller_manager"]
        self.assertEqual(manager["controller_names"], ["joint_trajectory_controller"])
        controller = manager["joint_trajectory_controller"]
        self.assertEqual(controller["type"], "FollowJointTrajectory")
        self.assertEqual(controller["action_ns"], "follow_joint_trajectory")
        self.assertEqual(controller["joints"], JOINTS)

    def test_p22_verifier_requires_real_joint_pose_and_collision_evidence(self) -> None:
        """A02 evidence must cover execution and an intentionally rejected collision."""
        source = required_source(P22_VERIFIER_PATH)
        ast.parse(source, filename=str(P22_VERIFIER_PATH))

        for token in (
            "rclpy",
            "MoveGroup",
            "JointState",
            "PoseStamped",
            "tool0",
            "CollisionObject",
            "PlanningScene",
            "ApplyPlanningScene",
            "velocity_only",
            "canonical_velocity_limits",
            "acceleration_limits_used",
            "temporary_collision_object_removed",
            "world_object_ids_after_removal",
            "math.isfinite",
            "MOVEIT_JOINT_EXECUTION=PASS",
            "MOVEIT_POSE_EXECUTION=PASS",
            "MOVEIT_HOME_POSE_HOME=PASS",
            "MOVEIT_COLLISION_REJECTION=PASS",
        ):
            with self.subTest(token=token):
                self.assertIn(token, source)

        self.assertTrue(
            any(
                token in source
                for token in ("GetStateValidity", "GetMotionPlan", "MoveItErrorCodes")
            ),
            "collision rejection must inspect a real MoveIt planning outcome",
        )
        self.assertNotIn("random", source.lower())

    def test_p22_harness_gates_a02_on_all_three_runtime_results(self) -> None:
        """The root runtime gate must not publish A02 PASS prematurely."""
        source = required_source(P22_HARNESS_PATH)
        required_tokens = (
            "set -eo pipefail",
            "setsid ros2 launch abb_lab_moveit_config irb1200_moveit_headless.launch.py",
            "verify_moveit_runtime.py",
            "MOVEIT_JOINT_EXECUTION=PASS",
            "MOVEIT_POSE_EXECUTION=PASS",
            "MOVEIT_HOME_POSE_HOME=PASS",
            "MOVEIT_COLLISION_REJECTION=PASS",
            "A02=NOT_STARTED",
            "A02=PASS",
        )
        for token in required_tokens:
            with self.subTest(token=token):
                self.assertIn(token, source)

        pass_index = source.index("A02=PASS")
        for marker in required_tokens[3:7]:
            with self.subTest(marker=marker):
                self.assertLess(
                    source.index(marker),
                    pass_index,
                    f"{marker} must be checked before A02 can pass",
                )
        self.assertIn("if", source[: pass_index + len("A02=PASS")])


if __name__ == "__main__":
    unittest.main()
