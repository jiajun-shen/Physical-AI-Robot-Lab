"""Source and expanded-URDF contracts for the isolated ABB gripper profile."""

from __future__ import annotations

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
URDF_DIR = PACKAGE_ROOT / "urdf"
CONFIG_DIR = PACKAGE_ROOT / "config"
PROFILE_XACRO = URDF_DIR / "irb1200_5_90_gz_gripper_control.urdf.xacro"
DRAWING_PROFILE_XACRO = URDF_DIR / "irb1200_5_90_gz_drawing_pen_control.urdf.xacro"
CONTROLLERS_YAML = CONFIG_DIR / "irb1200_gripper_controllers.yaml"
LIMITS_YAML = CONFIG_DIR / "two_finger_gripper_limits.yaml"
SAFETY_YAML = (
    PACKAGE_ROOT.parent
    / "abb_lab_description"
    / "config"
    / "controller_manager_safety.yaml"
)

ARM_JOINTS = tuple(f"joint_{index}" for index in range(1, 7))
RIGHT_FINGER_JOINT = "two_finger_right_joint"
LEFT_FINGER_JOINT = "two_finger_left_joint"


def _expanded_robot() -> ET.Element:
    completed = subprocess.run(
        ["xacro", str(PROFILE_XACRO)],
        check=True,
        text=True,
        capture_output=True,
    )
    return ET.fromstring(completed.stdout)


def _expanded_drawing_robot() -> ET.Element:
    completed = subprocess.run(
        ["xacro", str(DRAWING_PROFILE_XACRO)],
        check=True,
        text=True,
        capture_output=True,
    )
    return ET.fromstring(completed.stdout)


def _load_yaml(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream)
    assert isinstance(loaded, dict)
    return loaded


def test_profile_is_a_new_self_contained_package() -> None:
    assert PROFILE_XACRO.is_file()
    assert CONTROLLERS_YAML.is_file()
    assert LIMITS_YAML.is_file()
    assert DRAWING_PROFILE_XACRO.is_file()
    assert not (PACKAGE_ROOT / "launch").exists()
    assert not (PACKAGE_ROOT / "scripts").exists()

    profile_text = PROFILE_XACRO.read_text(encoding="utf-8")
    assert "$(find abb_lab_description)/urdf/irb1200_5_90_macro.project.xacro" in profile_text
    assert "$(find abb_lab_control)" not in profile_text
    assert "$(find abb_lab_gazebo)" not in profile_text
    assert "irb1200_5_90_gz_control.urdf.xacro" not in profile_text


def test_profile_has_no_embedded_attachment_and_declares_p5_dynamic_guard() -> None:
    prohibited_mechanisms = (
        "gazebo_ros_link_attacher",
        "gazebo_grasp_plugin",
        "gazebo_grasp_fix",
        "grasp_fix",
        "attachment_plugin",
        "ros2_control_demo_hardware",
        "mock_components",
        "fake_components",
        "fake_system",
    )
    profile_text = "\n".join(
        path.read_text(encoding="utf-8").lower()
        for path in (*sorted(URDF_DIR.glob("*.xacro")), *sorted(CONFIG_DIR.glob("*.yaml")))
    )
    assert not any(token in profile_text for token in prohibited_mechanisms)

    default_robot = _expanded_robot()
    assert default_robot.findall("./gazebo/plugin[@name='gz::sim::systems::DetachableJoint']") == []

    limits = _load_yaml(LIMITS_YAML)
    assert limits["scope"] == "SIMULATION_ONLY"
    assert limits["safety_status"]["object_attachment"] == "PHASE5_GUARDED_SIMULATION_ONLY"
    assert limits["safety_status"]["force_control"] == "NOT_CONFIGURED"
    assert limits["safety_status"]["real_robot_ready"] is False
    phase5 = limits["phase5_simulation_grasp"]
    assert phase5["mechanism"] == "gz_sim_detachable_joint"
    assert phase5["activation"] == "dynamic_entity_system_add_after_guard"
    assert phase5["detach_topic"] == "/phase5/grasp/detach"
    assert phase5["state_topic"] == "/phase5/grasp/state"
    assert 0.0 < phase5["pinch_envelope_position"] <= 0.040


def test_expanded_profile_has_one_combined_gazebo_system_and_expected_interfaces() -> None:
    robot = _expanded_robot()
    systems = robot.findall("ros2_control")
    assert len(systems) == 1
    system = systems[0]
    assert system.attrib == {"name": "GazeboSimSystem", "type": "system"}
    assert [plugin.text.strip() for plugin in system.findall("./hardware/plugin")] == [
        "gz_ros2_control/GazeboSimSystem"
    ]

    safety = _load_yaml(SAFETY_YAML)
    assert safety["controller_manager"]["ros__parameters"]["enforce_command_limits"] is True
    plugin_parameters = [
        parameter.text.strip() for parameter in robot.findall("./gazebo/plugin/parameters")
    ]
    assert len(plugin_parameters) == 2
    assert plugin_parameters[0].endswith("abb_lab_description/config/controller_manager_safety.yaml")
    assert plugin_parameters[1].endswith("abb_lab_gripper/config/irb1200_gripper_controllers.yaml")
    plugin = robot.find("./gazebo/plugin")
    assert plugin is not None
    assert plugin.findtext("hold_joints") == "true"
    assert plugin.findtext("position_proportional_gain") == "1.0"

    controlled_joints = {joint.attrib["name"]: joint for joint in system.findall("joint")}
    assert set(controlled_joints) == {*ARM_JOINTS, RIGHT_FINGER_JOINT, LEFT_FINGER_JOINT}
    for name in (*ARM_JOINTS, RIGHT_FINGER_JOINT):
        assert [interface.attrib["name"] for interface in controlled_joints[name].findall("command_interface")] == [
            "position"
        ]
        state_interfaces = [interface.attrib["name"] for interface in controlled_joints[name].findall("state_interface")]
        assert "position" in state_interfaces
        assert "velocity" in state_interfaces
    right_position_state = next(
        interface
        for interface in controlled_joints[RIGHT_FINGER_JOINT].findall("state_interface")
        if interface.attrib["name"] == "position"
    )
    initial_value = right_position_state.find("param")
    assert initial_value is not None
    assert initial_value.attrib == {"name": "initial_value"}
    assert float(initial_value.text) == 0.020
    assert controlled_joints[LEFT_FINGER_JOINT].findall("command_interface") == []
    assert [interface.attrib["name"] for interface in controlled_joints[LEFT_FINGER_JOINT].findall("state_interface")] == [
        "position",
        "velocity",
    ]


def test_gripper_is_fixed_to_tool0_with_a_fixed_tcp_and_urdf_mimic_follower() -> None:
    robot = _expanded_robot()
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}

    mount = joints["tool0_to_gripper_base"]
    assert mount.attrib["type"] == "fixed"
    assert mount.find("parent").attrib["link"] == "tool0"
    assert mount.find("child").attrib["link"] == "gripper_base"

    tcp = joints["gripper_base_to_gripper_tcp"]
    assert tcp.attrib["type"] == "fixed"
    assert tcp.find("parent").attrib["link"] == "gripper_base"
    assert tcp.find("child").attrib["link"] == "gripper_tcp"

    right = joints[RIGHT_FINGER_JOINT]
    assert right.attrib["type"] == "prismatic"
    assert right.find("mimic") is None

    left = joints[LEFT_FINGER_JOINT]
    assert left.attrib["type"] == "prismatic"
    mimic = left.find("mimic")
    assert mimic is not None
    assert mimic.attrib == {"joint": RIGHT_FINGER_JOINT, "multiplier": "-1", "offset": "0"}


def test_gripper_palm_collision_clears_both_finger_rails_for_the_full_stroke() -> None:
    """The prismatic stroke is along y, so x clearance proves no palm overlap."""

    robot = _expanded_robot()
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    links = {link.attrib["name"]: link for link in robot.findall("link")}

    def x_interval(link_name: str, parent_joint_name: str | None = None) -> tuple[float, float]:
        link = links[link_name]
        collision = link.find("collision")
        assert collision is not None
        collision_origin = collision.find("origin")
        assert collision_origin is not None
        collision_x = float(collision_origin.attrib["xyz"].split()[0])
        size = collision.find("geometry/box")
        assert size is not None
        width_x = float(size.attrib["size"].split()[0])
        parent_x = 0.0
        if parent_joint_name is not None:
            joint_origin = joints[parent_joint_name].find("origin")
            assert joint_origin is not None
            parent_x = float(joint_origin.attrib["xyz"].split()[0])
        center_x = parent_x + collision_x
        return center_x - width_x / 2.0, center_x + width_x / 2.0

    palm_min_x, palm_max_x = x_interval("gripper_base")
    assert palm_min_x == 0.0
    for finger_joint_name, finger_link_name in (
        (RIGHT_FINGER_JOINT, "two_finger_right_link"),
        (LEFT_FINGER_JOINT, "two_finger_left_link"),
    ):
        finger_min_x, _ = x_interval(finger_link_name, finger_joint_name)
        # x is invariant for both prismatic strokes; retain at least 10 mm
        # of collision clearance over every commanded aperture.
        assert finger_min_x - palm_max_x >= 0.010


def test_controller_profile_uses_supported_jazzy_controller_types_and_action_name() -> None:
    controllers = _load_yaml(CONTROLLERS_YAML)
    manager = controllers["controller_manager"]["ros__parameters"]
    assert manager["joint_state_broadcaster"]["type"] == "joint_state_broadcaster/JointStateBroadcaster"
    assert manager["joint_trajectory_controller"]["type"] == "joint_trajectory_controller/JointTrajectoryController"
    assert manager["two_finger_gripper_controller"]["type"] == (
        "parallel_gripper_action_controller/GripperActionController"
    )

    arm = controllers["joint_trajectory_controller"]["ros__parameters"]
    assert arm["joints"] == list(ARM_JOINTS)
    assert arm["command_interfaces"] == ["position"]
    assert arm["state_interfaces"] == ["position", "velocity"]

    gripper = controllers["two_finger_gripper_controller"]["ros__parameters"]
    assert gripper["joint"] == RIGHT_FINGER_JOINT
    assert gripper["state_interfaces"] == ["position", "velocity"]
    assert gripper["allow_stalling"] is False
    config_text = CONTROLLERS_YAML.read_text(encoding="utf-8")
    assert "/two_finger_gripper_controller/gripper_cmd" in config_text


def test_gripper_initial_and_safe_positions_are_bounded_by_the_only_actuated_joint_limit() -> None:
    limits = _load_yaml(LIMITS_YAML)
    joint = limits["joint_limits"][RIGHT_FINGER_JOINT]
    lower = joint["min_position"]
    upper = joint["max_position"]
    assert joint["has_position_limits"] is True
    assert joint["has_velocity_limits"] is True
    assert lower == 0.0
    assert upper > lower
    assert joint["max_velocity"] > 0.0

    initial = limits["initial_positions"][RIGHT_FINGER_JOINT]
    safe = limits["safe_commands"]
    assert lower <= initial <= upper
    assert initial == 0.020
    assert lower <= safe["closed_clearance_position"] <= upper
    assert lower <= safe["open_position"] <= upper
    assert safe["open_position"] > safe["closed_clearance_position"]


def test_phase6_drawing_profile_has_a_collision_bearing_tip_without_new_control_interfaces() -> None:
    robot = _expanded_drawing_robot()
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    assert {"drawing_pen_shaft", "drawing_tip"}.issubset(links)
    shaft = links["drawing_pen_shaft"]
    cylinder = shaft.find("collision/geometry/cylinder")
    assert cylinder is not None
    assert float(cylinder.attrib["radius"]) == 0.004
    assert float(cylinder.attrib["length"]) == 0.085
    assert joints["gripper_tcp_to_drawing_pen_shaft"].find("parent").attrib["link"] == "gripper_tcp"
    assert joints["drawing_pen_shaft_to_drawing_tip"].find("child").attrib["link"] == "drawing_tip"
    assert len(robot.findall("ros2_control")) == 1
