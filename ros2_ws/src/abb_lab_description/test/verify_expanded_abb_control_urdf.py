#!/usr/bin/env python3
"""Verify expanded P1.3a control-description structure without running Gazebo."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import yaml


EXPECTED_JOINT_NAMES = {f"joint_{index}" for index in range(1, 7)}
EXPECTED_CONTROL_PLUGIN = "gz_ros2_control/GazeboSimSystem"
MINIMUM_CONTROLLER_MANAGER_VERSION = (4, 45, 2)
MINIMUM_GZ_ROS2_CONTROL_VERSION = (1, 2, 19)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def as_finite(value: str, label: str) -> float:
    parsed = float(value)
    require(math.isfinite(parsed), f"{label} is not finite")
    return parsed


def installed_package_version(package_name: str) -> tuple[str, tuple[int, int, int]]:
    manifest = Path(get_package_share_directory(package_name)) / "package.xml"
    require(manifest.is_file(), f"missing {package_name} manifest: {manifest}")
    version = ET.parse(manifest).findtext("version")
    require(version is not None, f"{package_name} version is missing")
    fields = version.split(".")
    require(
        len(fields) == 3 and all(field.isdigit() for field in fields),
        f"unexpected {package_name} version: {version}",
    )
    return version, tuple(int(field) for field in fields)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf", type=Path)
    arguments = parser.parse_args()

    urdf_text = arguments.urdf.read_text(encoding="utf-8")
    root = ET.fromstring(urdf_text)
    require(
        root.attrib.get("name") == "abb_irb1200_5_90_gz_control",
        "unexpected robot name",
    )

    link_elements = root.findall("link")
    links = {link.attrib["name"] for link in link_elements}
    require(len(link_elements) == 11, f"expected 11 link entries, observed {len(link_elements)}")
    require(len(links) == 11, f"expected 11 links, observed {len(links)}")
    require("world" in links and "base_link" in links, "world/base_link missing")

    joints = root.findall("joint")
    require(len(joints) == 10, f"expected 10 kinematic joints, observed {len(joints)}")
    world_joints = [
        joint for joint in joints if joint.attrib["name"] == "world_to_base"
    ]
    require(
        len(world_joints) == 1,
        f"expected one world_to_base, observed {len(world_joints)}",
    )
    world_joint = world_joints[0]
    require(world_joint.attrib["type"] == "fixed", "world_to_base is not fixed")
    require(world_joint.find("parent").attrib["link"] == "world", "world_to_base parent")
    require(world_joint.find("child").attrib["link"] == "base_link", "world_to_base child")
    require(
        world_joint.find("origin").attrib == {"xyz": "0 0 0", "rpy": "0 0 0"},
        "world_to_base origin",
    )

    geometry_joints = {
        joint.attrib["name"]: joint
        for joint in joints
        if joint.attrib["type"] == "revolute"
    }
    require(set(geometry_joints) == EXPECTED_JOINT_NAMES, "unexpected revolute joints")

    share_root = Path(get_package_share_directory("abb_lab_description"))
    canonical_path = share_root / "config" / "irb1200_5_90_control_limits.yaml"
    safety_path = share_root / "config" / "controller_manager_safety.yaml"
    require(canonical_path.is_file(), f"missing installed limits: {canonical_path}")
    require(safety_path.is_file(), f"missing installed safety parameters: {safety_path}")
    installed_manifest = ET.parse(share_root / "package.xml")
    gazebo_exports = installed_manifest.findall("export/gazebo_ros")
    require(len(gazebo_exports) == 1, "expected one installed gazebo_ros export")
    require(
        gazebo_exports[0].attrib == {"gazebo_model_path": "${prefix}/../"},
        "Gazebo model resource export mismatch",
    )
    gazebo_resource_root = Path(
        gazebo_exports[0].attrib["gazebo_model_path"].replace(
            "${prefix}", str(share_root)
        )
    ).resolve()
    require(
        gazebo_resource_root == share_root.parent,
        "Gazebo model resource export does not resolve to the share root",
    )
    canonical = yaml.safe_load(canonical_path.read_text(encoding="utf-8"))
    canonical_limits = canonical["joint_limits"]
    require(set(canonical_limits) == EXPECTED_JOINT_NAMES, "canonical joint set mismatch")

    control_blocks = root.findall("ros2_control")
    require(
        len(control_blocks) == 1,
        f"expected one ros2_control block, observed {len(control_blocks)}",
    )
    control = control_blocks[0]
    require(control.attrib["name"] == "GazeboSimSystem", "ros2_control name mismatch")
    require(control.attrib["type"] == "system", "ros2_control type is not system")
    require(
        control.findtext("hardware/plugin") == EXPECTED_CONTROL_PLUGIN,
        "hardware plugin mismatch",
    )
    control_joints = {joint.attrib["name"]: joint for joint in control.findall("joint")}
    require(set(control_joints) == EXPECTED_JOINT_NAMES, "control joint set mismatch")

    for joint_name in sorted(EXPECTED_JOINT_NAMES):
        expected = canonical_limits[joint_name]
        geometry_limit = geometry_joints[joint_name].find("limit").attrib
        observed_geometry = {
            "min_position": as_finite(geometry_limit["lower"], f"{joint_name}.lower"),
            "max_position": as_finite(geometry_limit["upper"], f"{joint_name}.upper"),
            "max_velocity": as_finite(geometry_limit["velocity"], f"{joint_name}.velocity"),
        }
        for key, value in observed_geometry.items():
            require(
                value == float(expected[key]),
                f"{joint_name}.{key} differs from canonical map",
            )

        control_joint = control_joints[joint_name]
        command_interfaces = control_joint.findall("command_interface")
        require(len(command_interfaces) == 1, f"{joint_name}: command interface count")
        require(command_interfaces[0].attrib["name"] == "position", f"{joint_name}: command type")
        parameter_entries = command_interfaces[0].findall("param")
        require(len(parameter_entries) == 2, f"{joint_name}: command parameter count")
        require(
            {parameter.attrib["name"] for parameter in parameter_entries}
            == {"min", "max"},
            f"{joint_name}: command parameter names",
        )
        command_parameters = {
            parameter.attrib["name"]: as_finite(
                parameter.text, f"{joint_name}.command.{parameter.attrib['name']}"
            )
            for parameter in parameter_entries
        }
        require(
            command_parameters
            == {
                "min": float(expected["min_position"]),
                "max": float(expected["max_position"]),
            },
            f"{joint_name}: command limits differ from canonical map",
        )
        state_names = [entry.attrib["name"] for entry in control_joint.findall("state_interface")]
        require(
            state_names == ["position", "velocity", "effort"],
            f"{joint_name}: state interfaces",
        )
        require(
            not control_joint.findall(".//param[@name='initial_value']"),
            f"{joint_name}: invented initial value",
        )

    safety_parameters = yaml.safe_load(safety_path.read_text(encoding="utf-8"))
    require(
        safety_parameters
        == {"controller_manager": {"ros__parameters": {"enforce_command_limits": True}}},
        "controller_manager safety file contains unexpected or disabled parameters",
    )
    require(
        canonical["safety_status"]["command_limit_enforcement"]
        == "CONFIGURED_TRUE_RUNTIME_UNVERIFIED",
        "runtime-test truth label is missing",
    )

    gazebo_plugins = root.findall("gazebo/plugin")
    require(
        len(gazebo_plugins) == 1,
        f"expected one Gazebo plugin, observed {len(gazebo_plugins)}",
    )
    gazebo_plugin = gazebo_plugins[0]
    require(
        gazebo_plugin.attrib["filename"] == "gz_ros2_control-system",
        "Gazebo plugin filename",
    )
    require(
        gazebo_plugin.attrib["name"] == "gz_ros2_control::GazeboSimROS2ControlPlugin",
        "Gazebo plugin class",
    )
    require(
        gazebo_plugin.findtext("parameters")
        == str(safety_path),
        "Gazebo plugin parameter path",
    )
    require(
        gazebo_plugin.findtext("hold_joints") == "true",
        "hold_joints is not true",
    )
    require(
        not root.findall(".//limits[@enable='false']"),
        "command limits explicitly disabled",
    )

    plugin_manifest = (
        Path(get_package_share_directory("gz_ros2_control")) / "gz_hardware_plugins.xml"
    )
    require(
        plugin_manifest.is_file(),
        f"missing installed plugin manifest: {plugin_manifest}",
    )
    plugin_root = ET.parse(plugin_manifest).getroot()
    installed_plugins = {entry.attrib["name"] for entry in plugin_root.findall("class")}
    require(
        EXPECTED_CONTROL_PLUGIN in installed_plugins,
        "GazeboSimSystem plugin is not installed",
    )
    plugin_library = (
        Path(get_package_prefix("gz_ros2_control"))
        / "lib"
        / "libgz_ros2_control-system.so"
    )
    require(plugin_library.is_file(), f"missing Gazebo system plugin: {plugin_library}")
    plugin_inspection = subprocess.run(
        ["gz", "plugin", "-i", "-p", str(plugin_library)],
        check=False,
        capture_output=True,
        text=True,
    )
    require(
        plugin_inspection.returncode == 0,
        f"Gazebo system plugin inspection failed: {plugin_inspection.stderr.strip()}",
    )
    require(
        "gz_ros2_control::GazeboSimROS2ControlPlugin"
        in plugin_inspection.stdout + plugin_inspection.stderr,
        "Gazebo system plugin class is not registered",
    )

    controller_manager_version, controller_manager_numeric = installed_package_version(
        "controller_manager"
    )
    require(
        controller_manager_numeric >= MINIMUM_CONTROLLER_MANAGER_VERSION,
        "controller_manager predates the audited command-enforcement parameter",
    )
    gz_ros2_control_version, gz_ros2_control_numeric = installed_package_version(
        "gz_ros2_control"
    )
    require(
        gz_ros2_control_numeric >= MINIMUM_GZ_ROS2_CONTROL_VERSION,
        "gz_ros2_control predates the audited Harmonic plugin contract",
    )

    for forbidden_token in (
        "joint_trajectory_controller",
        "joint_state_broadcaster",
        "moveit",
        "gripper",
        "ABBSystemHardware",
        "abb_hardware_interface",
        "abb_rws",
        "abb_egm",
    ):
        require(
            forbidden_token.lower() not in urdf_text.lower(),
            f"out-of-scope token: {forbidden_token}",
        )

    print("ABB_CONTROL_DESCRIPTION=PASS")
    print(f"LINK_COUNT={len(links)}")
    print(f"KINEMATIC_JOINT_COUNT={len(joints)}")
    print(f"CONTROL_JOINT_COUNT={len(control_joints)}")
    print(f"CANONICAL_LIMIT_JOINT_COUNT={len(canonical_limits)}")
    print(f"CONTROLLER_MANAGER_VERSION={controller_manager_version}")
    print(f"GZ_ROS2_CONTROL_VERSION={gz_ros2_control_version}")
    print("GAZEBO_MODEL_RESOURCE_EXPORT=PASS")
    print("GAZEBO_SYSTEM_PLUGIN_INSPECTION=PASS")
    print("COMMAND_LIMIT_ENFORCEMENT_CONFIGURED=true")
    print("RUNTIME_ENFORCEMENT_VERIFIED=false")
    print("GAZEBO_RUNTIME=NOT_RUN")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (
        AssertionError,
        ET.ParseError,
        KeyError,
        OSError,
        TypeError,
        ValueError,
        yaml.YAMLError,
    ) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
