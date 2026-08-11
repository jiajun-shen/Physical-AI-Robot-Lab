#!/usr/bin/env python3
"""Source contracts for the pre-runtime ABB Gazebo control description."""

from __future__ import annotations

import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


DESCRIPTION_ROOT = Path(__file__).resolve().parents[1]
CANONICAL_LIMITS_PATH = (
    DESCRIPTION_ROOT / "config" / "irb1200_5_90_control_limits.yaml"
)
ASSET_REFERENCE_PATH = (
    DESCRIPTION_ROOT / "config" / "irb1200_5_90_asset_reference.yaml"
)
SAFETY_PARAMETERS_PATH = DESCRIPTION_ROOT / "config" / "controller_manager_safety.yaml"
CONTROL_MACRO_PATH = DESCRIPTION_ROOT / "urdf" / "irb1200_5_90_gz_ros2_control.xacro"
CONTROL_DESCRIPTION_PATH = (
    DESCRIPTION_ROOT / "urdf" / "irb1200_5_90_gz_control.urdf.xacro"
)
PROJECT_GEOMETRY_MACRO_PATH = (
    DESCRIPTION_ROOT / "urdf" / "irb1200_5_90_macro.project.xacro"
)
PINNED_GEOMETRY_MACRO_PATH = (
    DESCRIPTION_ROOT
    / "third_party"
    / "abb_irb1200_support"
    / "source"
    / "abb_irb1200_support"
    / "urdf"
    / "irb1200_5_90_macro.xacro"
)

PINNED_COMMIT = "05cbabe774ef026b1cd36fe6b253b55f55e16340"
XACRO_NAMESPACE = "http://ros.org/wiki/xacro"
EXPECTED_JOINT_NAMES = {f"joint_{index}" for index in range(1, 7)}


def required_yaml(path: Path) -> dict:
    if not path.is_file():
        raise AssertionError(f"required YAML is missing: {path}")
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def required_xml(path: Path) -> ET.Element:
    if not path.is_file():
        raise AssertionError(f"required Xacro is missing: {path}")
    return ET.parse(path).getroot()


class ABBControlDescriptionSourceTests(unittest.TestCase):
    def test_p1_3a_files_exist(self) -> None:
        required_paths = {
            CANONICAL_LIMITS_PATH,
            SAFETY_PARAMETERS_PATH,
            CONTROL_MACRO_PATH,
            CONTROL_DESCRIPTION_PATH,
        }
        for path in sorted(required_paths):
            with self.subTest(path=path.name):
                self.assertTrue(path.is_file(), path)

    def test_canonical_limits_preserve_pinned_source_values(self) -> None:
        canonical = required_yaml(CANONICAL_LIMITS_PATH)
        asset_reference = required_yaml(ASSET_REFERENCE_PATH)
        self.assertEqual(canonical["schema_version"], 1)
        self.assertEqual(canonical["scope"], "SIMULATION_ONLY")
        self.assertEqual(canonical["source"]["commit"], PINNED_COMMIT)
        self.assertEqual(
            canonical["source"]["asset_reference"], ASSET_REFERENCE_PATH.name
        )
        self.assertEqual(
            canonical["units"],
            {"position": "rad", "velocity": "rad_per_second"},
        )
        limits = canonical["joint_limits"]
        self.assertSetEqual(set(limits), EXPECTED_JOINT_NAMES)
        reference_limits = asset_reference["joint_position_velocity_limits"]
        self.assertEqual(
            canonical["units"],
            reference_limits["units"],
        )

        pinned_root = ET.parse(PINNED_GEOMETRY_MACRO_PATH).getroot()
        pinned_macro = pinned_root.find(f"{{{XACRO_NAMESPACE}}}macro")
        pinned_joints = {
            joint.attrib["name"].replace("${prefix}", ""): joint
            for joint in pinned_macro.findall("joint")
            if joint.attrib["type"] == "revolute"
        }
        self.assertSetEqual(set(pinned_joints), EXPECTED_JOINT_NAMES)

        for joint_name in sorted(EXPECTED_JOINT_NAMES):
            source_limit = pinned_joints[joint_name].find("limit").attrib
            canonical_limit = limits[joint_name]
            self.assertIs(canonical_limit["has_position_limits"], True)
            self.assertIs(canonical_limit["has_velocity_limits"], True)
            self.assertNotIn("has_acceleration_limits", canonical_limit)
            self.assertNotIn("max_acceleration", canonical_limit)
            self.assertTrue(
                all(
                    math.isfinite(float(canonical_limit[key]))
                    for key in ("min_position", "max_position", "max_velocity")
                )
            )
            self.assertLess(
                canonical_limit["min_position"], canonical_limit["max_position"]
            )
            self.assertGreater(canonical_limit["max_velocity"], 0.0)
            key_mapping = {
                "min_position": "lower",
                "max_position": "upper",
                "max_velocity": "velocity",
            }
            for canonical_key, source_key in key_mapping.items():
                with self.subTest(joint=joint_name, limit=canonical_key):
                    self.assertAlmostEqual(
                        canonical_limit[canonical_key],
                        float(source_limit[source_key]),
                        places=9,
                    )
                    self.assertAlmostEqual(
                        canonical_limit[canonical_key],
                        float(reference_limits[joint_name][source_key]),
                        places=9,
                    )

        safety_status = canonical["safety_status"]
        self.assertEqual(
            safety_status["simulation_description_limits"],
            "RECONCILED_WITH_PINNED_URDF",
        )
        self.assertEqual(
            safety_status["command_limit_enforcement"],
            "CONFIGURED_TRUE_RUNTIME_UNVERIFIED",
        )
        self.assertIs(safety_status["real_robot_ready"], False)
        self.assertEqual(safety_status["upstream_effort"], "FICTIONAL_NOT_APPROVED")
        for derivative in ("acceleration", "deceleration", "jerk"):
            self.assertEqual(
                safety_status[derivative], "NOT_DEFINED_NOT_VERIFIED"
            )

    def test_controller_manager_enforcement_is_the_only_runtime_parameter(self) -> None:
        parameters = required_yaml(SAFETY_PARAMETERS_PATH)
        self.assertSetEqual(set(parameters), {"controller_manager"})
        ros_parameters = parameters["controller_manager"]["ros__parameters"]
        self.assertDictEqual(ros_parameters, {"enforce_command_limits": True})

    def test_geometry_macro_remains_the_pinned_geometry_contract(self) -> None:
        root = required_xml(PROJECT_GEOMETRY_MACRO_PATH)
        macro = root.find(f"{{{XACRO_NAMESPACE}}}macro")
        self.assertEqual(macro.attrib["name"], "abb_irb1200_5_90")
        self.assertEqual(macro.attrib["params"].split(), ["prefix"])

        revolute_joints = {
            joint.attrib["name"].replace("${prefix}", ""): joint
            for joint in macro.findall("joint")
            if joint.attrib["type"] == "revolute"
        }
        self.assertSetEqual(set(revolute_joints), EXPECTED_JOINT_NAMES)
        limits = required_yaml(CANONICAL_LIMITS_PATH)["joint_limits"]
        for joint_name, joint in revolute_joints.items():
            limit = joint.find("limit").attrib
            self.assertAlmostEqual(
                float(limit["lower"]), limits[joint_name]["min_position"]
            )
            self.assertAlmostEqual(
                float(limit["upper"]), limits[joint_name]["max_position"]
            )
            self.assertAlmostEqual(
                float(limit["velocity"]), limits[joint_name]["max_velocity"]
            )
            self.assertEqual(limit["effort"], "1000")

    def test_ros2_control_macro_has_exactly_six_position_control_joints(self) -> None:
        root = required_xml(CONTROL_MACRO_PATH)
        macro = root.find(f"{{{XACRO_NAMESPACE}}}macro")
        self.assertEqual(macro.attrib["name"], "abb_irb1200_5_90_gz_ros2_control")
        self.assertEqual(
            macro.attrib["params"].split(), ["name", "prefix", "joint_limits"]
        )

        control = macro.find("ros2_control")
        self.assertEqual(control.attrib, {"name": "${name}", "type": "system"})
        self.assertEqual(
            control.findtext("hardware/plugin"),
            "gz_ros2_control/GazeboSimSystem",
        )
        joints = control.findall("joint")
        self.assertEqual(len(joints), 6)
        self.assertSetEqual(
            {joint.attrib["name"].replace("${prefix}", "") for joint in joints},
            EXPECTED_JOINT_NAMES,
        )
        for joint in joints:
            joint_name = joint.attrib["name"].replace("${prefix}", "")
            command_interfaces = joint.findall("command_interface")
            self.assertEqual(
                [entry.attrib["name"] for entry in command_interfaces],
                ["position"],
            )
            parameter_entries = command_interfaces[0].findall("param")
            self.assertEqual(len(parameter_entries), 2)
            self.assertSetEqual(
                {entry.attrib["name"] for entry in parameter_entries},
                {"min", "max"},
            )
            parameters = {
                entry.attrib["name"]: entry.text
                for entry in parameter_entries
            }
            self.assertDictEqual(
                parameters,
                {
                    "min": f"${{joint_limits['{joint_name}']['min_position']}}",
                    "max": f"${{joint_limits['{joint_name}']['max_position']}}",
                },
            )
            self.assertEqual(
                [entry.attrib["name"] for entry in joint.findall("state_interface")],
                ["position", "velocity", "effort"],
            )
            self.assertEqual(joint.findall(".//param[@name='initial_value']"), [])
            self.assertEqual(joint.findall(".//limits[@enable='false']"), [])

    def test_full_description_has_fixed_world_and_harmonic_plugin(self) -> None:
        root = required_xml(CONTROL_DESCRIPTION_PATH)
        self.assertEqual(root.attrib["name"], "abb_irb1200_5_90_gz_control")
        arguments = {
            element.attrib["name"]: element.attrib["default"]
            for element in root.findall(f"{{{XACRO_NAMESPACE}}}arg")
        }
        self.assertDictEqual(
            arguments,
            {"prefix": "", "controller_config_file": ""},
        )
        self.assertEqual({link.attrib["name"] for link in root.findall("link")}, {"world"})

        world_joint = root.find("joint")
        self.assertEqual(
            world_joint.attrib,
            {"name": "${prefix}world_to_base", "type": "fixed"},
        )
        self.assertEqual(world_joint.find("parent").attrib["link"], "world")
        self.assertEqual(
            world_joint.find("child").attrib["link"], "${prefix}base_link"
        )
        self.assertEqual(
            world_joint.find("origin").attrib,
            {"xyz": "0 0 0", "rpy": "0 0 0"},
        )

        includes = {
            include.attrib["filename"]
            for include in root.findall(f"{{{XACRO_NAMESPACE}}}include")
        }
        self.assertSetEqual(
            includes,
            {
                "$(find abb_lab_description)/urdf/irb1200_5_90_macro.project.xacro",
                "$(find abb_lab_description)/urdf/irb1200_5_90_gz_ros2_control.xacro",
            },
        )
        properties = {
            element.attrib["name"]: element.attrib["value"]
            for element in root.findall(f"{{{XACRO_NAMESPACE}}}property")
        }
        self.assertSetEqual(
            set(properties),
            {"prefix", "controller_config_file", "joint_limits"},
        )
        self.assertEqual(
            properties["controller_config_file"],
            "$(arg controller_config_file)",
        )
        self.assertIn("irb1200_5_90_control_limits.yaml", properties["joint_limits"])

        geometry_call = root.find(f"{{{XACRO_NAMESPACE}}}abb_irb1200_5_90")
        control_call = root.find(
            f"{{{XACRO_NAMESPACE}}}abb_irb1200_5_90_gz_ros2_control"
        )
        self.assertEqual(geometry_call.attrib, {"prefix": "${prefix}"})
        self.assertEqual(
            control_call.attrib,
            {
                "name": "GazeboSimSystem",
                "prefix": "${prefix}",
                "joint_limits": "${joint_limits}",
            },
        )

        gazebo_plugins = root.findall("gazebo/plugin")
        self.assertEqual(len(gazebo_plugins), 1)
        self.assertEqual(
            gazebo_plugins[0].attrib["filename"], "gz_ros2_control-system"
        )
        self.assertEqual(
            gazebo_plugins[0].attrib["name"],
            "gz_ros2_control::GazeboSimROS2ControlPlugin",
        )
        self.assertEqual(
            gazebo_plugins[0].findtext("parameters"),
            "$(find abb_lab_description)/config/controller_manager_safety.yaml",
        )
        runtime_parameters = gazebo_plugins[0].findall(
            f"{{{XACRO_NAMESPACE}}}if"
        )
        self.assertEqual(len(runtime_parameters), 1)
        self.assertEqual(
            runtime_parameters[0].attrib,
            {"value": "${controller_config_file != ''}"},
        )
        self.assertEqual(
            runtime_parameters[0].findtext("parameters"),
            "$(arg controller_config_file)",
        )
        self.assertEqual(gazebo_plugins[0].findtext("hold_joints"), "true")
        self.assertEqual(root.findall(".//limits[@enable='false']"), [])

    def test_control_slice_excludes_later_phase_and_real_driver_content(self) -> None:
        files = [
            CANONICAL_LIMITS_PATH,
            SAFETY_PARAMETERS_PATH,
            CONTROL_MACRO_PATH,
            CONTROL_DESCRIPTION_PATH,
        ]
        combined = "\n".join(
            path.read_text(encoding="utf-8") for path in files if path.is_file()
        )
        for forbidden_token in (
            "joint_trajectory_controller",
            "joint_state_broadcaster",
            "moveit",
            "gripper",
            "abb_hardware_interface",
            "abbsystemhardware",
            "abb_rws",
            "abb_egm",
        ):
            self.assertNotIn(forbidden_token, combined.lower())

    def test_package_declares_pre_runtime_control_dependencies(self) -> None:
        root = ET.parse(DESCRIPTION_ROOT / "package.xml").getroot()
        exec_dependencies = {
            element.text for element in root.findall("exec_depend")
        }
        self.assertTrue(
            {
                "controller_manager",
                "gz_ros2_control",
                "python3-yaml",
                "ros_gz_sim",
                "xacro",
            }
            <= exec_dependencies
        )
        self.assertIn(
            "urdf", {element.text for element in root.findall("test_depend")}
        )
        gazebo_exports = root.findall("export/gazebo_ros")
        self.assertEqual(len(gazebo_exports), 1)
        self.assertEqual(
            gazebo_exports[0].attrib,
            {"gazebo_model_path": "${prefix}/../"},
        )


if __name__ == "__main__":
    unittest.main()
