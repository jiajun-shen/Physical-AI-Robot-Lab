#!/usr/bin/env python3
"""Source-tree contract for the minimal geometry-only ROS description slice."""

from __future__ import annotations

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


DESCRIPTION_ROOT = Path(__file__).resolve().parents[1]
ROS_SOURCE_ROOT = DESCRIPTION_ROOT.parent
XACRO_NAMESPACE = "http://ros.org/wiki/xacro"


class ABBDescriptionSourceTests(unittest.TestCase):
    def test_workspace_contains_description_without_upstream_driver_packages(self) -> None:
        observed_packages: list[tuple[str, Path]] = []
        for package_xml in ROS_SOURCE_ROOT.glob("*/package.xml"):
            package_root = package_xml.parent
            package_name = ET.parse(package_xml).getroot().findtext("name")
            observed_packages.append((package_name, package_root))

        self.assertIn(("abb_lab_description", DESCRIPTION_ROOT), observed_packages)
        forbidden_package_names = {
            "abb_hardware_interface",
            "abb_bringup",
            "abb_rws_client",
            "abb_egm_rws_managers",
            "abb_ros2_msgs",
            "abb_irb1200_5_90_moveit_config",
        }
        self.assertTrue(
            forbidden_package_names.isdisjoint(name for name, _ in observed_packages)
        )

    def test_project_package_installs_only_selected_snapshot_resources(self) -> None:
        cmake_text = (DESCRIPTION_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("third_party/abb_irb1200_support", cmake_text)
        self.assertIn("common_materials.xacro", cmake_text)
        self.assertIn("common_colours.xacro", cmake_text)
        self.assertIn("meshes", cmake_text)
        self.assertIn("install(DIRECTORY urdf config third_party", cmake_text)
        self.assertNotIn("REPOSITORY_ROOT", cmake_text)
        self.assertNotIn("../../", cmake_text)
        self.assertNotIn("irb1200.ros2_control.xacro", cmake_text)
        self.assertNotIn("abb_hardware_interface", cmake_text)

    def test_package_metadata_has_no_driver_dependencies(self) -> None:
        forbidden_dependencies = {
            "abb_hardware_interface",
            "abb_bringup",
            "abb_rws_client",
            "abb_egm_rws_managers",
            "abb_ros2_msgs",
        }
        package_xml = DESCRIPTION_ROOT / "package.xml"
        root = ET.parse(package_xml).getroot()
        self.assertEqual(root.findtext("name"), "abb_lab_description")
        dependency_names = {
            element.text
            for element in root
            if element.tag.endswith("depend") and element.text is not None
        }
        self.assertTrue(forbidden_dependencies.isdisjoint(dependency_names))
        self.assertIn("Apache-2.0", {element.text for element in root.findall("license")})

    def test_limit_reference_matches_source_and_stays_non_operational(self) -> None:
        reference_path = DESCRIPTION_ROOT / "config" / "irb1200_5_90_asset_reference.yaml"
        reference = yaml.safe_load(reference_path.read_text(encoding="utf-8"))

        self.assertEqual(
            reference["source"]["commit"],
            "05cbabe774ef026b1cd36fe6b253b55f55e16340",
        )
        reference_limits = reference["joint_position_velocity_limits"]
        self.assertEqual(reference_limits["units"]["position"], "rad")
        self.assertEqual(reference_limits["units"]["velocity"], "rad_per_second")

        source_macro_path = (
            DESCRIPTION_ROOT
            / "third_party"
            / "abb_irb1200_support"
            / "source"
            / "abb_irb1200_support"
            / "urdf"
            / "irb1200_5_90_macro.xacro"
        )
        source_root = ET.parse(source_macro_path).getroot()
        source_macro = source_root.find(f"{{{XACRO_NAMESPACE}}}macro")
        source_joints = {
            joint.attrib["name"].replace("${prefix}", ""): joint
            for joint in source_macro.findall("joint")
            if joint.attrib["type"] == "revolute"
        }
        expected_joint_names = {f"joint_{index}" for index in range(1, 7)}
        self.assertSetEqual(set(source_joints), expected_joint_names)
        self.assertSetEqual(
            {key for key in reference_limits if key != "units"},
            expected_joint_names,
        )
        for joint_name in sorted(expected_joint_names):
            source_limit = source_joints[joint_name].find("limit").attrib
            recorded_limit = reference_limits[joint_name]
            for key in ("lower", "upper", "velocity"):
                self.assertAlmostEqual(
                    float(recorded_limit[key]),
                    float(source_limit[key]),
                    places=9,
                    msg=f"{joint_name}.{key}",
                )

        safety_status = reference["safety_status"]
        self.assertEqual(safety_status["operational_control_limits"], "NOT_UNIFIED")
        self.assertEqual(safety_status["command_limit_enforcement"], "NOT_TESTED")
        self.assertEqual(safety_status["upstream_effort_value"], 1000)
        self.assertIs(safety_status["upstream_effort_is_real_robot_limit"], False)
        self.assertEqual(
            safety_status["inertial_source"],
            "mesh_volume_constant_density_estimate",
        )
        self.assertIs(safety_status["real_robot_ready"], False)

    def test_geometry_wrapper_has_no_control_or_simulator_elements(self) -> None:
        wrapper_path = DESCRIPTION_ROOT / "urdf" / "irb1200_5_90_geometry_only.urdf.xacro"
        wrapper_text = wrapper_path.read_text(encoding="utf-8")
        root = ET.parse(wrapper_path).getroot()
        self.assertEqual(root.attrib["name"], "abb_irb1200_5_90_geometry_only")

        includes = root.findall(f"{{{XACRO_NAMESPACE}}}include")
        self.assertEqual(len(includes), 1)
        self.assertEqual(
            includes[0].attrib["filename"],
            "$(find abb_lab_description)/urdf/irb1200_5_90_macro.project.xacro",
        )
        macro_calls = root.findall(f"{{{XACRO_NAMESPACE}}}abb_irb1200_5_90")
        self.assertEqual(len(macro_calls), 1)

        for forbidden_token in (
            "ros2_control",
            "<gazebo",
            "plugin",
            "transmission",
            "gripper",
            "abb_hardware_interface",
        ):
            self.assertNotIn(forbidden_token, wrapper_text)

    def test_derived_macro_has_notice_and_project_local_resources(self) -> None:
        macro_path = DESCRIPTION_ROOT / "urdf" / "irb1200_5_90_macro.project.xacro"
        macro_text = macro_path.read_text(encoding="utf-8")
        pinned_macro_path = (
            DESCRIPTION_ROOT
            / "third_party"
            / "abb_irb1200_support"
            / "source"
            / "abb_irb1200_support"
            / "urdf"
            / "irb1200_5_90_macro.xacro"
        )
        pinned_macro_text = pinned_macro_path.read_text(encoding="utf-8")
        macro_notice = (
            "  <!-- Modified by Physical-AI-Robot-Lab from PickNikRobotics/abb_ros2\n"
            "       commit 05cbabe774ef026b1cd36fe6b253b55f55e16340. Only package resource\n"
            "       paths were changed; see ../NOTICE.md and the installed provenance. -->\n"
        )
        expected_macro_text = pinned_macro_text.replace(
            '<robot xmlns:xacro="http://ros.org/wiki/xacro">\n',
            '<robot xmlns:xacro="http://ros.org/wiki/xacro">\n' + macro_notice,
            1,
        )
        expected_macro_text = expected_macro_text.replace(
            "$(find abb_resources)/urdf/common_materials.xacro",
            "$(find abb_lab_description)/urdf/vendor/common_materials.project.xacro",
        ).replace(
            "package://abb_irb1200_support/meshes/",
            "package://abb_lab_description/meshes/",
        )
        self.assertEqual(macro_text, expected_macro_text)
        self.assertIn("Modified by Physical-AI-Robot-Lab", macro_text)
        self.assertIn("05cbabe774ef026b1cd36fe6b253b55f55e16340", macro_text)
        self.assertIn("$(find abb_lab_description)/urdf/vendor/common_materials.project.xacro", macro_text)
        self.assertNotIn("$(find abb_resources)", macro_text)
        self.assertEqual(macro_text.count("package://abb_lab_description/meshes/"), 14)
        self.assertNotIn("package://abb_irb1200_support/", macro_text)

        materials_path = DESCRIPTION_ROOT / "urdf" / "vendor" / "common_materials.project.xacro"
        materials_text = materials_path.read_text(encoding="utf-8")
        pinned_materials_path = (
            DESCRIPTION_ROOT
            / "third_party"
            / "abb_irb1200_support"
            / "source"
            / "abb_resources"
            / "urdf"
            / "common_materials.xacro"
        )
        pinned_materials_text = pinned_materials_path.read_text(encoding="utf-8")
        materials_notice = (
            "  <!-- Modified by Physical-AI-Robot-Lab from PickNikRobotics/abb_ros2\n"
            "       commit 05cbabe774ef026b1cd36fe6b253b55f55e16340. Only the package\n"
            "       resource path below was changed. -->\n"
        )
        expected_materials_text = pinned_materials_text.replace(
            '<robot xmlns:xacro="http://ros.org/wiki/xacro">\n',
            '<robot xmlns:xacro="http://ros.org/wiki/xacro">\n' + materials_notice,
            1,
        ).replace(
            "$(find abb_resources)/urdf/common_colours.xacro",
            "$(find abb_lab_description)/urdf/vendor/common_colours.xacro",
        )
        self.assertEqual(materials_text, expected_materials_text)
        self.assertIn("Modified by Physical-AI-Robot-Lab", materials_text)
        self.assertIn("$(find abb_lab_description)/urdf/vendor/common_colours.xacro", materials_text)
        self.assertNotIn("$(find abb_resources)", materials_text)

        project_colours = DESCRIPTION_ROOT / "urdf" / "vendor" / "common_colours.xacro"
        pinned_colours = (
            DESCRIPTION_ROOT
            / "third_party"
            / "abb_irb1200_support"
            / "source"
            / "abb_resources"
            / "urdf"
            / "common_colours.xacro"
        )
        self.assertEqual(project_colours.read_bytes(), pinned_colours.read_bytes())

    def test_no_upstream_control_or_bringup_files_entered_workspace(self) -> None:
        relative_files = {
            path.relative_to(ROS_SOURCE_ROOT).as_posix()
            for path in ROS_SOURCE_ROOT.rglob("*")
            if path.is_file()
        }
        for forbidden_fragment in (
            "irb1200.ros2_control.xacro",
            "abb_hardware_interface",
            "abb_bringup",
            "abb_rws_client",
            "abb_irb1200_5_90_moveit_config",
        ):
            self.assertFalse(
                any(forbidden_fragment in relative_path for relative_path in relative_files),
                forbidden_fragment,
            )


if __name__ == "__main__":
    unittest.main()
