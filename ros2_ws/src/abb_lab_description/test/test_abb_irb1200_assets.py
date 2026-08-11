#!/usr/bin/env python3
"""Structural and provenance tests for the pinned ABB IRB1200 asset snapshot."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET


DESCRIPTION_ROOT = Path(__file__).resolve().parents[1]
SNAPSHOT_ROOT = DESCRIPTION_ROOT / "third_party" / "abb_irb1200_support"
SOURCE_ROOT = SNAPSHOT_ROOT / "source"
SUPPORT_ROOT = SOURCE_ROOT / "abb_irb1200_support"
MACRO_PATH = SUPPORT_ROOT / "urdf" / "irb1200_5_90_macro.xacro"
HASH_LOCK_PATH = DESCRIPTION_ROOT / "config" / "abb-irb1200-assets.sha256"
REPOS_PATH = DESCRIPTION_ROOT / "config" / "abb-model-assets.repos"

PINNED_COMMIT = "05cbabe774ef026b1cd36fe6b253b55f55e16340"
XACRO_NAMESPACE = "http://ros.org/wiki/xacro"
PREFIX_TOKEN = "${prefix}"

EXPECTED_SOURCE_FILES = {
    "abb_resources/urdf/common_colours.xacro",
    "abb_resources/urdf/common_materials.xacro",
    "abb_irb1200_support/package.xml",
    "abb_irb1200_support/urdf/irb1200_5_90_macro.xacro",
    *{
        f"abb_irb1200_support/meshes/irb1200_5_90/{kind}/{link}.{extension}"
        for kind, extension in (("collision", "stl"), ("visual", "dae"))
        for link in ("base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6")
    },
}

EXPECTED_REVOLUTE_CHAIN = {
    "joint_1": ("base_link", "link_1"),
    "joint_2": ("link_1", "link_2"),
    "joint_3": ("link_2", "link_3"),
    "joint_4": ("link_3", "link_4"),
    "joint_5": ("link_4", "link_5"),
    "joint_6": ("link_5", "link_6"),
}

# Source values from ABB datasheet ROB0275EN_A as represented by the pinned
# upstream macro. These are asset inputs, not yet the project control policy.
EXPECTED_POSITION_VELOCITY_LIMITS = {
    "joint_1": (-2.967, 2.967, 5.027),
    "joint_2": (-1.745, 2.269, 4.189),
    "joint_3": (-3.491, 1.222, 5.236),
    "joint_4": (-4.712, 4.712, 6.981),
    "joint_5": (-2.269, 2.269, 7.069),
    "joint_6": (-6.283, 6.283, 10.472),
}


def without_prefix(value: str) -> str:
    return value.replace(PREFIX_TOKEN, "")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class ABBIRB1200AssetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        if not MACRO_PATH.is_file():
            raise AssertionError(f"pinned ABB macro is missing: {MACRO_PATH}")
        cls.tree = ET.parse(MACRO_PATH)
        cls.root = cls.tree.getroot()
        macro_tag = f"{{{XACRO_NAMESPACE}}}macro"
        cls.macro = cls.root.find(macro_tag)
        if cls.macro is None:
            raise AssertionError("ABB robot xacro macro is missing")

    def test_full_sha_source_lock(self) -> None:
        repos_text = REPOS_PATH.read_text(encoding="utf-8")
        self.assertIn("https://github.com/PickNikRobotics/abb_ros2.git", repos_text)
        versions = re.findall(r"^\s+version:\s+([0-9a-f]+)\s*$", repos_text, re.MULTILINE)
        self.assertEqual(versions, [PINNED_COMMIT])
        self.assertEqual(len(PINNED_COMMIT), 40)

    def test_snapshot_scope_and_provenance(self) -> None:
        actual_source_files = {
            path.relative_to(SOURCE_ROOT).as_posix()
            for path in SOURCE_ROOT.rglob("*")
            if path.is_file()
        }
        self.assertSetEqual(actual_source_files, EXPECTED_SOURCE_FILES)

        provenance = json.loads((SNAPSHOT_ROOT / "PROVENANCE.json").read_text(encoding="utf-8"))
        self.assertEqual(provenance["repository"], "https://github.com/PickNikRobotics/abb_ros2.git")
        self.assertEqual(provenance["commit"], PINNED_COMMIT)
        self.assertEqual(provenance["license"], "Apache-2.0")
        self.assertEqual(provenance["driver_compatibility"], "NOT_CLAIMED")
        self.assertEqual(provenance["selected_file_count"], len(EXPECTED_SOURCE_FILES))

        license_text = (SNAPSHOT_ROOT / "LICENSE").read_text(encoding="utf-8")
        self.assertIn("Apache License", license_text)
        self.assertIn("Version 2.0, January 2004", license_text)

    def test_snapshot_hash_lock(self) -> None:
        locked_hashes: dict[str, str] = {}
        for line in HASH_LOCK_PATH.read_text(encoding="utf-8").splitlines():
            if not line or line.startswith("#"):
                continue
            digest, relative_path = line.split(maxsplit=1)
            self.assertRegex(digest, r"^[0-9a-f]{64}$")
            self.assertNotIn(relative_path, locked_hashes)
            locked_hashes[relative_path] = digest

        expected_locked_paths = {"LICENSE", *{f"source/{path}" for path in EXPECTED_SOURCE_FILES}}
        self.assertSetEqual(set(locked_hashes), expected_locked_paths)
        for relative_path, expected_digest in locked_hashes.items():
            self.assertEqual(sha256(SNAPSHOT_ROOT / relative_path), expected_digest, relative_path)

    def test_macro_identity_and_expected_frames(self) -> None:
        self.assertEqual(self.root.tag, "robot")
        self.assertEqual(self.macro.attrib, {"name": "abb_irb1200_5_90", "params": "prefix"})

        links = {without_prefix(link.attrib["name"]) for link in self.macro.findall("link")}
        self.assertSetEqual(
            links,
            {"base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6", "base", "flange", "tool0"},
        )

        fixed_joint_names = {
            without_prefix(joint.attrib["name"])
            for joint in self.macro.findall("joint")
            if joint.attrib["type"] == "fixed"
        }
        self.assertSetEqual(fixed_joint_names, {"base_link-base", "joint_6-flange", "link_6-tool0"})

        fixed_joints = {
            without_prefix(joint.attrib["name"]): joint
            for joint in self.macro.findall("joint")
            if joint.attrib["type"] == "fixed"
        }
        expected_fixed_frames = {
            "base_link-base": ("base_link", "base", "0 0 0", "0 0 0"),
            "joint_6-flange": ("link_6", "flange", "0 0 0", "0 0 0"),
            "link_6-tool0": ("flange", "tool0", "0 0 0", "0 ${radians(90)} 0"),
        }
        for joint_name, expected in expected_fixed_frames.items():
            joint = fixed_joints[joint_name]
            origin = joint.find("origin")
            observed = (
                without_prefix(joint.find("parent").attrib["link"]),
                without_prefix(joint.find("child").attrib["link"]),
                origin.attrib["xyz"],
                origin.attrib["rpy"],
            )
            self.assertTupleEqual(observed, expected, joint_name)

    def test_exact_six_revolute_joint_chain(self) -> None:
        revolute_joints = [
            joint for joint in self.macro.findall("joint") if joint.attrib["type"] == "revolute"
        ]
        self.assertEqual(len(revolute_joints), 6)
        observed_chain = {
            without_prefix(joint.attrib["name"]): (
                without_prefix(joint.find("parent").attrib["link"]),
                without_prefix(joint.find("child").attrib["link"]),
            )
            for joint in revolute_joints
        }
        self.assertDictEqual(observed_chain, EXPECTED_REVOLUTE_CHAIN)

    def test_source_position_and_velocity_limits(self) -> None:
        revolute_joints = {
            without_prefix(joint.attrib["name"]): joint
            for joint in self.macro.findall("joint")
            if joint.attrib["type"] == "revolute"
        }
        for joint_name, expected_values in EXPECTED_POSITION_VELOCITY_LIMITS.items():
            limit = revolute_joints[joint_name].find("limit")
            self.assertIsNotNone(limit)
            actual_values = tuple(float(limit.attrib[key]) for key in ("lower", "upper", "velocity"))
            for actual, expected in zip(actual_values, expected_values, strict=True):
                self.assertTrue(math.isfinite(actual), joint_name)
                self.assertAlmostEqual(actual, expected, places=9, msg=joint_name)
            self.assertLess(actual_values[0], actual_values[1], joint_name)
            self.assertGreater(actual_values[2], 0.0, joint_name)

        package_metadata = (SUPPORT_ROOT / "package.xml").read_text(encoding="utf-8")
        self.assertIn("effort values", package_metadata)
        self.assertIn("fictional values", package_metadata)

    def test_finite_positive_definite_link_inertias(self) -> None:
        inertial_links = [
            link for link in self.macro.findall("link") if link.find("inertial") is not None
        ]
        self.assertEqual(len(inertial_links), 7)
        for link in inertial_links:
            link_name = without_prefix(link.attrib["name"])
            inertial = link.find("inertial")
            mass = float(inertial.find("mass").attrib["value"])
            inertia = inertial.find("inertia").attrib
            ixx, ixy, ixz, iyy, iyz, izz = (
                float(inertia[key]) for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
            )
            values = (mass, ixx, ixy, ixz, iyy, iyz, izz)
            self.assertTrue(all(math.isfinite(value) for value in values), link_name)
            self.assertGreater(mass, 0.0, link_name)
            self.assertGreater(ixx, 0.0, link_name)
            self.assertGreater(ixx * iyy - ixy * ixy, 0.0, link_name)
            determinant = (
                ixx * (iyy * izz - iyz * iyz)
                - ixy * (ixy * izz - iyz * ixz)
                + ixz * (ixy * iyz - iyy * ixz)
            )
            self.assertGreater(determinant, 0.0, link_name)

    def test_visual_and_collision_meshes_resolve(self) -> None:
        mesh_references = []
        for link in self.macro.findall("link"):
            for geometry_kind in ("collision", "visual"):
                geometry = link.find(f"{geometry_kind}/geometry/mesh")
                if geometry is not None:
                    mesh_references.append((geometry_kind, geometry.attrib["filename"]))

        self.assertEqual(len(mesh_references), 14)
        for geometry_kind, package_uri in mesh_references:
            prefix = "package://abb_irb1200_support/"
            self.assertTrue(package_uri.startswith(prefix), package_uri)
            mesh_path = SUPPORT_ROOT / package_uri.removeprefix(prefix)
            self.assertTrue(mesh_path.is_file(), mesh_path)
            self.assertGreater(mesh_path.stat().st_size, 80, mesh_path)
            expected_suffix = ".stl" if geometry_kind == "collision" else ".dae"
            self.assertEqual(mesh_path.suffix, expected_suffix)


if __name__ == "__main__":
    unittest.main()
