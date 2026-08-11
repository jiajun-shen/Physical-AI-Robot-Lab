#!/usr/bin/env python3
"""Static and deterministic contracts for the project-owned manipulation lab."""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / "config" / "manipulation_scene.yaml"
WORLD_PATH = PACKAGE_ROOT / "worlds" / "manipulation_lab.sdf"
GENERATOR_PATH = PACKAGE_ROOT / "scripts" / "generate_scene_manifest.py"

STATIC_MODELS = {
    "ground_plane",
    "work_table",
    "drawing_board",
    "obstacle_left",
    "obstacle_right",
}
DYNAMIC_MODELS = {
    "green_ball",
    "red_ball",
    "blue_cube",
    "yellow_cube",
}
SEEDABLE_STATIC_MODELS = {"obstacle_left", "obstacle_right"}
EXPECTED_COLORS = {
    "green_ball": (0.05, 0.75, 0.15, 1.0),
    "red_ball": (0.80, 0.05, 0.05, 1.0),
    "blue_cube": (0.05, 0.25, 0.85, 1.0),
    "yellow_cube": (0.95, 0.80, 0.05, 1.0),
}


def required_text(path: Path) -> str:
    if not path.is_file():
        raise AssertionError(f"required file is missing: {path}")
    return path.read_text(encoding="utf-8")


def required_xml(path: Path) -> ET.Element:
    if not path.is_file():
        raise AssertionError(f"required XML is missing: {path}")
    return ET.parse(path).getroot()


def model_by_name(world: ET.Element, name: str) -> ET.Element:
    for model in world.findall("model"):
        if model.attrib.get("name") == name:
            return model
    raise AssertionError(f"missing model {name}")


def pose_values(model: ET.Element) -> tuple[float, float, float, float, float, float]:
    pose = model.findtext("pose")
    if pose is None:
        raise AssertionError(f"model {model.attrib.get('name')} has no pose")
    values = tuple(float(value) for value in pose.split())
    if len(values) != 6:
        raise AssertionError(f"invalid pose for {model.attrib.get('name')}: {pose}")
    return values


class ABBLabLabSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        spec = importlib.util.spec_from_file_location(
            "generate_scene_manifest_for_test", GENERATOR_PATH
        )
        assert spec is not None and spec.loader is not None
        cls.generator = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = cls.generator
        spec.loader.exec_module(cls.generator)

    def test_package_installs_only_project_owned_scene_resources(self) -> None:
        for path in (
            PACKAGE_ROOT / "CMakeLists.txt",
            PACKAGE_ROOT / "package.xml",
            CONFIG_PATH,
            WORLD_PATH,
            GENERATOR_PATH,
        ):
            with self.subTest(path=path.name):
                self.assertTrue(path.is_file(), path)

        manifest = required_xml(PACKAGE_ROOT / "package.xml")
        self.assertEqual(manifest.findtext("name"), "abb_lab_lab")
        dependencies = {element.text for element in manifest.findall("exec_depend")}
        self.assertEqual(dependencies, {"python3-yaml"})
        self.assertTrue(
            {
                "abb_hardware_interface",
                "controller_manager",
                "gazebo_ros",
                "moveit_ros_move_group",
                "ros_gz_bridge",
                "ros_gz_sim",
            }.isdisjoint(dependencies)
        )

    def test_world_has_declared_physical_static_and_dynamic_entities(self) -> None:
        root = required_xml(WORLD_PATH)
        self.assertEqual(root.tag, "sdf")
        self.assertEqual(root.attrib.get("version"), "1.10")
        worlds = root.findall("world")
        self.assertEqual(len(worlds), 1)
        world = worlds[0]
        self.assertEqual(world.attrib.get("name"), "manipulation_lab")
        self.assertEqual(world.findtext("gravity"), "0 0 -9.81")
        self.assertEqual(world.findall("gui"), [])
        self.assertEqual(world.findall("sensor"), [])

        models = {model.attrib["name"]: model for model in world.findall("model")}
        self.assertEqual(set(models), STATIC_MODELS | DYNAMIC_MODELS)
        for name in STATIC_MODELS:
            with self.subTest(model=name):
                self.assertEqual(models[name].findtext("static"), "true")
        for name in DYNAMIC_MODELS:
            with self.subTest(model=name):
                self.assertEqual(models[name].findtext("static"), "false")

        for name, model in models.items():
            with self.subTest(physical_model=name):
                link = model.find("link")
                self.assertIsNotNone(link)
                inertial = link.find("inertial")
                self.assertIsNotNone(inertial)
                self.assertGreater(float(inertial.findtext("mass")), 0.0)
                self.assertIsNotNone(inertial.find("inertia"))
                collision = link.find("collision")
                self.assertIsNotNone(collision)
                self.assertIsNotNone(collision.find("geometry"))
                self.assertGreater(
                    float(collision.findtext("surface/friction/ode/mu")), 0.0
                )
                self.assertGreater(
                    float(collision.findtext("surface/friction/ode/mu2")), 0.0
                )
                visual = link.find("visual")
                self.assertIsNotNone(visual)
                self.assertIsNotNone(visual.find("geometry"))

        for name, expected_color in EXPECTED_COLORS.items():
            diffuse = models[name].findtext("link/visual/material/diffuse")
            self.assertIsNotNone(diffuse)
            observed = tuple(float(value) for value in diffuse.split())
            self.assertEqual(observed, expected_color)

        world_source = required_text(WORLD_PATH)
        for forbidden in ("DetachableJoint", "detachable_joint", "<joint"):
            self.assertNotIn(forbidden, world_source)

    def test_default_seeded_poses_match_the_project_owned_world(self) -> None:
        config = self.generator.load_scene_config(CONFIG_PATH)
        manifest = self.generator.build_scene_manifest(config, config["default_seed"])
        self.generator.assert_manifest_valid(manifest)

        world = required_xml(WORLD_PATH).find("world")
        for entry in manifest["obstacles"] + manifest["objects"]:
            with self.subTest(object=entry["name"]):
                pose = pose_values(model_by_name(world, entry["name"]))
                self.assertEqual(
                    pose,
                    tuple(entry["pose"]["position_m"] + entry["pose"]["rpy_rad"]),
                )

    def test_same_seed_is_identical_and_different_seed_changes_placements(self) -> None:
        config = self.generator.load_scene_config(CONFIG_PATH)
        first = self.generator.build_scene_manifest(config, 42)
        second = self.generator.build_scene_manifest(config, 42)
        alternate = self.generator.build_scene_manifest(config, 43)
        self.assertEqual(first, second)
        self.assertNotEqual(
            [entry["pose"] for entry in first["obstacles"] + first["objects"]],
            [
                entry["pose"]
                for entry in alternate["obstacles"] + alternate["objects"]
            ],
        )
        self.assertNotEqual(
            [entry["pose"] for entry in first["obstacles"]],
            [entry["pose"] for entry in alternate["obstacles"]],
        )
        for name in SEEDABLE_STATIC_MODELS | DYNAMIC_MODELS:
            with self.subTest(model=name):
                first_entry = next(
                    entry
                    for entry in first["obstacles"] + first["objects"]
                    if entry["name"] == name
                )
                alternate_entry = next(
                    entry
                    for entry in alternate["obstacles"] + alternate["objects"]
                    if entry["name"] == name
                )
                self.assertNotEqual(
                    first_entry["pose"]["rpy_rad"][2],
                    alternate_entry["pose"]["rpy_rad"][2],
                )
        self.generator.assert_manifest_valid(first)
        self.generator.assert_manifest_valid(alternate)

    def test_boundary_seeds_remain_valid(self) -> None:
        config = self.generator.load_scene_config(CONFIG_PATH)
        seeds = (0, 1, 42, 43, 2**63 - 1, 2**128 - 1)
        manifests = []
        for seed in seeds:
            with self.subTest(seed=seed):
                manifest = self.generator.build_scene_manifest(config, seed)
                self.assertEqual(manifest["seed"], seed)
                self.generator.assert_manifest_valid(manifest)
                manifests.append(manifest)
        obstacle_positions = {
            tuple(
                tuple(entry["pose"]["position_m"])
                for entry in manifest["obstacles"]
            )
            for manifest in manifests
        }
        obstacle_yaws = {
            tuple(entry["pose"]["rpy_rad"][2] for entry in manifest["obstacles"])
            for manifest in manifests
        }
        self.assertGreater(len(obstacle_positions), 1)
        self.assertGreater(len(obstacle_yaws), 1)
        for name in SEEDABLE_STATIC_MODELS | DYNAMIC_MODELS:
            with self.subTest(model=name):
                entries = [
                    next(
                        entry
                        for entry in manifest["obstacles"] + manifest["objects"]
                        if entry["name"] == name
                    )
                    for manifest in manifests
                ]
                self.assertGreater(
                    len({entry["pose"]["rpy_rad"][2] for entry in entries}), 1
                )
                expected_symmetric = name in {"green_ball", "red_ball"}
                self.assertTrue(
                    all(
                        entry["orientation_is_geometrically_symmetric"]
                        == expected_symmetric
                        for entry in entries
                    )
                )

    def test_seeded_yaw_uses_primitive_specific_level_contract(self) -> None:
        config = self.generator.load_scene_config(CONFIG_PATH)
        manifest = self.generator.build_scene_manifest(config, 20260811)
        placement = config["placement"]
        yaw_candidates_by_primitive = {
            "sphere": placement["sphere_yaw_candidates_rad"],
            "box": placement["cube_yaw_candidates_rad"],
        }
        for entry in manifest["objects"]:
            with self.subTest(model=entry["name"]):
                roll, pitch, yaw = entry["pose"]["rpy_rad"]
                self.assertEqual((roll, pitch), (0.0, 0.0))
                self.assertIn(yaw, yaw_candidates_by_primitive[entry["primitive"]])
                self.assertEqual(
                    entry["orientation_is_geometrically_symmetric"],
                    entry["primitive"] == "sphere",
                )

    def test_generated_manifest_is_inside_reachable_bounds_and_nonoverlapping(self) -> None:
        config = self.generator.load_scene_config(CONFIG_PATH)
        manifest = self.generator.build_scene_manifest(config, 20260811)
        self.generator.assert_manifest_valid(manifest)

        bounds = manifest["table"]["reachable_bounds_m"]
        entries = manifest["obstacles"] + manifest["objects"]
        for entry in entries:
            with self.subTest(object=entry["name"]):
                x_value, y_value, z_value = entry["pose"]["position_m"]
                radius = entry["footprint_radius_m"]
                self.assertGreaterEqual(x_value - radius, bounds["x_m"][0])
                self.assertLessEqual(x_value + radius, bounds["x_m"][1])
                self.assertGreaterEqual(y_value - radius, bounds["y_m"][0])
                self.assertLessEqual(y_value + radius, bounds["y_m"][1])
                self.assertGreater(z_value, manifest["table"]["surface_z_m"])

        minimum_gap = manifest["placement"]["minimum_pairwise_gap_m"]
        for index, first in enumerate(entries):
            for second in entries[index + 1 :]:
                dx = first["pose"]["position_m"][0] - second["pose"]["position_m"][0]
                dy = first["pose"]["position_m"][1] - second["pose"]["position_m"][1]
                separation = math.hypot(dx, dy)
                self.assertGreaterEqual(
                    separation,
                    first["footprint_radius_m"]
                    + second["footprint_radius_m"]
                    + minimum_gap,
                )

    def test_cli_writes_repeatable_external_manifest_and_world_without_mutating_source(self) -> None:
        source_world = WORLD_PATH.read_bytes()
        with tempfile.TemporaryDirectory(prefix="abb-lab-scene-") as directory:
            root = Path(directory)
            first_manifest = root / "first.json"
            first_world = root / "first.sdf"
            second_manifest = root / "second.json"
            second_world = root / "second.sdf"
            altered_manifest = root / "altered.json"
            altered_world = root / "altered.sdf"

            def invoke(seed: int, manifest_path: Path, world_path: Path) -> None:
                subprocess.run(
                    [
                        sys.executable,
                        str(GENERATOR_PATH),
                        "--config",
                        str(CONFIG_PATH),
                        "--seed",
                        str(seed),
                        "--manifest-output",
                        str(manifest_path),
                        "--world-template",
                        str(WORLD_PATH),
                        "--world-output",
                        str(world_path),
                    ],
                    check=True,
                    capture_output=True,
                    text=True,
                )

            invoke(91, first_manifest, first_world)
            invoke(91, second_manifest, second_world)
            invoke(92, altered_manifest, altered_world)

            self.assertEqual(first_manifest.read_bytes(), second_manifest.read_bytes())
            self.assertEqual(first_world.read_bytes(), second_world.read_bytes())
            self.assertNotEqual(first_manifest.read_bytes(), altered_manifest.read_bytes())
            self.assertEqual(WORLD_PATH.read_bytes(), source_world)

            manifest = json.loads(first_manifest.read_text(encoding="utf-8"))
            seeded_world = required_xml(first_world).find("world")
            self.assertEqual(seeded_world.attrib["name"], "manipulation_lab")
            for entry in manifest["obstacles"] + manifest["objects"]:
                with self.subTest(object=entry["name"]):
                    self.assertEqual(
                        pose_values(model_by_name(seeded_world, entry["name"])),
                        tuple(entry["pose"]["position_m"] + entry["pose"]["rpy_rad"]),
                    )

    def test_generator_is_hash_seeded_and_does_not_hide_runtime_capabilities(self) -> None:
        source = required_text(GENERATOR_PATH)
        self.assertIn("hashlib.sha256", source)
        self.assertNotIn("random", source.lower())
        for forbidden in ("DetachableJoint", "grasp", "perception", "ground_truth"):
            self.assertNotIn(forbidden, source)

        config = yaml.safe_load(required_text(CONFIG_PATH))
        self.assertEqual(config["scene_schema_version"], 1)
        self.assertEqual(set(config["objects"]), DYNAMIC_MODELS)
        self.assertEqual(set(config["obstacles"]), SEEDABLE_STATIC_MODELS)
        placement = config["placement"]
        self.assertEqual(len(placement["obstacle_yaw_candidates_rad"]), 3)
        self.assertEqual(len(placement["cube_yaw_candidates_rad"]), 4)
        self.assertEqual(len(placement["sphere_yaw_candidates_rad"]), 4)


if __name__ == "__main__":
    unittest.main()
