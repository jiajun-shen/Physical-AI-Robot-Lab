#!/usr/bin/env python3
"""Create a repeatable manipulation-lab layout and optional external SDF copy."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable
import xml.etree.ElementTree as ET

import yaml


SCHEMA_VERSION = 1
OBSTACLE_ORDER = ("obstacle_left", "obstacle_right")
OBJECT_ORDER = ("green_ball", "red_ball", "blue_cube", "yellow_cube")
EXPECTED_PRIMITIVES = {
    "green_ball": "sphere",
    "red_ball": "sphere",
    "blue_cube": "box",
    "yellow_cube": "box",
}


class SceneConfigurationError(ValueError):
    """Raised when the scene configuration cannot define a safe layout."""


def _finite_number(value: Any, label: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SceneConfigurationError(f"{label} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise SceneConfigurationError(f"{label} must be finite")
    if minimum is not None and result < minimum:
        raise SceneConfigurationError(f"{label} must be at least {minimum}")
    return result


def _number_list(value: Any, label: str, length: int) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise SceneConfigurationError(f"{label} must contain exactly {length} values")
    return [_finite_number(item, f"{label}[{index}]") for index, item in enumerate(value)]


def _bounds(value: Any, label: str) -> list[float]:
    bounds = _number_list(value, label, 2)
    if not bounds[0] < bounds[1]:
        raise SceneConfigurationError(f"{label} must be increasing")
    return bounds


def _yaw_candidates(value: Any, label: str, length: int) -> list[float]:
    candidates = _number_list(value, label, length)
    if len(set(candidates)) != len(candidates):
        raise SceneConfigurationError(f"{label} must contain unique values")
    if any(abs(candidate) > math.pi for candidate in candidates):
        raise SceneConfigurationError(f"{label} values must be within [-pi, pi]")
    return candidates


def _validate_config(config: Any) -> None:
    if not isinstance(config, dict):
        raise SceneConfigurationError("scene configuration must be a mapping")
    if config.get("scene_schema_version") != SCHEMA_VERSION:
        raise SceneConfigurationError(
            f"scene_schema_version must be {SCHEMA_VERSION}"
        )
    seed = config.get("default_seed")
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise SceneConfigurationError("default_seed must be a non-negative integer")
    if config.get("frame_id") != "world":
        raise SceneConfigurationError("frame_id must be world")

    table = config.get("table")
    if not isinstance(table, dict) or table.get("model_name") != "work_table":
        raise SceneConfigurationError("table.model_name must name work_table")
    _finite_number(table.get("surface_z_m"), "table.surface_z_m", minimum=0.0)
    reachable = table.get("reachable_bounds_m")
    if not isinstance(reachable, dict):
        raise SceneConfigurationError("table.reachable_bounds_m must be a mapping")
    _bounds(reachable.get("x_m"), "table.reachable_bounds_m.x_m")
    _bounds(reachable.get("y_m"), "table.reachable_bounds_m.y_m")

    placement = config.get("placement")
    if not isinstance(placement, dict):
        raise SceneConfigurationError("placement must be a mapping")
    _finite_number(
        placement.get("minimum_pairwise_gap_m"),
        "placement.minimum_pairwise_gap_m",
        minimum=0.0,
    )
    grid = placement.get("candidate_grid_m")
    if not isinstance(grid, dict):
        raise SceneConfigurationError("placement.candidate_grid_m must be a mapping")
    x_values = _number_list(grid.get("x_m"), "placement.candidate_grid_m.x_m", 3)
    y_values = _number_list(grid.get("y_m"), "placement.candidate_grid_m.y_m", 3)
    if len(set(x_values)) != len(x_values) or len(set(y_values)) != len(y_values):
        raise SceneConfigurationError("placement candidates must be unique")
    _yaw_candidates(
        placement.get("obstacle_yaw_candidates_rad"),
        "placement.obstacle_yaw_candidates_rad",
        3,
    )
    _yaw_candidates(
        placement.get("cube_yaw_candidates_rad"),
        "placement.cube_yaw_candidates_rad",
        4,
    )
    _yaw_candidates(
        placement.get("sphere_yaw_candidates_rad"),
        "placement.sphere_yaw_candidates_rad",
        4,
    )

    obstacles = config.get("obstacles")
    if not isinstance(obstacles, dict) or set(obstacles) != set(OBSTACLE_ORDER):
        raise SceneConfigurationError(
            f"obstacles must be exactly {', '.join(OBSTACLE_ORDER)}"
        )

    objects = config.get("objects")
    if not isinstance(objects, dict) or set(objects) != set(OBJECT_ORDER):
        raise SceneConfigurationError(
            f"objects must be exactly {', '.join(OBJECT_ORDER)}"
        )
    surface_z = float(table["surface_z_m"])
    for name in OBSTACLE_ORDER:
        obstacle_config = obstacles[name]
        if not isinstance(obstacle_config, dict):
            raise SceneConfigurationError(f"obstacles.{name} must be a mapping")
        if obstacle_config.get("primitive") != "box":
            raise SceneConfigurationError(f"obstacles.{name}.primitive must be box")
        _finite_number(
            obstacle_config.get("mass_kg"), f"obstacles.{name}.mass_kg", minimum=0.0
        )
        size = _number_list(obstacle_config.get("size_m"), f"obstacles.{name}.size_m", 3)
        if any(value <= 0.0 for value in size):
            raise SceneConfigurationError(f"obstacles.{name}.size_m must be positive")
        radius = _finite_number(
            obstacle_config.get("footprint_radius_m"),
            f"obstacles.{name}.footprint_radius_m",
            minimum=0.0,
        )
        expected_radius = math.hypot(size[0], size[1]) / 2.0
        if not math.isclose(radius, expected_radius, rel_tol=0.0, abs_tol=1e-12):
            raise SceneConfigurationError(
                f"obstacles.{name}.footprint_radius_m must bound the box footprint"
            )
        z_center = _finite_number(
            obstacle_config.get("z_center_m"), f"obstacles.{name}.z_center_m"
        )
        if not math.isclose(
            z_center, surface_z + size[2] / 2.0, rel_tol=0.0, abs_tol=1e-12
        ):
            raise SceneConfigurationError(
                f"obstacles.{name}.z_center_m must rest on the table surface"
            )

    for name in OBJECT_ORDER:
        object_config = objects[name]
        if not isinstance(object_config, dict):
            raise SceneConfigurationError(f"objects.{name} must be a mapping")
        if object_config.get("primitive") != EXPECTED_PRIMITIVES[name]:
            raise SceneConfigurationError(f"objects.{name}.primitive is not supported")
        if not isinstance(object_config.get("color"), str):
            raise SceneConfigurationError(f"objects.{name}.color must be text")
        _finite_number(object_config.get("mass_kg"), f"objects.{name}.mass_kg", minimum=0.0)
        radius = _finite_number(
            object_config.get("footprint_radius_m"),
            f"objects.{name}.footprint_radius_m",
            minimum=0.0,
        )
        z_center = _finite_number(
            object_config.get("z_center_m"), f"objects.{name}.z_center_m"
        )
        if object_config["primitive"] == "sphere":
            physical_radius = _finite_number(
                object_config.get("radius_m"), f"objects.{name}.radius_m", minimum=0.0
            )
            if not math.isclose(radius, physical_radius, rel_tol=0.0, abs_tol=1e-12):
                raise SceneConfigurationError(
                    f"objects.{name}.footprint_radius_m must equal radius_m"
                )
            expected_z = surface_z + physical_radius
        else:
            size = _number_list(object_config.get("size_m"), f"objects.{name}.size_m", 3)
            if any(value <= 0.0 for value in size):
                raise SceneConfigurationError(f"objects.{name}.size_m must be positive")
            expected_radius = math.hypot(size[0], size[1]) / 2.0
            if not math.isclose(radius, expected_radius, rel_tol=0.0, abs_tol=1e-12):
                raise SceneConfigurationError(
                    f"objects.{name}.footprint_radius_m must bound the box footprint"
                )
            expected_z = surface_z + size[2] / 2.0
        if not math.isclose(z_center, expected_z, rel_tol=0.0, abs_tol=1e-12):
            raise SceneConfigurationError(
                f"objects.{name}.z_center_m must rest on the table surface"
            )


def load_scene_config(path: str | Path) -> dict[str, Any]:
    """Load and validate the only supported scene schema."""

    config_path = Path(path)
    if not config_path.is_file():
        raise SceneConfigurationError(f"scene config is missing: {config_path}")
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    _validate_config(config)
    return config


def _candidate_order(seed: int, object_name: str, count: int) -> Iterable[int]:
    """Return a stable seed-specific order without interpreter RNG state."""

    return sorted(
        range(count),
        key=lambda index: hashlib.sha256(
            f"abb_lab_lab/v{SCHEMA_VERSION}/{seed}/{object_name}/{index}".encode(
                "ascii"
            )
        ).hexdigest(),
    )


def _seeded_yaw(seed: int, name: str, yaw_candidates: list[float]) -> float:
    digest = hashlib.sha256(
        f"abb_lab_lab/v{SCHEMA_VERSION}/{seed}/{name}/yaw".encode("ascii")
    ).digest()
    return yaw_candidates[int.from_bytes(digest[:8], byteorder="big") % len(yaw_candidates)]


def _candidate_is_usable(
    candidate: tuple[float, float],
    footprint_radius: float,
    placed: list[dict[str, Any]],
    bounds_x: list[float],
    bounds_y: list[float],
    minimum_gap: float,
) -> bool:
    x_value, y_value = candidate
    if (
        x_value - footprint_radius < bounds_x[0]
        or x_value + footprint_radius > bounds_x[1]
        or y_value - footprint_radius < bounds_y[0]
        or y_value + footprint_radius > bounds_y[1]
    ):
        return False
    for existing in placed:
        existing_x, existing_y, _ = existing["pose"]["position_m"]
        separation = math.hypot(x_value - existing_x, y_value - existing_y)
        required = footprint_radius + existing["footprint_radius_m"] + minimum_gap
        if separation + 1e-12 < required:
            return False
    return True


def build_scene_manifest(config: dict[str, Any], seed: int) -> dict[str, Any]:
    """Build a validated, JSON-serializable scene layout for a non-negative seed."""

    _validate_config(config)
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise SceneConfigurationError("seed must be a non-negative integer")

    table = config["table"]
    bounds_x = [float(value) for value in table["reachable_bounds_m"]["x_m"]]
    bounds_y = [float(value) for value in table["reachable_bounds_m"]["y_m"]]
    placement = config["placement"]
    minimum_gap = float(placement["minimum_pairwise_gap_m"])
    candidates = [
        (float(x_value), float(y_value))
        for x_value in placement["candidate_grid_m"]["x_m"]
        for y_value in placement["candidate_grid_m"]["y_m"]
    ]

    placed: list[dict[str, Any]] = []

    obstacle_yaw_candidates = [
        float(value) for value in placement["obstacle_yaw_candidates_rad"]
    ]
    cube_yaw_candidates = [
        float(value) for value in placement["cube_yaw_candidates_rad"]
    ]
    sphere_yaw_candidates = [
        float(value) for value in placement["sphere_yaw_candidates_rad"]
    ]

    def place_entries(
        names: tuple[str, ...], section: str, *, static: bool
    ) -> list[dict[str, Any]]:
        entries: list[dict[str, Any]] = []
        for name in names:
            entry_config = config[section][name]
            footprint_radius = float(entry_config["footprint_radius_m"])
            selected: tuple[float, float] | None = None
            for index in _candidate_order(seed, name, len(candidates)):
                candidate = candidates[index]
                if _candidate_is_usable(
                    candidate,
                    footprint_radius,
                    placed,
                    bounds_x,
                    bounds_y,
                    minimum_gap,
                ):
                    selected = candidate
                    break
            if selected is None:
                raise SceneConfigurationError(
                    f"no safe candidate remained for {name} with seed {seed}"
                )
            if static:
                yaw_candidates = obstacle_yaw_candidates
            elif entry_config["primitive"] == "sphere":
                yaw_candidates = sphere_yaw_candidates
            else:
                yaw_candidates = cube_yaw_candidates
            yaw_value = _seeded_yaw(seed, name, yaw_candidates)
            if entry_config["primitive"] == "sphere":
                support_offset = float(entry_config["radius_m"])
                geometry = {"radius_m": support_offset}
            else:
                size = [float(value) for value in entry_config["size_m"]]
                support_offset = size[2] / 2.0
                geometry = {"size_m": size}
            entry = {
                "name": name,
                "primitive": entry_config["primitive"],
                "mass_kg": float(entry_config["mass_kg"]),
                "footprint_radius_m": footprint_radius,
                "support_offset_m": support_offset,
                "geometry": geometry,
                "static": static,
                # Sphere yaw is retained so all seeded models expose the same
                # pose representation; it has no observable directional
                # geometry for a spherical primitive.
                "orientation_is_geometrically_symmetric": entry_config["primitive"]
                == "sphere",
                "pose": {
                    "position_m": [selected[0], selected[1], float(entry_config["z_center_m"])],
                    "rpy_rad": [0.0, 0.0, yaw_value],
                },
            }
            if "color" in entry_config:
                entry["color"] = entry_config["color"]
            placed.append(entry)
            entries.append(entry)
        return entries

    obstacles = place_entries(OBSTACLE_ORDER, "obstacles", static=True)
    objects = place_entries(OBJECT_ORDER, "objects", static=False)

    manifest = {
        "scene_schema_version": SCHEMA_VERSION,
        "seed": seed,
        "frame_id": config["frame_id"],
        "table": {
            "model_name": table["model_name"],
            "surface_z_m": float(table["surface_z_m"]),
            "reachable_bounds_m": {"x_m": bounds_x, "y_m": bounds_y},
        },
        "placement": {"minimum_pairwise_gap_m": minimum_gap},
        "obstacles": obstacles,
        "objects": objects,
    }
    assert_manifest_valid(manifest)
    return manifest


def assert_manifest_valid(manifest: dict[str, Any]) -> None:
    """Verify placement bounds, tabletop contact height, and pairwise clearance."""

    if manifest.get("scene_schema_version") != SCHEMA_VERSION:
        raise SceneConfigurationError("manifest schema version is unsupported")
    if manifest.get("frame_id") != "world":
        raise SceneConfigurationError("manifest frame_id must be world")
    table = manifest.get("table")
    placement = manifest.get("placement")
    obstacles = manifest.get("obstacles")
    objects = manifest.get("objects")
    if (
        not isinstance(table, dict)
        or not isinstance(placement, dict)
        or not isinstance(obstacles, list)
        or not isinstance(objects, list)
    ):
        raise SceneConfigurationError("manifest is incomplete")
    if [entry.get("name") for entry in obstacles] != list(OBSTACLE_ORDER):
        raise SceneConfigurationError("manifest obstacles are incomplete or reordered")
    if [entry.get("name") for entry in objects] != list(OBJECT_ORDER):
        raise SceneConfigurationError("manifest objects are incomplete or reordered")
    bounds = table.get("reachable_bounds_m")
    if not isinstance(bounds, dict):
        raise SceneConfigurationError("manifest lacks reachable bounds")
    bounds_x = _bounds(bounds.get("x_m"), "manifest.table.reachable_bounds_m.x_m")
    bounds_y = _bounds(bounds.get("y_m"), "manifest.table.reachable_bounds_m.y_m")
    surface_z = _finite_number(table.get("surface_z_m"), "manifest.table.surface_z_m")
    minimum_gap = _finite_number(
        placement.get("minimum_pairwise_gap_m"),
        "manifest.placement.minimum_pairwise_gap_m",
        minimum=0.0,
    )
    entries = obstacles + objects
    for entry in entries:
        primitive = entry.get("primitive")
        radius = _finite_number(
            entry.get("footprint_radius_m"), f"manifest.{entry['name']}.footprint_radius_m", minimum=0.0
        )
        support_offset = _finite_number(
            entry.get("support_offset_m"),
            f"manifest.{entry['name']}.support_offset_m",
            minimum=0.0,
        )
        geometry = entry.get("geometry")
        if not isinstance(geometry, dict):
            raise SceneConfigurationError(f"manifest.{entry['name']}.geometry is missing")
        if primitive == "sphere":
            physical_radius = _finite_number(
                geometry.get("radius_m"), f"manifest.{entry['name']}.geometry.radius_m", minimum=0.0
            )
            if not math.isclose(radius, physical_radius, rel_tol=0.0, abs_tol=1e-12):
                raise SceneConfigurationError(
                    f"manifest.{entry['name']} sphere footprint is inconsistent"
                )
            expected_offset = physical_radius
        elif primitive == "box":
            size = _number_list(
                geometry.get("size_m"), f"manifest.{entry['name']}.geometry.size_m", 3
            )
            if any(value <= 0.0 for value in size):
                raise SceneConfigurationError(
                    f"manifest.{entry['name']}.geometry.size_m must be positive"
                )
            expected_radius = math.hypot(size[0], size[1]) / 2.0
            if not math.isclose(radius, expected_radius, rel_tol=0.0, abs_tol=1e-12):
                raise SceneConfigurationError(
                    f"manifest.{entry['name']} box footprint is inconsistent"
                )
            expected_offset = size[2] / 2.0
        else:
            raise SceneConfigurationError(f"manifest.{entry['name']}.primitive is unsupported")
        if not math.isclose(
            support_offset, expected_offset, rel_tol=0.0, abs_tol=1e-12
        ):
            raise SceneConfigurationError(
                f"manifest.{entry['name']} support offset is inconsistent"
            )
        pose = entry.get("pose")
        if not isinstance(pose, dict):
            raise SceneConfigurationError(f"manifest.{entry['name']}.pose is missing")
        x_value, y_value, z_value = _number_list(
            pose.get("position_m"), f"manifest.{entry['name']}.position_m", 3
        )
        roll, pitch, _ = _number_list(
            pose.get("rpy_rad"), f"manifest.{entry['name']}.rpy_rad", 3
        )
        if roll != 0.0 or pitch != 0.0:
            raise SceneConfigurationError(
                f"manifest.{entry['name']} must remain level on the table"
            )
        orientation_is_geometrically_symmetric = entry.get(
            "orientation_is_geometrically_symmetric"
        )
        if not isinstance(orientation_is_geometrically_symmetric, bool):
            raise SceneConfigurationError(
                f"manifest.{entry['name']} must declare orientation symmetry"
            )
        if orientation_is_geometrically_symmetric != (primitive == "sphere"):
            raise SceneConfigurationError(
                f"manifest.{entry['name']} has inconsistent orientation symmetry"
            )
        if x_value - radius < bounds_x[0] or x_value + radius > bounds_x[1]:
            raise SceneConfigurationError(f"manifest.{entry['name']} is outside x bounds")
        if y_value - radius < bounds_y[0] or y_value + radius > bounds_y[1]:
            raise SceneConfigurationError(f"manifest.{entry['name']} is outside y bounds")
        if not math.isclose(
            z_value, surface_z + support_offset, rel_tol=0.0, abs_tol=1e-12
        ):
            raise SceneConfigurationError(
                f"manifest.{entry['name']} does not rest on the table"
            )
    if [entry.get("static") for entry in obstacles] != [True] * len(OBSTACLE_ORDER):
        raise SceneConfigurationError("manifest obstacles must remain static")
    if [entry.get("static") for entry in objects] != [False] * len(OBJECT_ORDER):
        raise SceneConfigurationError("manifest objects must remain dynamic")
    for index, first in enumerate(entries):
        first_x, first_y, _ = first["pose"]["position_m"]
        for second in entries[index + 1 :]:
            second_x, second_y, _ = second["pose"]["position_m"]
            separation = math.hypot(first_x - second_x, first_y - second_y)
            required = (
                first["footprint_radius_m"]
                + second["footprint_radius_m"]
                + minimum_gap
            )
            if separation + 1e-12 < required:
                raise SceneConfigurationError(
                    f"manifest objects {first['name']} and {second['name']} overlap"
                )


def _model_by_name(world: ET.Element, name: str) -> ET.Element:
    for model in world.findall("model"):
        if model.attrib.get("name") == name:
            return model
    raise SceneConfigurationError(f"world template is missing model {name}")


def write_seeded_world(
    manifest: dict[str, Any], world_template: str | Path, world_output: str | Path
) -> None:
    """Write a copy of a project-owned template with seedable poses replaced."""

    assert_manifest_valid(manifest)
    template_path = Path(world_template).resolve()
    output_path = Path(world_output).resolve()
    if not template_path.is_file():
        raise SceneConfigurationError(f"world template is missing: {template_path}")
    if output_path == template_path:
        raise SceneConfigurationError("world output must not overwrite its template")
    tree = ET.parse(template_path)
    root = tree.getroot()
    world = root.find("world")
    if root.tag != "sdf" or world is None or world.attrib.get("name") != "manipulation_lab":
        raise SceneConfigurationError("world template is not the manipulation_lab world")
    for entry in manifest["obstacles"] + manifest["objects"]:
        model = _model_by_name(world, entry["name"])
        expected_static = "true" if entry["static"] else "false"
        if model.findtext("static") != expected_static:
            raise SceneConfigurationError(
                f"world model {entry['name']} has the wrong static setting"
            )
        pose = model.find("pose")
        if pose is None:
            raise SceneConfigurationError(f"world model {entry['name']} has no pose")
        values = entry["pose"]["position_m"] + entry["pose"]["rpy_rad"]
        pose.text = " ".join(f"{float(value):.12g}" for value in values)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


def write_manifest(manifest: dict[str, Any], manifest_output: str | Path) -> None:
    """Write formatted JSON outside the source package."""

    assert_manifest_valid(manifest)
    output_path = Path(manifest_output).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a deterministic manipulation-lab manifest and optional SDF copy."
    )
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--seed", required=True, type=int)
    parser.add_argument("--manifest-output", required=True, type=Path)
    parser.add_argument("--world-template", type=Path)
    parser.add_argument("--world-output", type=Path)
    arguments = parser.parse_args()
    if (arguments.world_template is None) != (arguments.world_output is None):
        parser.error("--world-template and --world-output must be supplied together")
    return arguments


def main() -> int:
    arguments = parse_arguments()
    config = load_scene_config(arguments.config)
    manifest = build_scene_manifest(config, arguments.seed)
    write_manifest(manifest, arguments.manifest_output)
    print(f"SCENE_MANIFEST={arguments.manifest_output.resolve()}")
    print(f"SCENE_SEED={arguments.seed}")
    if arguments.world_output is not None:
        write_seeded_world(manifest, arguments.world_template, arguments.world_output)
        print(f"SEEDED_WORLD={arguments.world_output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
