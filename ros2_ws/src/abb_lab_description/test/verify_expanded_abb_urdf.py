#!/usr/bin/env python3
"""Verify an expanded geometry-only ABB URDF and its installed resources."""

from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


EXPECTED_REVOLUTE_CHAIN = {
    "joint_1": ("base_link", "link_1"),
    "joint_2": ("link_1", "link_2"),
    "joint_3": ("link_2", "link_3"),
    "joint_4": ("link_3", "link_4"),
    "joint_5": ("link_4", "link_5"),
    "joint_6": ("link_5", "link_6"),
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf", type=Path)
    arguments = parser.parse_args()

    urdf_text = arguments.urdf.read_text(encoding="utf-8")
    root = ET.fromstring(urdf_text)
    require(root.tag == "robot", "root element is not robot")
    require(root.attrib.get("name") == "abb_irb1200_5_90_geometry_only", "unexpected robot name")

    for forbidden_token in ("<ros2_control", "<gazebo", "<transmission", "ABBSystemHardware"):
        require(forbidden_token not in urdf_text, f"forbidden control/simulator token: {forbidden_token}")

    links = {link.attrib["name"] for link in root.findall("link")}
    require(
        links == {"base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6", "base", "flange", "tool0"},
        f"unexpected links: {sorted(links)}",
    )

    joints = root.findall("joint")
    require(len(joints) == 9, f"expected 9 joints, observed {len(joints)}")
    revolute = {joint.attrib["name"]: joint for joint in joints if joint.attrib["type"] == "revolute"}
    fixed = {joint.attrib["name"] for joint in joints if joint.attrib["type"] == "fixed"}
    require(set(revolute) == set(EXPECTED_REVOLUTE_CHAIN), f"unexpected revolute joints: {sorted(revolute)}")
    require(fixed == {"base_link-base", "joint_6-flange", "link_6-tool0"}, f"unexpected fixed joints: {sorted(fixed)}")

    for joint_name, expected_chain in EXPECTED_REVOLUTE_CHAIN.items():
        joint = revolute[joint_name]
        observed_chain = (joint.find("parent").attrib["link"], joint.find("child").attrib["link"])
        require(observed_chain == expected_chain, f"{joint_name}: {observed_chain}")
        limit = joint.find("limit")
        values = [float(limit.attrib[key]) for key in ("lower", "upper", "velocity", "effort")]
        require(all(math.isfinite(value) for value in values), f"{joint_name}: non-finite limit")
        require(values[0] < values[1] and values[2] > 0.0, f"{joint_name}: invalid limit range")

    share_root = Path(get_package_share_directory("abb_lab_description"))
    reference_path = share_root / "config" / "irb1200_5_90_asset_reference.yaml"
    require(reference_path.is_file(), f"missing installed limit reference: {reference_path}")
    lock_path = share_root / "config" / "abb-irb1200-assets.sha256"
    repos_path = share_root / "config" / "abb-model-assets.repos"
    snapshot_root = share_root / "third_party" / "abb_irb1200_support"
    require(lock_path.is_file(), f"missing installed asset lock: {lock_path}")
    require(repos_path.is_file(), f"missing installed source manifest: {repos_path}")
    require(
        "05cbabe774ef026b1cd36fe6b253b55f55e16340"
        in repos_path.read_text(encoding="utf-8"),
        "installed source manifest lost the full-SHA pin",
    )

    locked_assets: dict[str, str] = {}
    for line in lock_path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#"):
            continue
        digest, relative_path = line.split(maxsplit=1)
        locked_assets[relative_path] = digest
    require(len(locked_assets) == 19, f"expected 19 locked assets, observed {len(locked_assets)}")
    for relative_path, expected_digest in locked_assets.items():
        installed_asset = snapshot_root / relative_path
        require(installed_asset.is_file(), f"missing installed locked asset: {installed_asset}")
        require(
            sha256(installed_asset) == expected_digest,
            f"installed asset hash mismatch: {relative_path}",
        )

    mesh_elements = root.findall("./link/visual/geometry/mesh") + root.findall("./link/collision/geometry/mesh")
    require(len(mesh_elements) == 14, f"expected 14 mesh references, observed {len(mesh_elements)}")
    prefix = "package://abb_lab_description/"
    for mesh in mesh_elements:
        uri = mesh.attrib["filename"]
        require(uri.startswith(prefix), f"unexpected mesh URI: {uri}")
        resolved_path = share_root / uri.removeprefix(prefix)
        require(resolved_path.is_file(), f"unresolved installed mesh: {resolved_path}")
        require(resolved_path.stat().st_size > 80, f"empty installed mesh: {resolved_path}")
        raw_mesh = snapshot_root / "source" / "abb_irb1200_support" / uri.removeprefix(prefix)
        require(raw_mesh.is_file(), f"missing raw locked mesh: {raw_mesh}")
        require(sha256(resolved_path) == sha256(raw_mesh), f"active mesh differs from lock: {uri}")

    print("ABB_GEOMETRY_URDF=PASS")
    print(f"LINK_COUNT={len(links)}")
    print(f"JOINT_COUNT={len(joints)}")
    print(f"REVOLUTE_JOINT_COUNT={len(revolute)}")
    print(f"MESH_REFERENCE_COUNT={len(mesh_elements)}")
    print(f"INSTALLED_LOCKED_ASSET_COUNT={len(locked_assets)}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (AssertionError, ET.ParseError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
