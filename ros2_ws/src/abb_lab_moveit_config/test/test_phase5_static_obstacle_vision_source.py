"""Source contracts for P5's separate vision-only static-obstacle path."""

from __future__ import annotations

import ast
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAB_ROOT = PACKAGE_ROOT.parent / "abb_lab_lab"
P4_WORLD_PATH = LAB_ROOT / "worlds" / "manipulation_lab_rgbd.sdf"
P5_WORLD_PATH = LAB_ROOT / "worlds" / "manipulation_lab_phase5_rgbd.sdf"
SCENE_CONFIG_PATH = LAB_ROOT / "config" / "manipulation_scene.yaml"
GENERATOR_PATH = LAB_ROOT / "scripts" / "generate_scene_manifest.py"
P5_CONFIG_PATH = PACKAGE_ROOT / "config" / "phase5_static_obstacle_vision.yaml"
P5_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "irb1200_gripper_moveit_rgbd_headless.launch.py"
OBSTACLE_VISION_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "phase5_static_obstacle_vision.launch.py"
GUARD_VISION_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "phase5_guard_object_vision.launch.py"
TRANSPORT_VISION_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "phase5_transport_object_vision.launch.py"
LIFT_VISION_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "phase5_lift_object_vision.launch.py"

MARKERS = {
    "obstacle_left": "p5_orange_top_fiducial",
    "obstacle_right": "p5_violet_top_fiducial",
}
MARKER_CLASSES = tuple(MARKERS.values())


def model_by_name(world: ET.Element, name: str) -> ET.Element:
    for model in world.findall("model"):
        if model.attrib.get("name") == name:
            return model
    raise AssertionError(f"model is missing: {name}")


def pose_values(model: ET.Element) -> tuple[float, ...]:
    pose = model.findtext("pose")
    if pose is None:
        raise AssertionError(f"model {model.attrib.get('name')} has no pose")
    values = tuple(float(item) for item in pose.split())
    if len(values) != 6:
        raise AssertionError(f"invalid pose: {pose}")
    return values


def load_yaml(path: Path) -> dict:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise AssertionError(f"{path} must be a YAML mapping")
    return value


def test_p5_world_keeps_p4_collision_geometry_and_adds_markers_plus_p5_cameras() -> None:
    p4_world = ET.parse(P4_WORLD_PATH).getroot().find("world")
    p5_world = ET.parse(P5_WORLD_PATH).getroot().find("world")
    assert p4_world is not None
    assert p5_world is not None
    assert p5_world.attrib.get("name") == "manipulation_lab"

    for obstacle, marker in MARKERS.items():
        p4_model = model_by_name(p4_world, obstacle)
        p5_model = model_by_name(p5_world, obstacle)
        assert pose_values(p5_model) == pose_values(p4_model)
        p4_link = p4_model.find("link")
        p5_link = p5_model.find("link")
        assert p4_link is not None
        assert p5_link is not None
        assert p5_link.findtext("collision/geometry/box/size") == "0.10 0.10 0.20"
        assert p5_link.findtext("collision/geometry/box/size") == p4_link.findtext(
            "collision/geometry/box/size"
        )
        assert [item.attrib.get("name") for item in p5_link.findall("collision")] == [
            "collision"
        ]
        marker_visual = p5_link.find(f"visual[@name='{marker}']")
        assert marker_visual is not None
        assert marker_visual.findtext("pose") == "0 0 0.102 0 0 0"
        assert marker_visual.findtext("geometry/cylinder/radius") == "0.025"
        assert marker_visual.findtext("geometry/cylinder/length") == "0.004"
        assert marker_visual.find("collision") is None

    guard_camera = model_by_name(p5_world, "phase5_guard_rgbd_camera")
    assert pose_values(guard_camera) == (0.80, 1.45, 2.40, 0.0, 0.833, -1.674)
    assert guard_camera.findtext("link/sensor/topic") == "/phase5_guard_camera"
    assert guard_camera.find("link/collision") is None
    transport_camera = model_by_name(p5_world, "phase5_transport_rgbd_camera")
    assert pose_values(transport_camera) == (0.20, 1.45, 2.40, 0.0, 0.863, -1.156)
    assert transport_camera.findtext("link/sensor/topic") == "/phase5_transport_camera"
    assert transport_camera.find("link/collision") is None
    lift_camera = model_by_name(p5_world, "phase5_lift_rgbd_camera")
    assert pose_values(lift_camera) == (1.20, -1.35, 2.40, 0.0, 0.840, 1.958)
    assert lift_camera.findtext("link/sensor/topic") == "/phase5_lift_camera"
    assert lift_camera.find("link/collision") is None


def test_p5_obstacle_config_is_two_marker_rgbd_contract_with_conservative_geometry() -> None:
    config = load_yaml(P5_CONFIG_PATH)
    geometry = config["phase5_static_obstacle_geometry"]
    assert geometry["output_topic"] == "/phase5/perception/static_obstacle_poses"
    assert tuple(geometry["expected_marker_class_ids"]) == MARKER_CLASSES
    assert geometry["marker_top_to_box_center_z_offset_m"] == 0.104
    assert geometry["collision_box_size_xyz_m"] == [0.160, 0.160, 0.220]
    assert geometry["confidence_floor"] == 0.60
    assert geometry["source"] == "rgbd_camera_info_tf_only"

    classes = config["classes"]
    assert [entry["class_id"] for entry in classes] == list(MARKER_CLASSES)
    for entry in classes:
        assert entry["center_offset_m"] == 0.0
        assert entry["geometry_kind"] == "surface_ray"
        assert entry["min_pixels"] > 0
        assert entry["max_pixels"] >= entry["min_pixels"]
        assert entry["hsv_ranges"]


def test_generator_seeds_p5_template_without_needing_a_new_model_pose_path() -> None:
    with tempfile.TemporaryDirectory(prefix="p5-static-obstacle-vision-") as directory:
        root = Path(directory)
        manifest_path = root / "seed42.json"
        world_path = root / "seed42.sdf"
        subprocess.run(
            [
                sys.executable,
                str(GENERATOR_PATH),
                "--config",
                str(SCENE_CONFIG_PATH),
                "--seed",
                "42",
                "--manifest-output",
                str(manifest_path),
                "--world-template",
                str(P5_WORLD_PATH),
                "--world-output",
                str(world_path),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        seeded_world = ET.parse(world_path).getroot().find("world")
        assert seeded_world is not None
        for entry in manifest["obstacles"]:
            model = model_by_name(seeded_world, entry["name"])
            assert pose_values(model) == tuple(
                entry["pose"]["position_m"] + entry["pose"]["rpy_rad"]
            )
            assert model.find(f"link/visual[@name='{MARKERS[entry['name']]}']") is not None


def test_p5_launch_uses_the_p5_template_and_p5_localizers_remap_only_typed_outputs() -> None:
    p5_launch_source = P5_LAUNCH_PATH.read_text(encoding="utf-8")
    obstacle_launch_source = OBSTACLE_VISION_LAUNCH_PATH.read_text(encoding="utf-8")
    guard_launch_source = GUARD_VISION_LAUNCH_PATH.read_text(encoding="utf-8")
    transport_launch_source = TRANSPORT_VISION_LAUNCH_PATH.read_text(encoding="utf-8")
    lift_launch_source = LIFT_VISION_LAUNCH_PATH.read_text(encoding="utf-8")
    ast.parse(p5_launch_source, filename=str(P5_LAUNCH_PATH))
    ast.parse(obstacle_launch_source, filename=str(OBSTACLE_VISION_LAUNCH_PATH))
    ast.parse(guard_launch_source, filename=str(GUARD_VISION_LAUNCH_PATH))
    ast.parse(transport_launch_source, filename=str(TRANSPORT_VISION_LAUNCH_PATH))
    ast.parse(lift_launch_source, filename=str(LIFT_VISION_LAUNCH_PATH))
    assert "manipulation_lab_phase5_rgbd.sdf" in p5_launch_source
    for token in (
        'package="physical_ai_perception"',
        'executable="vision_localizer_node"',
        '"perception_mode": "vision"',
        "phase5_static_obstacle_vision.yaml",
        "/phase5/perception/static_obstacle_detections",
        "/phase5/perception/static_obstacle_camera_poses",
        "/phase5/perception/static_obstacle_poses",
    ):
        assert token in obstacle_launch_source
    for forbidden in ("ros_gz", "model_pose", "ground_truth", "manifest", "seed"):
        assert forbidden not in obstacle_launch_source.lower()
    for token in (
        'package="physical_ai_perception"',
        'executable="vision_localizer_node"',
        '"perception_mode": "vision"',
        "/phase5_guard_camera/color/image_raw",
        "/phase5_guard_camera/depth/image_raw",
        "/phase5_guard_camera/color/camera_info",
        "/phase5/perception/guard_world_object_poses",
    ):
        assert token in guard_launch_source
    for forbidden in ("ros_gz", "model_pose", "ground_truth", "manifest", "seed"):
        assert forbidden not in guard_launch_source.lower()
    for token in (
        'package="physical_ai_perception"',
        'executable="vision_localizer_node"',
        '"perception_mode": "vision"',
        "/phase5_transport_camera/color/image_raw",
        "/phase5_transport_camera/depth/image_raw",
        "/phase5_transport_camera/color/camera_info",
        "/phase5/perception/transport_world_object_poses",
    ):
        assert token in transport_launch_source
    for forbidden in ("ros_gz", "model_pose", "ground_truth", "manifest", "seed"):
        assert forbidden not in transport_launch_source.lower()
    for token in (
        'package="physical_ai_perception"',
        'executable="vision_localizer_node"',
        '"perception_mode": "vision"',
        "/phase5_lift_camera/color/image_raw",
        "/phase5_lift_camera/depth/image_raw",
        "/phase5_lift_camera/color/camera_info",
        "/phase5/perception/lift_world_object_poses",
    ):
        assert token in lift_launch_source
    for forbidden in ("ros_gz", "model_pose", "ground_truth", "manifest", "seed"):
        assert forbidden not in lift_launch_source.lower()
    for token in (
        "/phase5_guard_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/phase5_guard_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/phase5_guard_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "phase5_guard_camera_optical_frame",
        "/phase5_transport_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/phase5_transport_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/phase5_transport_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "phase5_transport_camera_optical_frame",
        "/phase5_lift_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/phase5_lift_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/phase5_lift_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "phase5_lift_camera_optical_frame",
    ):
        assert token in p5_launch_source
