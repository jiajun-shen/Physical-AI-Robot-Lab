from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODULE_ROOT = PACKAGE_ROOT / "physical_ai_perception"


def test_vision_package_has_no_simulation_or_scene_pose_dependency() -> None:
    forbidden = (
        "gazebo",
        "ros_" + "gz",
        "sub" + "process",
        "ground" + "_truth",
        "generate_scene_manifest",
        "model_pose",
        "scene_manifest",
        "seed",
    )
    for path in MODULE_ROOT.glob("*.py"):
        source = path.read_text(encoding="utf-8").lower()
        for token in forbidden:
            assert token not in source, f"{path.name} contains disallowed token {token!r}"


def test_manifest_allows_only_sensor_tf_and_typed_output_runtime_dependencies() -> None:
    manifest = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
    dependencies = {item.text for item in manifest.findall("exec_depend")}
    assert {
        "cv_bridge",
        "message_filters",
        "physical_ai_interfaces",
        "rclpy",
        "sensor_msgs",
        "tf2_geometry_msgs",
        "tf2_ros",
    } <= dependencies
    assert dependencies.isdisjoint(
        {
            "abb_lab_lab",
            "gazebo_msgs",
            "ros_gz_bridge",
            "ros_gz_interfaces",
            "ros_gz_sim",
        }
    )


def test_node_declares_vision_mode_and_only_sensor_topics() -> None:
    source = (MODULE_ROOT / "vision_localizer_node.py").read_text(encoding="utf-8")
    assert 'declare_parameter("perception_mode", "vision")' in source
    assert 'mode != "vision"' in source
    for required in (
        "/lab_camera/color/image_raw",
        "/lab_camera/depth/image_raw",
        "/lab_camera/color/camera_info",
        'lookup_transform(',
        '"/perception/detections"',
        '"/perception/camera_object_poses"',
        '"/perception/world_object_poses"',
    ):
        assert required in source
