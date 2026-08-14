from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def read_message(name: str) -> str:
    path = PACKAGE_ROOT / "msg" / name
    assert path.is_file(), path
    return path.read_text(encoding="utf-8")


def test_detection_and_pose_messages_preserve_sensor_and_frame_provenance() -> None:
    detection = read_message("ObjectDetection.msg")
    pose = read_message("ObjectPose.msg")
    detection_array = read_message("ObjectDetectionArray.msg")
    pose_array = read_message("ObjectPoseArray.msg")
    for token in (
        "std_msgs/Header header",
        "string detection_id",
        "string class_id",
        "sensor_msgs/RegionOfInterest roi",
        "float32 center_u_px",
        "float32 center_v_px",
        "uint8 SOURCE_VISION=1",
        "uint8 source_mode",
    ):
        assert token in detection
    for token in (
        "std_msgs/Header header",
        "std_msgs/Header source_header",
        "geometry_msgs/PoseWithCovariance pose",
        "float32 depth_m",
        "float32 depth_mad_m",
        "uint32 valid_depth_samples",
        "bool orientation_valid",
        "uint8 SOURCE_VISION=1",
    ):
        assert token in pose
    assert "physical_ai_interfaces/ObjectDetection[] detections" in detection_array
    assert "physical_ai_interfaces/ObjectPose[] poses" in pose_array
