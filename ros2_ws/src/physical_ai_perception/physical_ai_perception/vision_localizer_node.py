"""Publish vision-only typed object detections and RGB-D/TF2 localizations."""

from __future__ import annotations

import copy
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PointStamped
from physical_ai_interfaces.msg import (
    ObjectDetection,
    ObjectDetectionArray,
    ObjectPose,
    ObjectPoseArray,
)
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from physical_ai_perception.vision_pipeline import (
    DetectionCandidate,
    VisionInputError,
    apply_center_offset,
    detect_components,
    estimate_planar_rectangle_center,
    estimate_depth,
    load_class_specs,
    project_pixel,
)


def _stamp_ns(message: Image | CameraInfo) -> int:
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def _header_with_frame(source_header, frame_id: str):
    header = copy.deepcopy(source_header)
    header.frame_id = frame_id
    return header


class VisionLocalizer(Node):
    """Synchronize ROS sensor observations and transform only their derived XYZ."""

    def __init__(self) -> None:
        super().__init__("vision_localizer")
        self.declare_parameter("perception_mode", "vision")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("config_path", "")
        self.declare_parameter("max_sync_delta_sec", 0.05)
        self.declare_parameter("minimum_depth_samples", 20)
        mode = str(self.get_parameter("perception_mode").value)
        if mode != "vision":
            raise RuntimeError("vision_localizer accepts only perception_mode=vision")
        self.world_frame = str(self.get_parameter("world_frame").value)
        if not self.world_frame:
            raise RuntimeError("world_frame must be nonempty")
        self.max_sync_delta_ns = int(
            float(self.get_parameter("max_sync_delta_sec").value) * 1_000_000_000
        )
        self.minimum_depth_samples = int(
            self.get_parameter("minimum_depth_samples").value
        )
        if self.max_sync_delta_ns < 0 or self.minimum_depth_samples < 1:
            raise RuntimeError("synchronization and depth-sample parameters are invalid")
        config_value = str(self.get_parameter("config_path").value)
        if config_value:
            config_path = Path(config_value)
        else:
            config_path = Path(get_package_share_directory("physical_ai_perception")) / "config" / "vision_classes.yaml"
        self.specs = load_class_specs(config_path)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.detection_publisher = self.create_publisher(
            ObjectDetectionArray, "/perception/detections", 10
        )
        self.camera_pose_publisher = self.create_publisher(
            ObjectPoseArray, "/perception/camera_object_poses", 10
        )
        self.world_pose_publisher = self.create_publisher(
            ObjectPoseArray, "/perception/world_object_poses", 10
        )
        self.color_subscriber = message_filters.Subscriber(
            self,
            Image,
            "/lab_camera/color/image_raw",
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_subscriber = message_filters.Subscriber(
            self,
            Image,
            "/lab_camera/depth/image_raw",
            qos_profile=qos_profile_sensor_data,
        )
        self.info_subscriber = message_filters.Subscriber(
            self,
            CameraInfo,
            "/lab_camera/color/camera_info",
            qos_profile=qos_profile_sensor_data,
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.color_subscriber, self.depth_subscriber, self.info_subscriber],
            queue_size=15,
            slop=float(self.max_sync_delta_ns) / 1_000_000_000.0,
        )
        self.synchronizer.registerCallback(self._on_sensor_triplet)
        self._last_error = ""
        self._sequence = 0
        self.get_logger().info(
            "PERCEPTION_MODE=vision; accepting RGB, depth, CameraInfo, and TF only"
        )

    def _record_error(self, error: Exception) -> None:
        message = str(error)
        if message != self._last_error:
            self.get_logger().warning(message)
            self._last_error = message

    def _validate_headers(self, color: Image, depth: Image, info: CameraInfo) -> None:
        frame_ids = {color.header.frame_id, depth.header.frame_id, info.header.frame_id}
        if len(frame_ids) != 1 or not color.header.frame_id:
            raise VisionInputError("RGB, depth, and CameraInfo frame IDs must match and be nonempty")
        stamps = (_stamp_ns(color), _stamp_ns(depth), _stamp_ns(info))
        if any(stamp <= 0 for stamp in stamps):
            raise VisionInputError("RGB-D messages must have positive simulation timestamps")
        if max(stamps) - min(stamps) > self.max_sync_delta_ns:
            raise VisionInputError("RGB-D messages exceed the configured timestamp delta")
        if color.width != depth.width or color.height != depth.height:
            raise VisionInputError("RGB and depth image dimensions must match")
        if color.width != info.width or color.height != info.height:
            raise VisionInputError("CameraInfo dimensions must match the RGB image")

    def _depth_metres(self, message: Image) -> np.ndarray:
        try:
            raw = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
        except CvBridgeError as exc:
            raise VisionInputError(f"cannot decode depth image: {exc}") from exc
        if message.encoding == "32FC1":
            depth = np.asarray(raw, dtype=np.float32)
        elif message.encoding == "16UC1":
            depth = np.asarray(raw, dtype=np.float32) / 1000.0
        else:
            raise VisionInputError(
                f"unsupported depth encoding {message.encoding!r}; expected 32FC1 or 16UC1"
            )
        if depth.ndim != 2:
            raise VisionInputError("decoded depth image is not single-channel")
        return depth

    def _bgr(self, message: Image) -> np.ndarray:
        try:
            bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as exc:
            raise VisionInputError(f"cannot decode RGB image: {exc}") from exc
        result = np.asarray(bgr, dtype=np.uint8)
        if result.ndim != 3 or result.shape[2] != 3:
            raise VisionInputError("decoded RGB image is not three-channel")
        return result

    def _detection_message(self, candidate: DetectionCandidate, color: Image) -> ObjectDetection:
        message = ObjectDetection()
        message.header = copy.deepcopy(color.header)
        message.detection_id = f"{candidate.class_id}-{_stamp_ns(color)}-{self._sequence}"
        message.class_id = candidate.class_id
        message.roi.x_offset = candidate.x_offset_px
        message.roi.y_offset = candidate.y_offset_px
        message.roi.width = candidate.width_px
        message.roi.height = candidate.height_px
        message.roi.do_rectify = False
        message.image_width = color.width
        message.image_height = color.height
        message.center_u_px = candidate.center_u_px
        message.center_v_px = candidate.center_v_px
        message.mask_pixel_count = candidate.mask_pixel_count
        message.confidence = candidate.confidence
        message.source_mode = ObjectDetection.SOURCE_VISION
        return message

    def _pose_message(
        self,
        detection: ObjectDetection,
        source_header,
        frame_id: str,
        point_xyz: tuple[float, float, float],
        depth_m: float,
        depth_mad_m: float,
        valid_sample_count: int,
    ) -> ObjectPose:
        message = ObjectPose()
        message.header = _header_with_frame(source_header, frame_id)
        message.source_header = copy.deepcopy(source_header)
        message.detection_id = detection.detection_id
        message.class_id = detection.class_id
        message.pose.pose.position.x = point_xyz[0]
        message.pose.pose.position.y = point_xyz[1]
        message.pose.pose.position.z = point_xyz[2]
        message.pose.pose.orientation.w = 1.0
        variance = max(depth_mad_m * depth_mad_m, 0.0001)
        covariance = [0.0] * 36
        covariance[0] = variance
        covariance[7] = variance
        covariance[14] = variance
        covariance[21] = 1_000_000.0
        covariance[28] = 1_000_000.0
        covariance[35] = 1_000_000.0
        message.pose.covariance = covariance
        message.confidence = detection.confidence
        message.depth_m = depth_m
        message.depth_mad_m = depth_mad_m
        message.valid_depth_samples = valid_sample_count
        message.orientation_valid = False
        message.source_mode = ObjectPose.SOURCE_VISION
        return message

    def _on_sensor_triplet(self, color: Image, depth: Image, info: CameraInfo) -> None:
        try:
            self._validate_headers(color, depth, info)
            bgr = self._bgr(color)
            depth_m = self._depth_metres(depth)
            candidates = detect_components(
                bgr,
                self.specs,
                valid_depth_mask=np.isfinite(depth_m) & (depth_m > 0.0),
            )
            self._sequence += 1
            detection_messages: list[ObjectDetection] = []
            camera_poses: list[ObjectPose] = []
            world_poses: list[ObjectPose] = []
            source_header = copy.deepcopy(color.header)
            for candidate in candidates:
                estimate = estimate_depth(
                    depth_m,
                    candidate.mask,
                    minimum_samples=self.minimum_depth_samples,
                )
                if candidate.geometry_kind == "planar_rectangle":
                    optical_surface = estimate_planar_rectangle_center(
                        depth_m,
                        candidate.mask,
                        info.k,
                        minimum_samples=max(self.minimum_depth_samples, 100),
                    )
                else:
                    optical_surface = project_pixel(
                        candidate.center_u_px,
                        candidate.center_v_px,
                        estimate.depth_m,
                        info.k,
                    )
                optical_center = apply_center_offset(
                    optical_surface, candidate.center_offset_m
                )
                detection = self._detection_message(candidate, color)
                camera_pose = self._pose_message(
                    detection,
                    source_header,
                    source_header.frame_id,
                    optical_center,
                    estimate.depth_m,
                    estimate.depth_mad_m,
                    estimate.valid_sample_count,
                )
                detection_messages.append(detection)
                camera_poses.append(camera_pose)
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.world_frame,
                        source_header.frame_id,
                        Time.from_msg(source_header.stamp),
                        timeout=Duration(seconds=0.2),
                    )
                except TransformException as exc:
                    self._record_error(exc)
                    continue
                point = PointStamped()
                point.header = copy.deepcopy(source_header)
                point.point.x = optical_center[0]
                point.point.y = optical_center[1]
                point.point.z = optical_center[2]
                transformed = do_transform_point(point, transform)
                if not all(
                    math.isfinite(value)
                    for value in (
                        transformed.point.x,
                        transformed.point.y,
                        transformed.point.z,
                    )
                ):
                    raise VisionInputError("TF2 returned a non-finite world point")
                world_poses.append(
                    self._pose_message(
                        detection,
                        source_header,
                        self.world_frame,
                        (
                            transformed.point.x,
                            transformed.point.y,
                            transformed.point.z,
                        ),
                        estimate.depth_m,
                        estimate.depth_mad_m,
                        estimate.valid_sample_count,
                    )
                )
            detections = ObjectDetectionArray()
            detections.header = copy.deepcopy(source_header)
            detections.detections = detection_messages
            camera_pose_array = ObjectPoseArray()
            camera_pose_array.header = copy.deepcopy(source_header)
            camera_pose_array.poses = camera_poses
            world_pose_array = ObjectPoseArray()
            world_pose_array.header = _header_with_frame(source_header, self.world_frame)
            world_pose_array.poses = world_poses
            self.detection_publisher.publish(detections)
            self.camera_pose_publisher.publish(camera_pose_array)
            self.world_pose_publisher.publish(world_pose_array)
        except (VisionInputError, ValueError, TypeError) as exc:
            self._record_error(exc)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: VisionLocalizer | None = None
    try:
        node = VisionLocalizer()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        except Exception:
            if rclpy.ok():
                raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
