"""Launch P5's second RGB-D-only object localizer.

The guard camera is an independent P5 world sensor used only when the primary
Phase-4 camera is occluded by the arm. All image, depth, CameraInfo and TF
inputs are remapped into a P5 namespace; it neither starts a simulator nor
accesses hidden scene metadata.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create only the opt-in P5 guard RGB-D perception process."""

    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the already-running Gazebo simulation clock.",
            ),
            Node(
                package="physical_ai_perception",
                executable="vision_localizer_node",
                name="phase5_guard_object_vision",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "perception_mode": "vision"}],
                remappings=[
                    ("/lab_camera/color/image_raw", "/phase5_guard_camera/color/image_raw"),
                    ("/lab_camera/depth/image_raw", "/phase5_guard_camera/depth/image_raw"),
                    ("/lab_camera/color/camera_info", "/phase5_guard_camera/color/camera_info"),
                    ("/perception/detections", "/phase5/perception/guard_detections"),
                    (
                        "/perception/camera_object_poses",
                        "/phase5/perception/guard_camera_object_poses",
                    ),
                    (
                        "/perception/world_object_poses",
                        "/phase5/perception/guard_world_object_poses",
                    ),
                ],
            ),
        ]
    )
