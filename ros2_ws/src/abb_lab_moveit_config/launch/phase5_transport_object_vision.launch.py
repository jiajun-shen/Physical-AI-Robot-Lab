"""Launch P5's diagonal RGB-D transport localizer.

This opt-in P5 process consumes only its RGB, depth, CameraInfo and TF
boundary. It does not access a simulator scene query or hidden scene metadata.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the P5 transport-only RGB-D perception process."""

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
                name="phase5_transport_object_vision",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "perception_mode": "vision"}],
                remappings=[
                    ("/lab_camera/color/image_raw", "/phase5_transport_camera/color/image_raw"),
                    ("/lab_camera/depth/image_raw", "/phase5_transport_camera/depth/image_raw"),
                    ("/lab_camera/color/camera_info", "/phase5_transport_camera/color/camera_info"),
                    ("/perception/detections", "/phase5/perception/transport_detections"),
                    (
                        "/perception/camera_object_poses",
                        "/phase5/perception/transport_camera_object_poses",
                    ),
                    (
                        "/perception/world_object_poses",
                        "/phase5/perception/transport_world_object_poses",
                    ),
                ],
            ),
        ]
    )
