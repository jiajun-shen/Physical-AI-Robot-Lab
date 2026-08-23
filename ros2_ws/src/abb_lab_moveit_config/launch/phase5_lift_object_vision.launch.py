"""Launch P5's opposite-diagonal RGB-D lift localizer.

This opt-in observer consumes only RGB, depth, CameraInfo and TF. It is a
visual fallback for side-grasp occlusion and has no simulator state input.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the P5 lift-only RGB-D perception process."""

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
                name="phase5_lift_object_vision",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "perception_mode": "vision"}],
                remappings=[
                    ("/lab_camera/color/image_raw", "/phase5_lift_camera/color/image_raw"),
                    ("/lab_camera/depth/image_raw", "/phase5_lift_camera/depth/image_raw"),
                    ("/lab_camera/color/camera_info", "/phase5_lift_camera/color/camera_info"),
                    ("/perception/detections", "/phase5/perception/lift_detections"),
                    (
                        "/perception/camera_object_poses",
                        "/phase5/perception/lift_camera_object_poses",
                    ),
                    (
                        "/perception/world_object_poses",
                        "/phase5/perception/lift_world_object_poses",
                    ),
                ],
            ),
        ]
    )
