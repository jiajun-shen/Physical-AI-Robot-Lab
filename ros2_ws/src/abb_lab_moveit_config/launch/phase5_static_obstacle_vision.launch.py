"""Launch P5's separate vision-only static-obstacle localizer.

Run this after the RGB-D/MoveIt graph has reached readiness.  It intentionally
does not start Gazebo, query Gazebo state, or replace Phase-4's five-class
perception node.  The existing vision localizer receives the same RGB, depth,
CameraInfo, and TF inputs, but all three typed outputs are remapped into a P5
namespace so Phase-4 sealing continues to require exactly its five labels.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create only the opt-in P5 static-obstacle perception process."""

    use_sim_time = LaunchConfiguration("use_sim_time")
    config_path = LaunchConfiguration("config_path")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the already-running Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "config_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("abb_lab_moveit_config"),
                        "config",
                        "phase5_static_obstacle_vision.yaml",
                    ]
                ),
                description="P5-only RGB-D fiducial class specification.",
            ),
            Node(
                package="physical_ai_perception",
                executable="vision_localizer_node",
                name="phase5_static_obstacle_vision",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "perception_mode": "vision",
                        "config_path": config_path,
                    }
                ],
                remappings=[
                    (
                        "/perception/detections",
                        "/phase5/perception/static_obstacle_detections",
                    ),
                    (
                        "/perception/camera_object_poses",
                        "/phase5/perception/static_obstacle_camera_poses",
                    ),
                    (
                        "/perception/world_object_poses",
                        "/phase5/perception/static_obstacle_poses",
                    ),
                ],
            ),
        ]
    )
