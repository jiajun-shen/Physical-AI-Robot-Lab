"""Launch MoveIt against the project-owned headless ABB Gazebo control slice.

The Gazebo launch owns the only ``gz_ros2_control`` controller manager and
the two controller spawners.  This layer deliberately adds only MoveIt's
``move_group`` process, connected to the already-running trajectory action.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    """Bring up P1.3b Gazebo control followed by the arm-only MoveIt layer."""
    use_sim_time = LaunchConfiguration("use_sim_time")

    gazebo_control_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("abb_lab_gazebo"),
                    "launch",
                    "irb1200_headless.launch.py",
                ]
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "irb1200_5_90_gz_control", package_name="abb_lab_moveit_config"
        )
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .planning_pipelines(pipelines=["ompl"], load_all=False)
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .to_moveit_configs()
    )

    move_group_parameters = {
        "allow_trajectory_execution": True,
        "publish_robot_description_semantic": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
        "capabilities": "",
        "disable_capabilities": "",
    }
    moveit_parameters = moveit_config.to_dict()
    if "robot_description_planning" not in moveit_parameters:
        raise RuntimeError(
            "MoveIt configuration is missing robot_description_planning limits"
        )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_parameters,
            move_group_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo simulation clock.",
            ),
            gazebo_control_bringup,
            move_group,
        ]
    )
