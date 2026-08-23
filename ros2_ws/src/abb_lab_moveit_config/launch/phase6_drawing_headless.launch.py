"""Launch the one-controller P6 drawing-tool Gazebo/MoveIt graph.

The Phase-6 tool is part of the exact robot description supplied both to
Gazebo and MoveIt.  This launch adds no controller manager and transports no
Gazebo model state into ROS.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_path = LaunchConfiguration("world_path")
    world_name = LaunchConfiguration("world_name")
    execution_duration_scaling = LaunchConfiguration("execution_duration_scaling")
    execution_goal_duration_margin = LaunchConfiguration("execution_goal_duration_margin")
    gripper_share = Path(get_package_share_directory("abb_lab_gripper"))
    drawing_xacro = str(gripper_share / "urdf" / "irb1200_5_90_gz_drawing_pen_control.urdf.xacro")
    safe_initial_positions = str(gripper_share / "config" / "phase5_safe_arm_initial_positions.yaml")
    drawing_controllers = str(gripper_share / "config" / "phase6_drawing_controllers.yaml")
    # Jazzy's Gazebo position-command loop documents 1.0 as the largest
    # non-oscillatory gain.  Keep the real drawing run at that limit; path
    # discretization, not an unstable control override, handles tight curves.
    drawing_position_proportional_gain = "1.0"
    world_to_base_z = "0.75"

    rgbd_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("abb_lab_bringup"), "launch", "irb1200_gripper_lab_rgbd_headless.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world_path": world_path,
            "world_name": world_name,
            "arm_initial_positions_file": safe_initial_positions,
            "world_to_base_z": world_to_base_z,
            "robot_description_xacro": drawing_xacro,
            "controller_config_file": drawing_controllers,
            "position_proportional_gain": drawing_position_proportional_gain,
        }.items(),
    )
    moveit_config = (
        MoveItConfigsBuilder("irb1200_5_90_gz_gripper_control", package_name="abb_lab_moveit_config")
        .robot_description(
            file_path=drawing_xacro,
            mappings={
                "arm_initial_positions_file": safe_initial_positions,
                "world_to_base_z": world_to_base_z,
                "controller_config_file": drawing_controllers,
                "position_proportional_gain": drawing_position_proportional_gain,
            },
        )
        .robot_description_semantic(file_path="config/irb1200_5_90_gz_drawing_pen_control.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits_gripper.yaml")
        .planning_pipelines(pipelines=["ompl"], load_all=False)
        .trajectory_execution(file_path="config/moveit_controllers.yaml", moveit_manage_controllers=False)
        .to_moveit_configs()
    )
    parameters = moveit_config.to_dict()
    if "robot_description_planning" not in parameters:
        raise RuntimeError("Phase-6 MoveIt configuration lacks joint limits")
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            parameters,
            {
                "allow_trajectory_execution": True,
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
                "capabilities": "",
                "disable_capabilities": "",
                "use_sim_time": use_sim_time,
                "trajectory_execution.allowed_execution_duration_scaling": execution_duration_scaling,
                "trajectory_execution.allowed_goal_duration_margin": execution_goal_duration_margin,
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use Gazebo time."),
            DeclareLaunchArgument(
                "world_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("abb_lab_lab"), "worlds", "manipulation_lab_phase5_rgbd.sdf"]
                ),
                description="Drawing-board RGB-D world path.",
            ),
            DeclareLaunchArgument("world_name", default_value="manipulation_lab", description="Gazebo world name."),
            DeclareLaunchArgument(
                "execution_duration_scaling",
                default_value="2.2",
                description="Bounded P6 Cartesian execution monitor scaling.",
            ),
            DeclareLaunchArgument(
                "execution_goal_duration_margin",
                default_value="2.0",
                description="Bounded P6 Cartesian execution monitor margin in seconds.",
            ),
            rgbd_bringup,
            move_group,
        ]
    )
