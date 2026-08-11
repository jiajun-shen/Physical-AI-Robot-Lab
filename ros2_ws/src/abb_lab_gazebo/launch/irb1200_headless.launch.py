"""Launch the ABB IRB1200 simulation-only control slice without a GUI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription
from launch.actions import LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def continue_only_after_success(process_label, success_actions):
    """Return an OnProcessExit callback that shuts down on a failed stage."""

    def handle_exit(event, _context):
        if event.returncode == 0:
            return success_actions
        reason = f"{process_label} exited with return code {event.returncode}"
        return [
            LogInfo(msg=["ERROR: ", reason]),
            EmitEvent(event=Shutdown(reason=reason)),
        ]

    return handle_exit


def generate_launch_description():
    """Build the deterministic headless Gazebo launch graph."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_path = PathJoinSubstitution(
        [FindPackageShare("abb_lab_gazebo"), "worlds", "irb1200_minimal.sdf"]
    )
    description_path = PathJoinSubstitution(
        [
            FindPackageShare("abb_lab_description"),
            "urdf",
            "irb1200_5_90_gz_control.urdf.xacro",
        ]
    )
    controller_config = PathJoinSubstitution(
        [
            FindPackageShare("abb_lab_control"),
            "config",
            "irb1200_controllers.yaml",
        ]
    )
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                description_path,
                " controller_config_file:=",
                controller_config,
            ]
        ),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": ["-s -r -v 1 ", world_path],
            "on_exit_shutdown": "true",
        }.items(),
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time}
        ],
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            "abb_lab",
            "-topic",
            "robot_description",
            "-name",
            "abb_irb1200",
            "-allow_renaming=false",
        ],
    )

    bounded_spawner_arguments = [
        "--controller-manager",
        "/controller_manager",
        "--controller-manager-timeout",
        "20",
        "--switch-timeout",
        "10",
        "--service-call-timeout",
        "10",
    ]
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", *bounded_spawner_arguments],
    )
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_trajectory_controller",
            "--param-file",
            controller_config,
            *bounded_spawner_arguments,
        ],
    )

    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=continue_only_after_success(
                "ABB entity creation", [joint_state_broadcaster_spawner]
            ),
        )
    )
    start_trajectory_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=continue_only_after_success(
                "joint_state_broadcaster spawner",
                [joint_trajectory_controller_spawner],
            ),
        )
    )
    stop_after_trajectory_spawner_failure = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=continue_only_after_success(
                "joint_trajectory_controller spawner", []
            ),
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo simulation clock.",
            ),
            gazebo,
            clock_bridge,
            state_publisher,
            start_joint_state_broadcaster,
            start_trajectory_controller,
            stop_after_trajectory_spawner_failure,
            spawn_robot,
        ]
    )
