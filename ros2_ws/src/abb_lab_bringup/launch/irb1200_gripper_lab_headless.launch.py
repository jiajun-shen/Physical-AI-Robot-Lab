"""Launch one ABB IRB1200-with-gripper model into the headless lab world.

The spawned model owns the sole Gazebo-hosted control resources.  This launch
only starts the controller spawners, in dependency order, after entity
creation has succeeded.
"""

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
    """Continue a controller stage only when its predecessor succeeded."""

    def handle_exit(event, _context):
        if event.returncode == 0:
            return success_actions
        reason = f"{process_label} exited with return code {event.returncode}"
        return [
            LogInfo(msg=["ERROR: ", reason]),
            EmitEvent(event=Shutdown(reason=reason)),
        ]

    return handle_exit


def generate_launch_description() -> LaunchDescription:
    """Build the single-model, headless ABB gripper laboratory launch graph."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_path = LaunchConfiguration("world_path")
    world_name = LaunchConfiguration("world_name")
    arm_initial_positions_file = LaunchConfiguration("arm_initial_positions_file")
    world_to_base_z = LaunchConfiguration("world_to_base_z")
    description_path = PathJoinSubstitution(
        [
            FindPackageShare("abb_lab_gripper"),
            "urdf",
            "irb1200_5_90_gz_gripper_control.urdf.xacro",
        ]
    )
    controller_config = PathJoinSubstitution(
        [
            FindPackageShare("abb_lab_gripper"),
            "config",
            "irb1200_gripper_controllers.yaml",
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
                " arm_initial_positions_file:=",
                arm_initial_positions_file,
                " world_to_base_z:=",
                world_to_base_z,
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
            # Gazebo Sim's default DART engine does not implement the URDF
            # mimic constraint used by the state-only left gripper finger.
            # Bullet Featherstone is installed with the Jazzy/Harmonic stack
            # and is the engine required by Gazebo's own mimic-joint example.
            "gz_args": [
                "-s -r -v 1 --physics-engine "
                "gz-physics-bullet-featherstone-plugin ",
                world_path,
            ],
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
            world_name,
            "-topic",
            "robot_description",
            "-name",
            "abb_irb1200_gripper",
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
    two_finger_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "two_finger_gripper_controller",
            "--param-file",
            controller_config,
            *bounded_spawner_arguments,
        ],
    )

    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=continue_only_after_success(
                "ABB-with-gripper entity creation", [joint_state_broadcaster_spawner]
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
    start_gripper_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=continue_only_after_success(
                "joint_trajectory_controller spawner",
                [two_finger_gripper_controller_spawner],
            ),
        )
    )
    stop_after_gripper_spawner_failure = RegisterEventHandler(
        OnProcessExit(
            target_action=two_finger_gripper_controller_spawner,
            on_exit=continue_only_after_success(
                "two_finger_gripper_controller spawner", []
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
            DeclareLaunchArgument(
                "world_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("abb_lab_lab"),
                        "worlds",
                        "manipulation_lab.sdf",
                    ]
                ),
                description="Absolute or package-resolved SDF world path.",
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="manipulation_lab",
                description="Name of the SDF world that receives the ABB model.",
            ),
            DeclareLaunchArgument(
                "arm_initial_positions_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("abb_lab_gripper"),
                        "config",
                        "arm_initial_positions.yaml",
                    ]
                ),
                description="Bounded arm state-interface initial values.",
            ),
            DeclareLaunchArgument(
                "world_to_base_z",
                default_value="0",
                description="Fixed simulation mount height for base_link in metres.",
            ),
            gazebo,
            clock_bridge,
            state_publisher,
            start_joint_state_broadcaster,
            start_trajectory_controller,
            start_gripper_controller,
            stop_after_gripper_spawner_failure,
            spawn_robot,
        ]
    )
