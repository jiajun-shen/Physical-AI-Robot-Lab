"""Run Phase 5's single gripper-control chain with RGB-D perception and MoveIt.

This launch deliberately layers move_group and three narrow Gazebo transport
bridges on top of the existing gripper/RGB-D bringup. The Phase-5 harness
starts the vision node only after this graph is stable, matching the proven
Phase-4 startup ordering. It never creates a second controller manager and
it exposes no Gazebo model-pose topic to ROS.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    """Build the P5 RGB-D, gripper, MoveIt, and guarded attachment graph."""

    use_sim_time = LaunchConfiguration("use_sim_time")
    world_path = LaunchConfiguration("world_path")
    world_name = LaunchConfiguration("world_name")
    phase5_arm_initial_positions = str(
        Path(get_package_share_directory("abb_lab_gripper"))
        / "config"
        / "phase5_safe_arm_initial_positions.yaml"
    )
    # The historical P3/P4 lab spawned the arm at ground height, which is
    # fine for free-space gripper work but puts low-elbow IK solutions through
    # the 0.75 m work table.  P5 uses a fixed simulation pedestal at the
    # public table height; the same xacro mapping is supplied to Gazebo and
    # MoveIt so collision checking sees the actual mount.
    phase5_world_to_base_z = "0.75"

    rgbd_gripper_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("abb_lab_bringup"),
                    "launch",
                    "irb1200_gripper_lab_rgbd_headless.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world_path": world_path,
            "world_name": world_name,
            "arm_initial_positions_file": phase5_arm_initial_positions,
            "world_to_base_z": phase5_world_to_base_z,
        }.items(),
    )

    # Use the installed gripper share directly, keeping the exact spawned
    # xacro and MoveIt robot description identical without copied geometry.
    gripper_xacro = str(
        Path(get_package_share_directory("abb_lab_gripper"))
        / "urdf"
        / "irb1200_5_90_gz_gripper_control.urdf.xacro"
    )
    moveit_config = (
        MoveItConfigsBuilder(
            "irb1200_5_90_gz_gripper_control", package_name="abb_lab_moveit_config"
        )
        .robot_description(
            file_path=gripper_xacro,
            mappings={
                "arm_initial_positions_file": phase5_arm_initial_positions,
                "world_to_base_z": phase5_world_to_base_z,
            },
        )
        .robot_description_semantic(
            file_path="config/irb1200_5_90_gz_gripper_control.srdf"
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits_gripper.yaml")
        .planning_pipelines(pipelines=["ompl"], load_all=False)
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .to_moveit_configs()
    )
    moveit_parameters = moveit_config.to_dict()
    if "robot_description_planning" not in moveit_parameters:
        raise RuntimeError("Phase-5 MoveIt configuration lacks joint limits")

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_parameters,
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
            },
        ],
    )

    # The P5 skill dynamically adds the detachable joint only after its
    # vision/TCP proximity and gripper-state guards.  These bridges carry its
    # lifecycle commands/state, never a model pose.
    attachment_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/phase5/grasp/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/phase5/grasp/detach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/phase5/grasp/state@std_msgs/msg/String[gz.msgs.StringMsg",
        ],
    )
    # A second P5-only RGB-D camera observes the object from +Y. Its narrow
    # bridge and static TF chain carry exactly the same image/depth/CameraInfo
    # boundary as Phase 4; no Gazebo model-pose data crosses into ROS.
    guard_camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/phase5_guard_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/phase5_guard_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/phase5_guard_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "--ros-args",
            "-p",
            "override_frame_id:=phase5_guard_camera_optical_frame",
        ],
        remappings=[
            ("/phase5_guard_camera/image", "/phase5_guard_camera/color/image_raw"),
            ("/phase5_guard_camera/depth_image", "/phase5_guard_camera/depth/image_raw"),
            ("/phase5_guard_camera/camera_info", "/phase5_guard_camera/color/camera_info"),
        ],
    )
    world_to_guard_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "--x", "0.80", "--y", "1.45", "--z", "2.40",
            "--roll", "0", "--pitch", "0.833", "--yaw", "-1.674",
            "--frame-id", "world", "--child-frame-id", "phase5_guard_camera_link",
        ],
    )
    guard_camera_to_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "-1.57079632679", "--pitch", "0", "--yaw", "-1.57079632679",
            "--frame-id", "phase5_guard_camera_link",
            "--child-frame-id", "phase5_guard_camera_optical_frame",
        ],
    )
    transport_camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/phase5_transport_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/phase5_transport_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/phase5_transport_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "--ros-args",
            "-p",
            "override_frame_id:=phase5_transport_camera_optical_frame",
        ],
        remappings=[
            ("/phase5_transport_camera/image", "/phase5_transport_camera/color/image_raw"),
            ("/phase5_transport_camera/depth_image", "/phase5_transport_camera/depth/image_raw"),
            ("/phase5_transport_camera/camera_info", "/phase5_transport_camera/color/camera_info"),
        ],
    )
    world_to_transport_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "--x", "0.20", "--y", "1.45", "--z", "2.40",
            "--roll", "0", "--pitch", "0.863", "--yaw", "-1.156",
            "--frame-id", "world", "--child-frame-id", "phase5_transport_camera_link",
        ],
    )
    transport_camera_to_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "-1.57079632679", "--pitch", "0", "--yaw", "-1.57079632679",
            "--frame-id", "phase5_transport_camera_link",
            "--child-frame-id", "phase5_transport_camera_optical_frame",
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use Gazebo time."
            ),
            DeclareLaunchArgument(
                "world_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("abb_lab_lab"),
                        "worlds",
                        "manipulation_lab_phase5_rgbd.sdf",
                    ]
                ),
                description=(
                    "Seeded Phase-5 RGB-D world path with visual-only static "
                    "obstacle fiducials for the separately launched P5 localizer."
                ),
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="manipulation_lab",
                description="World that receives the one ABB model.",
            ),
            rgbd_gripper_bringup,
            guard_camera_bridge,
            world_to_guard_camera,
            guard_camera_to_optical,
            transport_camera_bridge,
            world_to_transport_camera,
            transport_camera_to_optical,
            attachment_bridge,
            move_group,
        ]
    )
