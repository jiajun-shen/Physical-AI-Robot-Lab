"""Start the isolated Phase-4 RGB-D lab profile without a second control chain.

The existing gripper/lab bringup remains the sole owner of Gazebo, the robot,
the controller manager, and the clock bridge.  This wrapper selects the
separate RGB-D world and adds only GZ-to-ROS image transport plus the calibrated
static camera frame chain.  It does not expose Gazebo model pose data to ROS.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


CAMERA_X_M = "0.10"
CAMERA_Y_M = "-1.30"
CAMERA_Z_M = "2.40"
CAMERA_ROLL_RAD = "0"
CAMERA_PITCH_RAD = "0.833"
CAMERA_YAW_RAD = "1.107"
HALF_PI_RAD = "1.57079632679"


def generate_launch_description() -> LaunchDescription:
    """Build the P4 camera transport graph around the existing P3 bringup."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_path = LaunchConfiguration("world_path")
    world_name = LaunchConfiguration("world_name")
    base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("abb_lab_bringup"),
                    "launch",
                    "irb1200_gripper_lab_headless.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world_path": world_path,
            "world_name": world_name,
        }.items(),
    )
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/lab_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/lab_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/lab_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "--ros-args",
            "-p",
            "override_frame_id:=lab_camera_optical_frame",
        ],
        remappings=[
            ("/lab_camera/image", "/lab_camera/color/image_raw"),
            ("/lab_camera/depth_image", "/lab_camera/depth/image_raw"),
            ("/lab_camera/camera_info", "/lab_camera/color/camera_info"),
        ],
    )
    world_to_camera_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "--x",
            CAMERA_X_M,
            "--y",
            CAMERA_Y_M,
            "--z",
            CAMERA_Z_M,
            "--roll",
            CAMERA_ROLL_RAD,
            "--pitch",
            CAMERA_PITCH_RAD,
            "--yaw",
            CAMERA_YAW_RAD,
            "--frame-id",
            "world",
            "--child-frame-id",
            "lab_camera_link",
        ],
    )
    camera_link_to_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            f"-{HALF_PI_RAD}",
            "--pitch",
            "0",
            "--yaw",
            f"-{HALF_PI_RAD}",
            "--frame-id",
            "lab_camera_link",
            "--child-frame-id",
            "lab_camera_optical_frame",
        ],
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
                        "manipulation_lab_rgbd.sdf",
                    ]
                ),
                description="Absolute or package-resolved Phase-4 world path.",
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="manipulation_lab",
                description="Name of the SDF world that receives the ABB model.",
            ),
            base_bringup,
            camera_bridge,
            world_to_camera_link,
            camera_link_to_optical,
        ]
    )
