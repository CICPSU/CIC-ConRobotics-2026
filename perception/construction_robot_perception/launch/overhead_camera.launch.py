from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    perception_share = FindPackageShare(
        "construction_robot_perception"
    )

    camera_config = PathJoinSubstitution(
        [
            perception_share,
            "config",
            "usb_cam_obsbot.yaml",
        ]
    )

    apriltag_config = PathJoinSubstitution(
        [
            perception_share,
            "config",
            "tags_multi_truck.yaml",
        ]
    )

    # ---------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------

    start_camera_argument = DeclareLaunchArgument(
        "start_camera",
        default_value="true",
        description="Start the overhead USB camera.",
    )

    start_apriltag_argument = DeclareLaunchArgument(
        "start_apriltag",
        default_value="true",
        description="Start the AprilTag detector.",
    )

    # ---------------------------------------------------------
    # USB camera
    # ---------------------------------------------------------

    usb_cam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        output="screen",
        parameters=[camera_config],
        condition=IfCondition(
            LaunchConfiguration("start_camera")
        ),
    )

    # ---------------------------------------------------------
    # AprilTag detector
    # ---------------------------------------------------------

    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        output="screen",
        parameters=[apriltag_config],
        remappings=[
            ("image_rect", "/image_raw"),
            ("camera_info", "/camera_info"),
        ],
        condition=IfCondition(
            LaunchConfiguration("start_apriltag")
        ),
    )

    # Give the camera time to start publishing before
    # starting the AprilTag detector.
    delayed_apriltag = TimerAction(
        period=2.0,
        actions=[apriltag_node],
    )

    return LaunchDescription(
        [
            start_camera_argument,
            start_apriltag_argument,
            usb_cam_node,
            delayed_apriltag,
        ]
    )