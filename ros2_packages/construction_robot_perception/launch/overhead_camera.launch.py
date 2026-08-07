from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    perception_share = FindPackageShare(
        'construction_robot_perception'
    )

    camera_config = PathJoinSubstitution([
        perception_share,
        'config',
        'usb_cam_obsbot.yaml',
    ])

    apriltag_config = PathJoinSubstitution([
        perception_share,
        'config',
        'tags_multi_truck.yaml',
    ])

    # ---------------------------------------------------------
    # USB camera
    # ---------------------------------------------------------

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[
            camera_config,
        ],
    )

    # ---------------------------------------------------------
    # AprilTag detector
    #
    # Preserve old Camera.py remapping:
    #
    #   image_rect  -> /image_raw
    #   camera_info -> /camera_info
    #
    # Start 2 seconds after the camera, matching the old runner.
    # ---------------------------------------------------------

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[
            apriltag_config,
        ],
        remappings=[
            ('image_rect', '/image_raw'),
            ('camera_info', '/camera_info'),
        ],
    )

    delayed_apriltag = TimerAction(
        period=2.0,
        actions=[
            apriltag_node,
        ],
    )

    return LaunchDescription([
        usb_cam_node,
        delayed_apriltag,
    ])