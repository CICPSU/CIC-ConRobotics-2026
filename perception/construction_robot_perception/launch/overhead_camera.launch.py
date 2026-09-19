from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


EXCAVATOR_DEFAULTS = {
    1: 'false',
    2: 'false',
    3: 'true',
    4: 'false',
    5: 'false',
    6: 'false',
}


def generate_launch_description():
    perception_share = FindPackageShare('construction_robot_perception')

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
    swing_adapters_config = PathJoinSubstitution([
        perception_share,
        'config',
        'swing_position_adapters.yaml',
    ])

    launch_arguments = [
        DeclareLaunchArgument(
            'start_camera',
            default_value='true',
            description='Start the overhead USB camera.',
        ),
        DeclareLaunchArgument(
            'start_apriltag',
            default_value='true',
            description='Start the AprilTag detector.',
        ),
    ]

    for excavator_number, default_value in EXCAVATOR_DEFAULTS.items():
        launch_arguments.append(
            DeclareLaunchArgument(
                f'excavator{excavator_number}',
                default_value=default_value,
                description=(
                    'Start the swing adapter for '
                    f'excavator{excavator_number}.'
                ),
            )
        )

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[camera_config],
        condition=IfCondition(LaunchConfiguration('start_camera')),
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[apriltag_config],
        remappings=[
            ('image_rect', '/image_raw'),
            ('camera_info', '/camera_info'),
        ],
        condition=IfCondition(LaunchConfiguration('start_apriltag')),
    )

    swing_adapter_nodes = []
    for excavator_number in EXCAVATOR_DEFAULTS:
        swing_adapter_nodes.append(
            Node(
                package='construction_robot_perception',
                executable='swing_position_adapter',
                name=(
                    'swing_position_adapter_'
                    f'excavator{excavator_number}'
                ),
                output='screen',
                parameters=[swing_adapters_config],
                condition=IfCondition(
                    LaunchConfiguration(f'excavator{excavator_number}')
                ),
            )
        )

    delayed_apriltag = TimerAction(
        period=2.0,
        actions=[apriltag_node],
    )
    delayed_swing_adapters = TimerAction(
        period=3.0,
        actions=swing_adapter_nodes,
    )

    return LaunchDescription([
        *launch_arguments,
        usb_cam_node,
        delayed_apriltag,
        delayed_swing_adapters,
    ])