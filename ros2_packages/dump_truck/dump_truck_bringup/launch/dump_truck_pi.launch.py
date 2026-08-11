from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def launch_setup(context, *args, **kwargs):
    # ---------------------------------------------------------
    # Resolve truck name
    #
    # Examples:
    #   truck1
    #   truck2
    #   dumptruck_03
    # ---------------------------------------------------------

    truck_name = LaunchConfiguration('truck_name').perform(context)

    # ---------------------------------------------------------
    # Package paths
    # ---------------------------------------------------------

    bringup_share = get_package_share_directory(
        'dump_truck_bringup'
    )

    truck_hardware_yaml = os.path.join(
        bringup_share,
        'config',
        'hardware',
        f'{truck_name}.yaml',
    )

    # ---------------------------------------------------------
    # Truck-specific topics
    # ---------------------------------------------------------

    cmd_vel_topic = (
        f'/{truck_name}/cmd_vel'
    )

    wheel_states_topic = (
        f'/{truck_name}/wheel_states'
    )

    bucket_action_cmd_topic = (
        f'/{truck_name}/bucket_action_cmd'
    )

    bucket_action_status_topic = (
        f'/{truck_name}/bucket_action_status'
    )

    # ---------------------------------------------------------
    # 1. Motor drive
    #
    # Input:
    #   /<truck_name>/cmd_vel
    #
    # Output:
    #   /<truck_name>/wheel_states
    # ---------------------------------------------------------

    motor_drive_node = Node(
        package='dump_truck_hardware',
        executable='motor_drive_node',
        output='screen',
        parameters=[
            {
                'cmd_vel_topic':
                    cmd_vel_topic,

                'wheel_states_topic':
                    wheel_states_topic,
            }
        ],
    )

    # ---------------------------------------------------------
    # 2. Bucket action
    #
    # Input:
    #   /<truck_name>/bucket_action_cmd
    #
    # Output:
    #   /<truck_name>/bucket_action_status
    #
    # Truck-specific servo calibration:
    #
    #   config/hardware/<truck_name>.yaml
    #
    # ---------------------------------------------------------

    bucket_action_node = Node(
        package='dump_truck_hardware',
        executable='bucket_action_node',
        output='screen',
        parameters=[
            truck_hardware_yaml,
            {
                'bucket_action_cmd_topic':
                    bucket_action_cmd_topic,

                'bucket_action_status_topic':
                    bucket_action_status_topic,
            },
        ],
    )

    return [
        motor_drive_node,
        bucket_action_node,
    ]


def generate_launch_description():

    truck_name_argument = DeclareLaunchArgument(
        'truck_name',
        description=(
            'Unique dump-truck name used for ROS topics '
            'and hardware configuration file. '
            'Example: dumptruck_03'
        ),
    )

    return LaunchDescription([
        truck_name_argument,

        OpaqueFunction(
            function=launch_setup,
        ),
    ])