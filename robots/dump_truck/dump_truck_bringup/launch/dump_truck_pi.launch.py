from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory,
)

import os
import yaml


def load_node_parameters(
    yaml_path,
    node_key,
):
    """
    Load ros__parameters for one node from the
    truck-specific hardware YAML.

    Example:

    motor_drive_node:
      ros__parameters:
        encoder_direction_mode: commanded
        encoder_glitch_filter_us: 200
        left_encoder_invert: false
        right_encoder_invert: false
        swap_encoders: true

    bucket_action_node:
      ros__parameters:
        servo_center: 1095
        servo_dump: 1500
    """

    if not os.path.isfile(
        yaml_path
    ):
        raise RuntimeError(
            'Truck configuration file '
            f'not found: {yaml_path}'
        )

    with open(
        yaml_path,
        'r',
        encoding='utf-8',
    ) as file:

        data = (
            yaml.safe_load(file)
            or {}
        )

    node_section = (
        data.get(
            node_key,
            {},
        )
    )

    parameters = (
        node_section.get(
            'ros__parameters',
            {},
        )
    )

    if not parameters:
        raise RuntimeError(
            'No ros__parameters found for '
            f'"{node_key}" in {yaml_path}'
        )

    return parameters


def launch_setup(
    context,
    *args,
    **kwargs,
):

    # =========================================================
    # Truck name
    #
    # Examples:
    #
    #   truck1
    #   truck3
    #   truck4
    #   truck5
    # =========================================================

    truck_name = (
        LaunchConfiguration(
            'truck_name'
        ).perform(
            context
        )
    )

    # =========================================================
    # Package paths
    # =========================================================

    bringup_share = (
        get_package_share_directory(
            'dump_truck_bringup'
        )
    )

    truck_hardware_yaml = (
        os.path.join(
            bringup_share,
            'config',
            'hardware',
            f'{truck_name}.yaml',
        )
    )

    # =========================================================
    # Load truck-specific hardware parameters
    # =========================================================

    motor_parameters = (
        load_node_parameters(
            truck_hardware_yaml,
            'motor_drive_node',
        )
    )

    bucket_parameters = (
        load_node_parameters(
            truck_hardware_yaml,
            'bucket_action_node',
        )
    )

    # =========================================================
    # Truck-specific topics
    # =========================================================

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

    # =========================================================
    # 1. Motor drive
    #
    # Node:
    #
    #   /truckX/motor_drive
    #
    # Input:
    #
    #   /truckX/cmd_vel
    #
    # Output:
    #
    #   /truckX/wheel_states
    #
    # Truck-specific hardware behavior comes from:
    #
    #   config/hardware/truckX.yaml
    #
    # Example parameters:
    #
    #   encoder_direction_mode
    #   encoder_glitch_filter_us
    #   left_encoder_invert
    #   right_encoder_invert
    #   swap_encoders
    # =========================================================

    motor_drive_node = Node(
        package='dump_truck_hardware',
        executable='motor_drive_node',

        namespace=truck_name,
        name='motor_drive',

        output='screen',

        parameters=[
            # Truck-specific hardware calibration
            motor_parameters,

            # Truck-specific ROS topics
            {
                'cmd_vel_topic':
                    cmd_vel_topic,

                'wheel_states_topic':
                    wheel_states_topic,
            },
        ],
    )

    # =========================================================
    # 2. Bucket action
    #
    # Node:
    #
    #   /truckX/bucket_action
    #
    # Input:
    #
    #   /truckX/bucket_action_cmd
    #
    # Output:
    #
    #   /truckX/bucket_action_status
    # =========================================================

    bucket_action_node = Node(
        package='dump_truck_hardware',
        executable='bucket_action_node',

        namespace=truck_name,
        name='bucket_action',

        output='screen',

        parameters=[
            # Truck-specific bucket calibration
            bucket_parameters,

            # Truck-specific ROS topics
            {
                'bucket_action_cmd_topic':
                    bucket_action_cmd_topic,

                'bucket_action_status_topic':
                    bucket_action_status_topic,
            },
        ],
    )

    # =========================================================
    # Launch nodes
    # =========================================================

    return [
        motor_drive_node,
        bucket_action_node,
    ]


def generate_launch_description():

    # =========================================================
    # Truck name argument
    # =========================================================

    truck_name_argument = (
        DeclareLaunchArgument(
            'truck_name',

            description=(
                'Unique dump truck name. '
                'Examples: '
                'truck1, truck3, truck4, truck5'
            ),
        )
    )

    # =========================================================
    # Launch description
    # =========================================================

    return LaunchDescription([
        truck_name_argument,

        OpaqueFunction(
            function=launch_setup,
        ),
    ])