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
    required=True,
):
    """
    Load ros__parameters for one node from the
    truck-specific YAML.

    If required=False and the node section does not exist,
    return an empty dictionary.

    This allows older truck YAML files to keep using the
    default parameters defined inside the ROS node.
    """

    if not os.path.isfile(
        yaml_path
    ):
        raise RuntimeError(
            f'Truck configuration file '
            f'not found: {yaml_path}'
        )

    with open(
        yaml_path,
        'r',
    ) as file:

        data = (
            yaml.safe_load(
                file
            )
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

    if (
        required
        and not parameters
    ):
        raise RuntimeError(
            'No ros__parameters found '
            f'for "{node_key}" '
            f'in {yaml_path}'
        )

    return parameters


def launch_setup(
    context,
    *args,
    **kwargs,
):

    # ---------------------------------------------------------
    # Truck name
    #
    # Examples:
    #   truck1
    #   truck3
    #   truck4
    #   truck5
    # ---------------------------------------------------------

    truck_name = (
        LaunchConfiguration(
            'truck_name'
        ).perform(
            context
        )
    )

    # ---------------------------------------------------------
    # Package paths
    # ---------------------------------------------------------

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

    # ---------------------------------------------------------
    # Load truck-specific motor parameters
    #
    # This section is optional.
    #
    # Truck 1 / 3 can omit it and continue using:
    #
    #   encoder_direction_mode = quadrature
    #   encoder_glitch_filter_us = 0
    #
    # Truck 4 / 5 can override them in YAML.
    # ---------------------------------------------------------

    motor_parameters = (
        load_node_parameters(
            truck_hardware_yaml,
            'motor_drive_node',
            required=False,
        )
    )

    # ---------------------------------------------------------
    # Load truck-specific bucket calibration
    # ---------------------------------------------------------

    bucket_parameters = (
        load_node_parameters(
            truck_hardware_yaml,
            'bucket_action_node',
            required=True,
        )
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
    # Node:
    #   /truckX/motor_drive
    #
    # Input:
    #   /truckX/cmd_vel
    #
    # Output:
    #   /truckX/wheel_states
    # ---------------------------------------------------------

    motor_drive_node = Node(
        package='dump_truck_hardware',
        executable='motor_drive_node',

        namespace=truck_name,
        name='motor_drive',

        output='screen',

        parameters=[
            motor_parameters,
            {
                'cmd_vel_topic':
                    cmd_vel_topic,

                'wheel_states_topic':
                    wheel_states_topic,
            },
        ],
    )

    # ---------------------------------------------------------
    # 2. Bucket action
    #
    # Node:
    #   /truckX/bucket_action
    #
    # Input:
    #   /truckX/bucket_action_cmd
    #
    # Output:
    #   /truckX/bucket_action_status
    # ---------------------------------------------------------

    bucket_action_node = Node(
        package='dump_truck_hardware',
        executable='bucket_action_node',

        namespace=truck_name,
        name='bucket_action',

        output='screen',

        parameters=[
            bucket_parameters,
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

    return LaunchDescription([
        truck_name_argument,

        OpaqueFunction(
            function=launch_setup,
        ),
    ])