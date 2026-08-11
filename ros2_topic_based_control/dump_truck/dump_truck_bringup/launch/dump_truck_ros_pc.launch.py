from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
import yaml


def load_node_parameters(yaml_path, node_key):
    """
    Load ros__parameters for one node from the truck-specific YAML.

    Example YAML:

    tag_odom_fusion_landmarks_node:
      ros__parameters:
        robot_tag_child_frame: tag36h11_0
        tag_yaw_offset: 1.57079632679
    """

    if not os.path.isfile(yaml_path):
        raise RuntimeError(
            f'Truck configuration file not found: {yaml_path}'
        )

    with open(yaml_path, 'r') as file:
        data = yaml.safe_load(file) or {}

    node_section = data.get(node_key, {})
    parameters = node_section.get('ros__parameters', {})

    if not parameters:
        raise RuntimeError(
            f'No ros__parameters found for "{node_key}" '
            f'in {yaml_path}'
        )

    return parameters


def launch_setup(context, *args, **kwargs):

    # ---------------------------------------------------------
    # Truck name
    #
    # Examples:
    #   truck1
    #   truck3
    #   truck4
    # ---------------------------------------------------------

    truck_name = LaunchConfiguration(
        'truck_name'
    ).perform(context)

    # ---------------------------------------------------------
    # Package paths
    # ---------------------------------------------------------

    bringup_share = get_package_share_directory(
        'dump_truck_bringup'
    )

    landmarks_yaml = os.path.join(
        bringup_share,
        'config',
        'localization',
        'landmarks.yaml',
    )

    truck_hardware_yaml = os.path.join(
        bringup_share,
        'config',
        'hardware',
        f'{truck_name}.yaml',
    )

    # ---------------------------------------------------------
    # Load truck-specific localization parameters
    #
    # We load them here as a Python dictionary instead of
    # passing the YAML file directly to the node.
    #
    # This avoids parameter-file matching problems once the
    # node is placed inside /truckX namespace.
    # ---------------------------------------------------------

    fusion_truck_parameters = load_node_parameters(
        truck_hardware_yaml,
        'tag_odom_fusion_landmarks_node',
    )

    # ---------------------------------------------------------
    # Truck-specific topics
    # ---------------------------------------------------------

    wheel_states_topic = (
        f'/{truck_name}/wheel_states'
    )

    odom_topic = (
        f'/{truck_name}/odom'
    )

    fused_odom_topic = (
        f'/{truck_name}/fused_odom'
    )

    # ---------------------------------------------------------
    # Truck-specific TF frames
    # ---------------------------------------------------------

    odom_frame = (
        f'{truck_name}/odom'
    )

    base_frame = (
        f'{truck_name}/base_link'
    )

    fused_child_frame = (
        f'{truck_name}/base_link_fused'
    )

    # ---------------------------------------------------------
    # 1. Wheel-state odometry
    #
    # Node:
    #   /truckX/odometry
    #
    # Input:
    #   /truckX/wheel_states
    #
    # Output:
    #   /truckX/odom
    # ---------------------------------------------------------

    odometry_node = Node(
        package='dump_truck_control',
        executable='odometry_node',

        namespace=truck_name,
        name='odometry',

        output='screen',

        parameters=[
            {
                'wheel_states_topic':
                    wheel_states_topic,

                'odom_topic':
                    odom_topic,

                'odom_frame':
                    odom_frame,

                'base_frame':
                    base_frame,

                'publish_tf':
                    False,
            }
        ],
    )

    # ---------------------------------------------------------
    # 2. AprilTag + odometry fusion
    #
    # Node:
    #   /truckX/tag_odom_fusion
    #
    # Truck-specific values are loaded from:
    #
    #   config/hardware/truckX.yaml
    #
    # Common camera/environment values remain here.
    # ---------------------------------------------------------

    tag_odom_fusion_node = Node(
        package='dump_truck_control',
        executable='tag_odom_fusion_node',

        namespace=truck_name,
        name='tag_odom_fusion',

        output='screen',

        parameters=[
            # Truck-specific values
            fusion_truck_parameters,

            # Common environment/fusion values
            {
                'alpha':
                    0.03,

                'tag_parent_frame':
                    'default_cam',

                'camera_y_scale':
                    -1.0,

                'camera_yaw_scale':
                    -1.0,

                'initial_yaw_source':
                    'tag',

                'use_initial_yaw_alignment':
                    True,

                'landmarks_yaml':
                    landmarks_yaml,
            },

            # Dynamically generated Truck-specific ROS values
            {
                'odom_topic':
                    odom_topic,

                'fused_odom_topic':
                    fused_odom_topic,

                'fused_child_frame':
                    fused_child_frame,
            },
        ],
    )

    # ---------------------------------------------------------
    # Startup sequence
    #
    # 0 sec:
    #   /truckX/odometry
    #
    # 1 sec:
    #   /truckX/tag_odom_fusion
    # ---------------------------------------------------------

    delayed_fusion = TimerAction(
        period=1.0,
        actions=[
            tag_odom_fusion_node,
        ],
    )

    return [
        odometry_node,
        delayed_fusion,
    ]


def generate_launch_description():

    truck_name_argument = DeclareLaunchArgument(
        'truck_name',
        description=(
            'Unique dump truck name. '
            'Examples: truck1, truck3, truck4'
        ),
    )

    return LaunchDescription([
        truck_name_argument,

        OpaqueFunction(
            function=launch_setup,
        ),
    ])