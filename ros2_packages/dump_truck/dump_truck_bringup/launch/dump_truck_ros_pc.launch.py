from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
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
    # Truck-specific ROS topics
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
    # Truck-specific TF frame names
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
    # Input:
    #   /<truck_name>/wheel_states
    #
    # Output:
    #   /<truck_name>/odom
    #
    # TF:
    #   Disabled here.
    # ---------------------------------------------------------

    odometry_node = Node(
        package='dump_truck_control',
        executable='odometry_node',
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
    # Common parameters stay here.
    #
    # Truck-specific parameters such as:
    #
    #   robot_tag_child_frame
    #   tag_yaw_offset
    #   odom_x_scale
    #   odom_y_scale
    #   odom_yaw_scale
    #
    # are loaded from:
    #
    #   config/hardware/<truck_name>.yaml
    #
    # ---------------------------------------------------------

    tag_odom_fusion_node = Node(
        package='dump_truck_control',
        executable='tag_odom_fusion_node',
        output='screen',
        parameters=[
            truck_hardware_yaml,
            {
                'odom_topic':
                    odom_topic,

                'fused_odom_topic':
                    fused_odom_topic,

                'fused_child_frame':
                    fused_child_frame,

                # ---------------------------------------------
                # Common fusion settings
                # ---------------------------------------------

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

                # ---------------------------------------------
                # Shared landmark map
                # ---------------------------------------------

                'landmarks_yaml':
                    landmarks_yaml,
            },
        ],
    )

    # ---------------------------------------------------------
    # Preserve startup timing
    #
    #   0 sec -> odometry
    #   1 sec -> AprilTag / odometry fusion
    #
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

    # ---------------------------------------------------------
    # Required robot identifier
    #
    # Examples:
    #
    #   truck1
    #   dumptruck_03
    #
    # ---------------------------------------------------------

    truck_name_argument = DeclareLaunchArgument(
        'truck_name',
        description=(
            'Unique dump-truck name used for ROS topics, '
            'TF frames, and hardware configuration file. '
            'Example: dumptruck_03'
        ),
    )

    return LaunchDescription([
        truck_name_argument,

        OpaqueFunction(
            function=launch_setup,
        ),
    ])