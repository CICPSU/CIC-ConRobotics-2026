from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    bringup_share = FindPackageShare(
        'dump_truck_bringup'
    )

    landmarks_yaml = PathJoinSubstitution([
        bringup_share,
        'config',
        'localization',
        'landmarks.yaml',
    ])

    # ---------------------------------------------------------
    # 1. Wheel-state odometry
    # ---------------------------------------------------------

    odometry_node = Node(
        package='dump_truck_control',
        executable='odometry_node',
        output='screen',
        parameters=[
            {
                'wheel_states_topic':
                    '/truck1/wheel_states',

                'odom_topic':
                    '/truck1/odom',

                'odom_frame':
                    'truck1/odom',

                'base_frame':
                    'truck1/base_link',

                'publish_tf':
                    False,
            }
        ],
    )

    # ---------------------------------------------------------
    # 2. AprilTag + odometry fusion
    # ---------------------------------------------------------

    tag_odom_fusion_node = Node(
        package='dump_truck_control',
        executable='tag_odom_fusion_node',
        output='screen',
        parameters=[
            {
                'odom_topic':
                    '/truck1/odom',

                'fused_odom_topic':
                    '/truck1/fused_odom',

                'robot_tag_child_frame':
                    'tag36h11_0',

                'fused_child_frame':
                    'truck1/base_link_fused',

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

                'odom_x_scale':
                    1.0,

                'odom_y_scale':
                    1.0,

                'odom_yaw_scale':
                    1.0,

                'tag_yaw_offset':
                    1.57079632679,

                'landmarks_yaml':
                    landmarks_yaml,
            }
        ],
    )

    # ---------------------------------------------------------
    # Preserve old startup timing:
    #
    #   0 sec -> odometry
    #   1 sec -> fusion
    #
    # Waypoint controller is now started separately with:
    #
    # ros2 run dump_truck_control waypoint_controller_node \
    #   truck1 <route.yaml>
    # ---------------------------------------------------------

    delayed_fusion = TimerAction(
        period=1.0,
        actions=[
            tag_odom_fusion_node,
        ],
    )

    return LaunchDescription([
        odometry_node,
        delayed_fusion,
    ])