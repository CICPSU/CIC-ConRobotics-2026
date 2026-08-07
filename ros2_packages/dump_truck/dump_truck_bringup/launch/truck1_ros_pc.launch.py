from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ---------------------------------------------------------
    # Package paths
    # ---------------------------------------------------------

    bringup_share = FindPackageShare('dump_truck_bringup')

    default_landmarks_yaml = PathJoinSubstitution([
        bringup_share,
        'config',
        'localization',
        'landmarks.yaml',
    ])

    default_waypoints_yaml = PathJoinSubstitution([
        bringup_share,
        'config',
        'waypoints',
        'truck1_waypoints.yaml',
    ])

    # ---------------------------------------------------------
    # Launch arguments
    #
    # These preserve the old truck1_run.py behavior where
    # waypoint YAML could be replaced from the command line.
    # ---------------------------------------------------------

    landmarks_yaml_arg = DeclareLaunchArgument(
        'landmarks_yaml',
        default_value=default_landmarks_yaml,
        description='Landmark YAML file used by Tag/Odom fusion',
    )

    waypoints_yaml_arg = DeclareLaunchArgument(
        'waypoints_yaml',
        default_value=default_waypoints_yaml,
        description='Waypoint YAML file used by Truck 1',
    )

    landmarks_yaml = LaunchConfiguration('landmarks_yaml')
    waypoints_yaml = LaunchConfiguration('waypoints_yaml')

    # ---------------------------------------------------------
    # 1. Wheel-state odometry
    #
    # Equivalent to:
    # python3 Odom_test_multi.py --ros-args ...
    # ---------------------------------------------------------

    odometry_node = Node(
        package='dump_truck_control',
        executable='odometry_node',
        output='screen',
        parameters=[
            {
                'wheel_states_topic': '/truck1/wheel_states',
                'odom_topic': '/truck1/odom',
                'odom_frame': 'truck1/odom',
                'base_frame': 'truck1/base_link',
                'publish_tf': False,
            }
        ],
    )

    # ---------------------------------------------------------
    # 2. AprilTag + odometry fusion
    #
    # Equivalent to:
    # python3 tag_odom_fusion_node_landmarks_v4.py --ros-args ...
    # ---------------------------------------------------------

    tag_odom_fusion_node = Node(
        package='dump_truck_control',
        executable='tag_odom_fusion_node',
        output='screen',
        parameters=[
            {
                'odom_topic': '/truck1/odom',
                'fused_odom_topic': '/truck1/fused_odom',

                'robot_tag_child_frame': 'tag36h11_0',
                'fused_child_frame': 'truck1/base_link_fused',

                'alpha': 0.03,

                'tag_parent_frame': 'default_cam',
                'camera_y_scale': -1.0,
                'camera_yaw_scale': -1.0,

                'initial_yaw_source': 'tag',
                'use_initial_yaw_alignment': True,

                'odom_x_scale': 1.0,
                'odom_y_scale': 1.0,
                'odom_yaw_scale': 1.0,

                'tag_yaw_offset': 1.57079632679,

                'landmarks_yaml': landmarks_yaml,
            }
        ],
    )

    # ---------------------------------------------------------
    # 3. Waypoint controller
    #
    # Equivalent to:
    # python3 ppwyr_fused_multi_v2.py --ros-args ...
    # ---------------------------------------------------------

    waypoint_controller_node = Node(
        package='dump_truck_control',
        executable='waypoint_controller_node',
        output='screen',
        parameters=[
            {
                'odom_topic': '/truck1/fused_odom',
                'cmd_vel_topic': '/truck1/cmd_vel',

                'bucket_action_cmd_topic':
                    '/truck1/bucket_action_cmd',

                'bucket_action_status_topic':
                    '/truck1/bucket_action_status',

                'waypoints_yaml': waypoints_yaml,
            }
        ],
    )

    # ---------------------------------------------------------
    # Startup order
    #
    # Preserve old truck1_run.py timing:
    #
    #   0 sec -> odometry
    #   1 sec -> fusion
    #   2 sec -> waypoint controller
    # ---------------------------------------------------------

    delayed_fusion = TimerAction(
        period=1.0,
        actions=[
            tag_odom_fusion_node,
        ],
    )

    delayed_waypoint_controller = TimerAction(
        period=2.0,
        actions=[
            waypoint_controller_node,
        ],
    )

    # ---------------------------------------------------------
    # Launch description
    # ---------------------------------------------------------

    return LaunchDescription([
        landmarks_yaml_arg,
        waypoints_yaml_arg,

        odometry_node,
        delayed_fusion,
        delayed_waypoint_controller,
    ])