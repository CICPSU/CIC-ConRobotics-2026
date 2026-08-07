from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    bringup_share = FindPackageShare('dump_truck_bringup')

    truck1_hardware_yaml = PathJoinSubstitution([
        bringup_share,
        'config',
        'hardware',
        'truck1.yaml',
    ])

    # ---------------------------------------------------------
    # Motor drive
    #
    # Input:
    #   /truck1/cmd_vel
    #
    # Output:
    #   /truck1/wheel_states
    # ---------------------------------------------------------

    motor_drive_node = Node(
        package='dump_truck_hardware',
        executable='motor_drive_node',
        output='screen',
        parameters=[
            {
                'cmd_vel_topic': '/truck1/cmd_vel',
                'wheel_states_topic': '/truck1/wheel_states',
            }
        ],
    )

    # ---------------------------------------------------------
    # Bucket action
    #
    # Input:
    #   /truck1/bucket_action_cmd
    #
    # Output:
    #   /truck1/bucket_action_status
    #
    # Servo calibration:
    #   config/hardware/truck1.yaml
    # ---------------------------------------------------------

    bucket_action_node = Node(
        package='dump_truck_hardware',
        executable='bucket_action_node',
        output='screen',
        parameters=[
            truck1_hardware_yaml,
            {
                'bucket_action_cmd_topic':
                    '/truck1/bucket_action_cmd',

                'bucket_action_status_topic':
                    '/truck1/bucket_action_status',
            },
        ],
    )

    return LaunchDescription([
        motor_drive_node,
        bucket_action_node,
    ])