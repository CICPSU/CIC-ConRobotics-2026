from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def as_bool(value):
    return str(value).strip().lower() in (
        '1',
        'true',
        'yes',
        'on',
    )


def launch_setup(context, *args, **kwargs):

    # ---------------------------------------------------------
    # Resolve launch arguments
    # ---------------------------------------------------------

    trucks_raw = LaunchConfiguration(
        'trucks'
    ).perform(context)

    trucks = [
        item.strip()
        for item in trucks_raw.split(',')
        if item.strip()
    ]

    start_camera = as_bool(
        LaunchConfiguration(
            'start_camera'
        ).perform(context)
    )

    start_apriltag = as_bool(
        LaunchConfiguration(
            'start_apriltag'
        ).perform(context)
    )

    start_localization = as_bool(
        LaunchConfiguration(
            'start_localization'
        ).perform(context)
    )

    start_action_servers = as_bool(
        LaunchConfiguration(
            'start_action_servers'
        ).perform(context)
    )

    start_scenario_manager = as_bool(
        LaunchConfiguration(
            'start_scenario_manager'
        ).perform(context)
    )

    scenario = LaunchConfiguration(
        'scenario'
    ).perform(context)

    # ---------------------------------------------------------
    # Package paths
    # ---------------------------------------------------------

    perception_share = get_package_share_directory(
        'construction_robot_perception'
    )

    dump_truck_bringup_share = get_package_share_directory(
        'dump_truck_bringup'
    )

    # ---------------------------------------------------------
    # Actions to launch
    # ---------------------------------------------------------

    actions = []

    # =========================================================
    # 1. Overhead camera / AprilTag detector
    #
    # overhead_camera.launch.py is responsible for BOTH:
    #
    #   - USB camera
    #   - AprilTag detector
    #
    # The two components can be enabled independently using
    # launch arguments.
    # =========================================================

    if start_camera or start_apriltag:

        perception_launch = os.path.join(
            perception_share,
            'launch',
            'overhead_camera.launch.py',
        )

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    perception_launch
                ),
                launch_arguments={
                    'start_camera':
                        str(start_camera).lower(),
                    'start_apriltag':
                        str(start_apriltag).lower(),
                }.items(),
            )
        )

    # =========================================================
    # 2. Truck localization / odometry stack
    # =========================================================

    if start_localization:

        dump_truck_ros_pc_launch = os.path.join(
            dump_truck_bringup_share,
            'launch',
            'dump_truck_ros_pc.launch.py',
        )

        for truck_name in trucks:

            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        dump_truck_ros_pc_launch
                    ),
                    launch_arguments={
                        'truck_name':
                            truck_name,
                    }.items(),
                )
            )

    # =========================================================
    # 3. Truck Action Servers
    # =========================================================

    if start_action_servers:

        for truck_name in trucks:

            actions.append(
                Node(
                    package='dump_truck_action_server',
                    executable='waypoint_action_server_node',
                    namespace=truck_name,
                    name='waypoint_action_server',
                    output='screen',
                    parameters=[
                        {
                            'truck_name':
                                truck_name,
                        }
                    ],
                )
            )

    # =========================================================
    # 4. Scenario Manager
    # =========================================================

    if start_scenario_manager:

        actions.append(
            Node(
                package='construction_site_control',
                executable='scenario_manager_node',
                name='scenario_manager',
                output='screen',
                parameters=[
                    {
                        'scenario':
                            scenario,
                        'trucks':
                            trucks_raw,
                    }
                ],
            )
        )

    # ---------------------------------------------------------
    # Startup summary
    # ---------------------------------------------------------

    print()
    print('========================================')
    print('Construction Robotics Command Center')
    print('========================================')
    print(f'Trucks: {trucks}')
    print(f'Camera: {start_camera}')
    print(f'AprilTag: {start_apriltag}')
    print(f'Localization: {start_localization}')
    print(f'Action servers: {start_action_servers}')
    print(
        f'Scenario manager: '
        f'{start_scenario_manager}'
    )

    if start_scenario_manager:
        print(f'Scenario: {scenario}')

    print('========================================')
    print()

    return actions


def generate_launch_description():

    # ---------------------------------------------------------
    # Truck selection
    # ---------------------------------------------------------

    trucks_argument = DeclareLaunchArgument(
        'trucks',
        default_value='truck1',
        description=(
            'Comma-separated truck names. '
            'Examples: truck1 or truck1,truck3'
        ),
    )

    # ---------------------------------------------------------
    # Component switches
    # ---------------------------------------------------------

    start_camera_argument = DeclareLaunchArgument(
        'start_camera',
        default_value='true',
        description=(
            'Start the overhead camera.'
        ),
    )

    start_apriltag_argument = DeclareLaunchArgument(
        'start_apriltag',
        default_value='true',
        description=(
            'Start the AprilTag detector.'
        ),
    )

    start_localization_argument = DeclareLaunchArgument(
        'start_localization',
        default_value='true',
        description=(
            'Start odometry and AprilTag fusion '
            'for each selected truck.'
        ),
    )

    start_action_servers_argument = DeclareLaunchArgument(
        'start_action_servers',
        default_value='true',
        description=(
            'Start one waypoint Action Server '
            'for each selected truck.'
        ),
    )

    start_scenario_manager_argument = DeclareLaunchArgument(
        'start_scenario_manager',
        default_value='false',
        description=(
            'Start the construction-site '
            'scenario manager.'
        ),
    )

    # ---------------------------------------------------------
    # Scenario
    # ---------------------------------------------------------

    scenario_argument = DeclareLaunchArgument(
        'scenario',
        default_value='truck1_then_truck3.yaml',
        description=(
            'Scenario YAML file used by '
            'the scenario manager.'
        ),
    )

    return LaunchDescription([
        trucks_argument,

        start_camera_argument,
        start_apriltag_argument,
        start_localization_argument,
        start_action_servers_argument,
        start_scenario_manager_argument,

        scenario_argument,

        OpaqueFunction(
            function=launch_setup,
        ),
    ])