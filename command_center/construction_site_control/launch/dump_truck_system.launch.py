import os

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    trucks_raw = LaunchConfiguration(
        "trucks"
    ).perform(context)

    trucks = [
        item.strip()
        for item in trucks_raw.split(",")
        if item.strip()
    ]

    perception_share = get_package_share_directory(
        "construction_robot_perception"
    )

    dump_truck_bringup_share = (
        get_package_share_directory(
            "dump_truck_bringup"
        )
    )

    overhead_camera_launch = os.path.join(
        perception_share,
        "launch",
        "overhead_camera.launch.py",
    )

    dump_truck_ros_pc_launch = os.path.join(
        dump_truck_bringup_share,
        "launch",
        "dump_truck_ros_pc.launch.py",
    )

    actions = []

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                overhead_camera_launch
            ),
            launch_arguments={
                "start_camera": "true",
                "start_apriltag": "true",
            }.items(),
        )
    )

    for truck_name in trucks:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    dump_truck_ros_pc_launch
                ),
                launch_arguments={
                    "truck_name": truck_name,
                }.items(),
            )
        )

        actions.append(
            Node(
                package="dump_truck_action_server",
                executable="waypoint_action_server_node",
                namespace=truck_name,
                name="waypoint_action_server",
                output="screen",
                parameters=[
                    {
                        "truck_name": truck_name,
                    }
                ],
            )
        )

    print()
    print("========================================")
    print("Dump Truck System")
    print("========================================")
    print(f"Trucks: {trucks}")
    print("Camera: True")
    print("AprilTag: True")
    print("Truck localization: True")
    print("Truck action servers: True")
    print("========================================")
    print()

    return actions


def generate_launch_description():
    trucks_argument = DeclareLaunchArgument(
        "trucks",
        default_value="truck1",
        description=(
            "Comma-separated truck names. "
            "Examples: truck1 or "
            "truck1,truck3,truck4"
        ),
    )

    return LaunchDescription(
        [
            trucks_argument,
            OpaqueFunction(
                function=launch_setup,
            ),
        ]
    )