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


def launch_setup(context, *args, **kwargs):
    # ---------------------------------------------------------
    # Resolve excavator selection
    # ---------------------------------------------------------

    excavators_raw = LaunchConfiguration(
        "excavators"
    ).perform(context)

    excavators = [
        item.strip()
        for item in excavators_raw.split(",")
        if item.strip()
    ]

    # ---------------------------------------------------------
    # Shared configuration
    # ---------------------------------------------------------

    perception_share = get_package_share_directory(
        "construction_robot_perception"
    )

    swing_adapters_config = os.path.join(
        perception_share,
        "config",
        "swing_position_adapters.yaml",
    )

    # ---------------------------------------------------------
    # Swing position adapters
    # ---------------------------------------------------------

    actions = []

    for excavator_name in excavators:
        node_name = (
            "swing_position_adapter_"
            f"{excavator_name}"
        )

        actions.append(
            Node(
                package="construction_robot_perception",
                executable="swing_position_adapter",
                name=node_name,
                output="screen",
                parameters=[
                    swing_adapters_config,
                ],
            )
        )

    # ---------------------------------------------------------
    # Startup summary
    # ---------------------------------------------------------

    print()
    print("========================================")
    print("Excavator Perception")
    print("========================================")
    print(f"Excavators: {excavators}")
    print("Swing position adapters:")

    for excavator_name in excavators:
        print(
            "  - swing_position_adapter_"
            f"{excavator_name}"
        )

    print("========================================")
    print()

    return actions


def generate_launch_description():
    excavators_argument = DeclareLaunchArgument(
        "excavators",
        default_value="excavator3",
        description=(
            "Comma-separated excavator names. "
            "Examples: excavator3 or "
            "excavator1,excavator3,excavator5"
        ),
    )

    return LaunchDescription(
        [
            excavators_argument,
            OpaqueFunction(
                function=launch_setup,
            ),
        ]
    )