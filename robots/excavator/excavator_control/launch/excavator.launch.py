import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    package_share = get_package_share_directory(
        "excavator_control"
    )

    mode = LaunchConfiguration("mode").perform(context)
    robot_name = LaunchConfiguration(
        "robot_name"
    ).perform(context)
    config = LaunchConfiguration("config").perform(context)

    # If no config file is explicitly provided, automatically use
    # config/<robot_name>.yaml.
    if not config:
        config = os.path.join(
            package_share,
            "config",
            f"{robot_name}.yaml",
        )

    return [
        Node(
            package="excavator_control",
            executable="excavator_trajectory_server",
            name="excavator_trajectory_server",
            namespace=robot_name,
            output="screen",
            arguments=[
                "--mode",
                mode,
                "--config",
                config,
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="sim",
                description=(
                    "Excavator execution mode: "
                    "'sim' or 'pi'."
                ),
            ),

            DeclareLaunchArgument(
                "robot_name",
                default_value="excavator1",
                description=(
                    "Robot name used as the ROS 2 namespace "
                    "and default configuration name. "
                    "Examples: excavator1, excavator3."
                ),
            ),

            DeclareLaunchArgument(
                "config",
                default_value="",
                description=(
                    "Optional path to an excavator configuration "
                    "YAML file. If omitted, config/<robot_name>.yaml "
                    "is selected automatically."
                ),
            ),

            OpaqueFunction(
                function=launch_setup,
            ),
        ]
    )