import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory(
        "excavator_control"
    )

    default_config = os.path.join(
        package_share,
        "config",
        "excavator1.yaml",
    )

    mode = LaunchConfiguration("mode")
    config = LaunchConfiguration("config")
    robot_name = LaunchConfiguration("robot_name")

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
                "config",
                default_value=default_config,
                description=(
                    "Path to the excavator "
                    "configuration YAML file."
                ),
            ),

            DeclareLaunchArgument(
                "robot_name",
                default_value="excavator1",
                description=(
                    "Robot name used as the ROS 2 namespace. "
                    "Examples: excavator1, excavator3."
                ),
            ),

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
    )