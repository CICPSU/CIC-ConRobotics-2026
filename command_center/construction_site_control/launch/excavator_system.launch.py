import os

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    perception_share = get_package_share_directory(
        "construction_robot_perception"
    )

    overhead_camera_launch = os.path.join(
        perception_share,
        "launch",
        "overhead_camera.launch.py",
    )

    excavator_perception_launch = os.path.join(
        perception_share,
        "launch",
        "excavator_perception.launch.py",
    )

    excavators_argument = DeclareLaunchArgument(
        "excavators",
        default_value="excavator3",
        description=(
            "Comma-separated excavator names. "
            "Examples: excavator3 or "
            "excavator1,excavator3"
        ),
    )

    start_camera_argument = DeclareLaunchArgument(
        "start_camera",
        default_value="true",
        description="Start the overhead camera.",
    )

    start_apriltag_argument = DeclareLaunchArgument(
        "start_apriltag",
        default_value="true",
        description="Start the AprilTag detector.",
    )

    shared_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            overhead_camera_launch
        ),
        launch_arguments={
            "start_camera": LaunchConfiguration(
                "start_camera"
            ),
            "start_apriltag": LaunchConfiguration(
                "start_apriltag"
            ),
        }.items(),
    )

    excavator_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            excavator_perception_launch
        ),
        launch_arguments={
            "excavators": LaunchConfiguration(
                "excavators"
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            excavators_argument,
            start_camera_argument,
            start_apriltag_argument,
            shared_perception,
            excavator_perception,
        ]
    )