from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    localization_share = Path(get_package_share_directory("carnation_localization"))
    default_config = localization_share / "config" / "pixel_to_world.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument("fixed_depth_m", default_value="1.0"),
            Node(
                package="carnation_localization",
                executable="pixel_to_world_node",
                name="pixel_to_world_node",
                parameters=[
                    str(default_config),
                    {"fixed_depth_m": LaunchConfiguration("fixed_depth_m")},
                ],
                output="screen",
            ),
        ]
    )
