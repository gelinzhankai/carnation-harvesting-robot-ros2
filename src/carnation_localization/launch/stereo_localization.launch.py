from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    localization_share = Path(get_package_share_directory("carnation_localization"))
    stereo_config = localization_share / "config" / "stereo_pixel_to_world.yaml"

    return LaunchDescription(
        [
            Node(
                package="carnation_localization",
                executable="stereo_pixel_to_world_node",
                name="stereo_pixel_to_world_node",
                parameters=[str(stereo_config)],
                output="screen",
            ),
        ]
    )
