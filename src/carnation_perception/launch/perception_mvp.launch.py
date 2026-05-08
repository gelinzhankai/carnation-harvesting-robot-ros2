from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perception_share = Path(get_package_share_directory("carnation_perception"))

    default_image_source_config = perception_share / "config" / "image_source.yaml"
    default_yolo_config = perception_share / "config" / "yolo_detector.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_image_source", default_value="true"),
            DeclareLaunchArgument("source_type", default_value="image"),
            DeclareLaunchArgument("source_path", default_value=""),
            DeclareLaunchArgument(
                "model_path",
                default_value="/root/carnation_harvest/models/carnation_yolov8m2_best.pt",
            ),
            Node(
                package="carnation_perception",
                executable="image_source_node",
                name="image_source_node",
                condition=IfCondition(LaunchConfiguration("use_image_source")),
                parameters=[
                    str(default_image_source_config),
                    {
                        "source_type": LaunchConfiguration("source_type"),
                        "source_path": LaunchConfiguration("source_path"),
                    },
                ],
                output="screen",
            ),
            Node(
                package="carnation_perception",
                executable="yolo_detector_node",
                name="yolo_detector_node",
                parameters=[
                    str(default_yolo_config),
                    {
                        "model_path": LaunchConfiguration("model_path"),
                    },
                ],
                output="screen",
            ),
        ]
    )
