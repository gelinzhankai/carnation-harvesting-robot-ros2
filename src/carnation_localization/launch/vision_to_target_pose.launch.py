from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = Path(get_package_share_directory("carnation_description"))
    perception_share = Path(get_package_share_directory("carnation_perception"))
    localization_share = Path(get_package_share_directory("carnation_localization"))
    planning_share = Path(get_package_share_directory("carnation_planning"))

    robot_model = description_share / "urdf" / "carnation_robot_v0.urdf"
    rviz_config = description_share / "config" / "display.rviz"
    image_source_config = perception_share / "config" / "image_source.yaml"
    yolo_config = perception_share / "config" / "yolo_detector.yaml"
    localization_config = localization_share / "config" / "pixel_to_world.yaml"
    planner_config = planning_share / "config" / "simple_target_follower.yaml"
    robot_description = {"robot_description": robot_model.read_text(encoding="utf-8")}

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_image_source", default_value="true"),
            DeclareLaunchArgument("source_type", default_value="image"),
            DeclareLaunchArgument("source_path", default_value=""),
            DeclareLaunchArgument("trigger_file_path", default_value=""),
            DeclareLaunchArgument("image_publish_rate_hz", default_value="2.0"),
            DeclareLaunchArgument("loop_images", default_value="false"),
            DeclareLaunchArgument("screen_region_width", default_value="1000"),
            DeclareLaunchArgument("screen_region_height", default_value="1000"),
            DeclareLaunchArgument("screen_region_right_margin", default_value="0"),
            DeclareLaunchArgument("screen_region_bottom_margin", default_value="0"),
            DeclareLaunchArgument("use_image_view", default_value="true"),
            DeclareLaunchArgument("image_view_topic", default_value="/carnation/detections/image_annotated"),
            DeclareLaunchArgument("use_planning", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("sequence_detections", default_value="true"),
            DeclareLaunchArgument("confidence_threshold", default_value="0.50"),
            DeclareLaunchArgument("yolo_image_size", default_value="640"),
            DeclareLaunchArgument(
                "runtime_config_path",
                default_value="/mnt/h/carnation_detection/ros_screen_region_config.json",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value="/root/carnation_harvest/models/carnation_yolov8m2_best.pt",
            ),
            TimerAction(
                period=6.0,
                actions=[
                    Node(
                        package="carnation_perception",
                        executable="image_source_node",
                        name="image_source_node",
                        condition=IfCondition(LaunchConfiguration("use_image_source")),
                        parameters=[
                            str(image_source_config),
                            {
                                "source_type": LaunchConfiguration("source_type"),
                                "source_path": LaunchConfiguration("source_path"),
                                "trigger_file_path": LaunchConfiguration("trigger_file_path"),
                                "publish_rate_hz": ParameterValue(
                                    LaunchConfiguration("image_publish_rate_hz"),
                                    value_type=float,
                                ),
                                "loop_images": ParameterValue(
                                    LaunchConfiguration("loop_images"),
                                    value_type=bool,
                                ),
                                "screen_region_width": ParameterValue(
                                    LaunchConfiguration("screen_region_width"),
                                    value_type=int,
                                ),
                                "screen_region_height": ParameterValue(
                                    LaunchConfiguration("screen_region_height"),
                                    value_type=int,
                                ),
                                "screen_region_right_margin": ParameterValue(
                                    LaunchConfiguration("screen_region_right_margin"),
                                    value_type=int,
                                ),
                                "screen_region_bottom_margin": ParameterValue(
                                    LaunchConfiguration("screen_region_bottom_margin"),
                                    value_type=int,
                                ),
                            },
                        ],
                        output="log",
                    )
                ],
            ),
            Node(
                package="carnation_perception",
                executable="yolo_detector_node",
                name="yolo_detector_node",
                parameters=[
                    str(yolo_config),
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "sequence_detections": ParameterValue(
                            LaunchConfiguration("sequence_detections"),
                            value_type=bool,
                        ),
                        "confidence_threshold": ParameterValue(
                            LaunchConfiguration("confidence_threshold"),
                            value_type=float,
                        ),
                        "image_size": ParameterValue(
                            LaunchConfiguration("yolo_image_size"),
                            value_type=int,
                        ),
                        "runtime_config_path": LaunchConfiguration("runtime_config_path"),
                    },
                ],
                output="log",
            ),
            Node(
                package="carnation_localization",
                executable="pixel_to_world_node",
                name="pixel_to_world_node",
                parameters=[str(localization_config)],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                condition=IfCondition(LaunchConfiguration("use_planning")),
                parameters=[robot_description],
                output="log",
            ),
            Node(
                package="carnation_planning",
                executable="simple_target_follower",
                name="simple_target_follower",
                condition=IfCondition(LaunchConfiguration("use_planning")),
                parameters=[str(planner_config)],
                output="log",
            ),
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="rqt_image_view",
                        executable="rqt_image_view",
                        name="rqt_image_view",
                        arguments=[LaunchConfiguration("image_view_topic")],
                        condition=IfCondition(LaunchConfiguration("use_image_view")),
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        arguments=["-d", str(rviz_config)],
                        condition=IfCondition(LaunchConfiguration("use_rviz")),
                        output="screen",
                    )
                ],
            ),
        ]
    )
