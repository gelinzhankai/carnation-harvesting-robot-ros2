from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = Path(get_package_share_directory("carnation_description"))
    planning_share = Path(get_package_share_directory("carnation_planning"))

    default_model = description_share / "urdf" / "carnation_robot_v0.urdf"
    default_rviz_config = description_share / "config" / "display.rviz"
    default_planner_config = planning_share / "config" / "simple_target_follower.yaml"

    robot_description = {"robot_description": default_model.read_text(encoding="utf-8")}
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the default robot model config.",
            ),
            Node(
                package="carnation_planning",
                executable="simple_target_follower",
                name="simple_target_follower",
                parameters=[str(default_planner_config)],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        arguments=["-d", str(default_rviz_config)],
                        condition=IfCondition(use_rviz),
                        output="screen",
                    )
                ],
            ),
        ]
    )
