from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("carnation_description"))
    default_model = package_share / "urdf" / "carnation_robot_v0.urdf"
    default_rviz_config = package_share / "config" / "display.rviz"

    robot_description = {"robot_description": default_model.read_text(encoding="utf-8")}

    use_joint_state_gui = LaunchConfiguration("use_joint_state_gui")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_joint_state_gui",
                default_value="true",
                description="Start joint_state_publisher_gui instead of the CLI publisher.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the default robot model config.",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                arguments=[str(default_model)],
                condition=IfCondition(use_joint_state_gui),
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                arguments=[str(default_model)],
                condition=UnlessCondition(use_joint_state_gui),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(default_rviz_config)],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
