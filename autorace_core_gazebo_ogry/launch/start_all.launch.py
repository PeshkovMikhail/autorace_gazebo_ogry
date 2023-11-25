import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    robot_bringup_path = get_package_share_directory("robot_bringup")
    core_path = get_package_share_directory("autorace_core_gazebo_ogry")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(robot_bringup_path, "launch", "autorace_2023.launch.py"),
                )
            ),
            Node(
                package="autorace_camera",
                executable="core_node_mission"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(core_path, "launch", "autorace_core.launch"),
                )
            ),
        ]
    )
