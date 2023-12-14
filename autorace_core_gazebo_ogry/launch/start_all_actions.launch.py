from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="autorace_core_gazebo_ogry",
            executable='intersection_action_server'
        ),
        Node(
            package="autorace_core_gazebo_ogry",
            executable="construction_action_server"
        ),
        Node(
            package="autorace_core_gazebo_ogry",
            executable="parking_action_server"
        ),
        Node(
            package="autorace_core_gazebo_ogry",
            executable="crosswalk_action_server"
        ),
        Node(
            package="autorace_core_gazebo_ogry",
            executable="tunnel_action_server"
        )

    ])
    