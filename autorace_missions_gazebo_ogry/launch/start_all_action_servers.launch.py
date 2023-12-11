from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autorace_core_gazebo_ogry',
            executable='intersection_action_server',
        ),
        Node(
            package='autorace_missions_gazebo_ogry',
            executable='construction',
        ),
        Node(
            package='autorace_core_gazebo_ogry',
            executable='parking_action_server',
        ),
        Node(
            package='autorace_missions_gazebo_ogry',
            executable='crossing',
        ),
        Node(
            package='autorace_missions_gazebo_ogry',
            executable='tunnel',
        )
    ])