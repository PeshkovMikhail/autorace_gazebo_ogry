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
    slam_pkg_path = get_package_share_directory("slam_toolbox")

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg_path, "launch", "online_async_launch.py"),
        ),
        launch_arguments={
            "use_sim_time": "true",
            "slam_params_file": os.path.join(
                robot_bringup_path, "config", "mapper.yaml"
            )
        }.items(),
    )

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(robot_bringup_path, 'config/mapper.yaml'), {'use_sim_time': True}]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(robot_bringup_path, "launch", "autorace_2023.launch.py"),
                )
            ),
            slam,
            robot_localization_node
        ]
    )
