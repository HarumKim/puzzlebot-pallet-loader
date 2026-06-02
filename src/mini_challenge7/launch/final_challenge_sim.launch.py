"""
Gazebo setup for the final challenge world with ArUcos and obstacles.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('puzzlebot_gazebo'),
                'launch',
                'gazebo_world_launch.py',
            )
        ),
        launch_arguments={
            'world': 'mini_challenge7_avoidance_1.world',
            'pause': 'false',
            'verbosity': '4',
        }.items(),
    )

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('puzzlebot_gazebo'),
                'launch',
                'gazebo_puzzlebot_launch.py',
            )
        ),
        launch_arguments={
            'robot': 'puzzlebot_jetson_lidar_ed',
            'robot_name': '',
            'x': '1.45',
            'y': '3.30',
            'yaw': '3.14159',
            'prefix': '',
            'lidar_frame': 'laser_frame',
            'camera_frame': 'camera_link_optical',
            'tof_frame': 'tof_link',
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([
        gazebo_launch,
        robot_launch,
    ])
