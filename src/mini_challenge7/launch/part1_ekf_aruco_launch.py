"""
Part 1 Launch — EKF + ArUco Localization

Launches:
    - aruco_detector_node      (detects ArUco markers from camera)
    - ekf_aruco_localization_node (EKF prediction + ArUco correction)
    - covariance_visualizer_node  (95% confidence ellipse in RViz)

Usage:
    ros2 launch mini_challenge7 part1_ekf_aruco_launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('mini_challenge7')

    ekf_params = os.path.join(pkg, 'config', 'ekf_params.yaml')
    aruco_map = os.path.join(pkg, 'config', 'aruco_map.yaml')

    return LaunchDescription([

        # ArUco detector — camera → /aruco_measurements
        Node(
            package='mini_challenge7',
            executable='aruco_detector_node',
            name='aruco_detector_node',
            output='screen',
            parameters=[aruco_map],
        ),

        # EKF localization — encoders + ArUco → /ekf_odom
        Node(
            package='mini_challenge7',
            executable='ekf_aruco_localization_node',
            name='ekf_aruco_localization_node',
            output='screen',
            parameters=[ekf_params, aruco_map],
        ),

        # Covariance visualizer — /ekf_odom → /covariance_ellipse
        Node(
            package='mini_challenge7',
            executable='covariance_visualizer_node',
            name='covariance_visualizer_node',
            output='screen',
        ),
    ])
