"""
Final challenge stack:
ArUco detector + EKF localization + covariance ellipse + Bug0 navigation.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('mini_challenge7')

    ekf_params = os.path.join(pkg, 'config', 'ekf_params.yaml')
    aruco_map = os.path.join(pkg, 'config', 'aruco_map.yaml')
    nav_params = os.path.join(pkg, 'config', 'final_challenge_nav.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'final_challenge.rviz')
    use_rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open RViz with EKF, ArUco detections, and covariance ellipse.',
        ),
        Node(
            package='mini_challenge7',
            executable='aruco_detector_node',
            name='aruco_detector_node',
            output='screen',
            parameters=[aruco_map, {'use_sim_time': True}],
        ),
        Node(
            package='mini_challenge7',
            executable='ekf_aruco_localization_node',
            name='ekf_aruco_localization_node',
            output='screen',
            parameters=[ekf_params, aruco_map, {'use_sim_time': True}],
        ),
        Node(
            package='mini_challenge7',
            executable='covariance_visualizer_node',
            name='covariance_visualizer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='mini_challenge7',
            executable='tangent_bug_node',
            name='tangent_bug_node',
            output='screen',
            parameters=[nav_params, {'use_sim_time': True}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_rviz),
        ),
    ])
