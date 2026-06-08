#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_pkg = get_package_share_directory('puzzlebot_description')
    pallet_pkg = get_package_share_directory('puzzlebot_pallet_loader')
    start_odometry = LaunchConfiguration('start_odometry')
    start_mcl = LaunchConfiguration('start_mcl')
    global_localization = LaunchConfiguration('global_localization')

    urdf_path = os.path.join(
        description_pkg,
        'urdf',
        'mcr2_robots',
        'puzzlebot_jetson_lidar_ed.xacro',
    )
    rviz_config = os.path.join(
        pallet_pkg,
        'rviz',
        'mcl_localization_debug.rviz',
    )

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    odometry = Node(
        package='puzzlebot_pallet_loader',
        executable='odometry_node',
        name='odometry_node',
        output='screen',
        condition=IfCondition(start_odometry),
        parameters=[{
            'odom_topic': '/wheel_odom',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'laser_frame': 'laser',
            'laser_parent_frame': 'base_link',
            'activate_Kalman': False,
        }],
    )

    mcl = Node(
        package='puzzlebot_pallet_loader',
        executable='mcl_localization_node',
        name='mcl_localization_node',
        output='screen',
        condition=IfCondition(start_mcl),
        parameters=[{
            'odom_topic': '/wheel_odom',
            'scan_topic': '/scan',
            'mcl_odom_topic': '/mcl_odom',
            'particles_points_topic': '/mcl/particles_points',
            'global_localization_on_start': global_localization,
            'publish_tf': True,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mcl_debug',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_odometry',
            default_value='true',
            description='Start odometry_node for odom -> base_link TF.',
        ),
        DeclareLaunchArgument(
            'start_mcl',
            default_value='true',
            description='Start mcl_localization_node for /map and particles.',
        ),
        DeclareLaunchArgument(
            'global_localization',
            default_value='true',
            description='Spread particles over the whole map instead of starting near odometry.',
        ),
        robot_state_publisher,
        joint_state_publisher,
        odometry,
        mcl,
        rviz,
    ])
