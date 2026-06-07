#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg = get_package_share_directory('puzzlebot_pallet_loader')

    # Keep your current navigation params file.
    # Change this filename if your launch uses a different config.
    nav_params = os.path.join(pkg, 'config', 'real_robot_nav_params.yaml')

    # If you have a config for MCL later, you can replace the inline dictionary
    # with that yaml file. For now these values match the simulation MCL baseline.
    mcl_params = {
        'odom_topic': '/odom',
        'scan_topic': '/scan',
        'initial_pose_topic': '/initialpose',
        'mcl_pose_topic': '/mcl_pose',
        'mcl_odom_topic': '/mcl_odom',
        'particles_topic': '/mcl/particles',
        'map_topic': '/mcl/map',
        'publish_tf': True,
        'publish_map': True,
        'show_debug_window': False,
        'map_resolution': 0.05,
        'map_origin_x': -7.5,
        'map_origin_y': -7.5,
        'map_width_px': 300,
        'map_height_px': 300,
        'num_particles': 500,
        'num_rays_subsample': 60,
        'lidar_yaw_offset_deg': 180.0,
    }

    mcl = Node(
        package='puzzlebot_pallet_loader',
        executable='mcl_localization_node',
        name='mcl_localization_node',
        output='screen',
        parameters=[mcl_params],
    )

    navigator = Node(
        package='puzzlebot_pallet_loader',
        executable='hybrid_a_star_bug0_node',
        name='hybrid_a_star_bug0_node',
        output='screen',
        parameters=[
            nav_params,
            {
                'cmd_vel_topic': '/cmd_vel',
                'odom_topic': '/mcl_odom',
                'scan_topic': '/scan',
                'enable_keyboard': False,
            },
        ],
    )

    return LaunchDescription([
        mcl,
        navigator,
    ])
