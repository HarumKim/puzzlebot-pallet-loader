from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    odometry_node = Node(
        package='slam',
        executable='odometry_node',
        name='odometry_node',
        output='screen'
    )

    occupancy_grid_mapper_node = Node(
        package='slam',
        executable='occupancy_grid_mapper',
        name='occupancy_grid_mapper',
        output='screen'
    )

    mcl_node = Node(
        package='slam',
        executable='mcl_node',
        name='mcl_node',
        output='screen'
    )

    rviz_config = os.path.join(
        get_package_share_directory('slam'),
        'rviz',
        'rviz_slam_map.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),

        odometry_node,
        occupancy_grid_mapper_node,
        mcl_node,
        rviz_node,
    ])
