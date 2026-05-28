from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    odometry_node = Node(
        package='puzzlebot_pallet_loader',
        executable='odometry_node',
        name='odometry_node',
        output='screen'
    )

    astar_config = os.path.join(
        get_package_share_directory('puzzlebot_pallet_loader'),
        'config',
        'pista_obtacles.yaml'
    )

    hybrid_a_star_bug0_node = Node(
        package='puzzlebot_pallet_loader',
        executable='hybrid_a_star_bug0_node',
        name='hybrid_a_star_bug0_node',
        output='screen',
        parameters=[
            astar_config,
            {
                'cmd_vel_topic': '/cmd_vel',
                'odom_topic': '/ground_truth',#'/odom',
                'scan_topic': '/scan',
            }
        ]
    )

    return LaunchDescription([
        odometry_node,
        hybrid_a_star_bug0_node,
    ])