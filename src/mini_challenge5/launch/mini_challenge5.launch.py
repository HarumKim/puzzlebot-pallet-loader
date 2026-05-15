from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_base = LaunchConfiguration('wheel_base')
    noise_gain_right = LaunchConfiguration('noise_gain_right')
    noise_gain_left = LaunchConfiguration('noise_gain_left')
    k_linear = LaunchConfiguration('k_linear')
    k_angular = LaunchConfiguration('k_angular')
    k_drift = LaunchConfiguration('k_drift')
    sample_time = LaunchConfiguration('sample_time')

    pkg_share = get_package_share_directory('mini_challenge5')

    urdf_file = os.path.join(
        pkg_share,
        'urdf',
        'puzzlebot.urdf'
    )

    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    rviz_config = os.path.join(
        pkg_share,
        'rviz',
        'mini_challenge5.rviz'
    )

    puzzlebot_node = Node(
        package='mini_challenge5',
        executable='puzzlebot_sim',
        name='puzzlebot_kinematic_model',
        output='screen',
        parameters=[{
            'wheel_radius': wheel_radius,
            'wheel_base': wheel_base,
            'noise_gain_right': noise_gain_right,
            'noise_gain_left': noise_gain_left,
            'k_linear': k_linear,
            'k_angular': k_angular,
            'k_drift': k_drift,
            'sample_time': sample_time,
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'wheel_radius',
            default_value='0.05',
            description='Puzzlebot wheel radius in meters'
        ),
        DeclareLaunchArgument(
            'wheel_base',
            default_value='0.19',
            description='Puzzlebot wheel base in meters'
        ),
        DeclareLaunchArgument(
            'noise_gain_right',
            default_value='0.016',
            description='Gaussian noise gain for right wheel encoder'
        ),
        DeclareLaunchArgument(
            'noise_gain_left',
            default_value='0.016',
            description='Gaussian noise gain for left wheel encoder'
        ),
        DeclareLaunchArgument(
            'k_linear',
            default_value='0.08',
            description='Linear motion covariance gain'
        ),
        DeclareLaunchArgument(
            'k_angular',
            default_value='0.08',
            description='Angular motion covariance gain'
        ),
        DeclareLaunchArgument(
            'k_drift',
            default_value='0.04',
            description='Lateral drift covariance gain'
        ),
        DeclareLaunchArgument(
            'sample_time',
            default_value='0.01',
            description='Simulation sample time in seconds'
        ),
        puzzlebot_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])