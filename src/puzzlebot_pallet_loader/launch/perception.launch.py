from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    udp_host_arg = DeclareLaunchArgument(
        'udp_host',
        default_value='10.42.0.160',
        description='IP de la computadora que recibe el stream UDP',
    )

    camera = Node(
        package='puzzlebot_pallet_loader',
        executable='camera_publisher',
        name='camera_publisher',
        parameters=[{
            'udp_host': LaunchConfiguration('udp_host'),
            'udp_port': 5004,
            'ros_fps': 25,
            'jpeg_quality': 80,
        }],
        output='screen',
    )

    detector = Node(
        package='puzzlebot_pallet_loader',
        executable='detector_node',
        name='detector_node',
        output='screen',
    )

    return LaunchDescription([udp_host_arg, camera, detector])
