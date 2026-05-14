import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    world_name = world.rsplit('.', 1)[0]

    pkg = get_package_share_directory('mini_challenge6')
    config_file = os.path.join(pkg, 'config', world_name + '.yaml')

    params = [config_file] if os.path.isfile(config_file) else []
    if not params:
        print(f'[bug2_launch] WARNING: no config found for world "{world_name}", '
              'using hardcoded waypoints.')

    return [
        Node(
            package='mini_challenge6',
            executable='bug2_node',
            name='bug2_node',
            output='screen',
            parameters=params,
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='obstacle_avoidance_4.world',
            description='World file name — must match a YAML in mini_challenge6/config/'
        ),
        OpaqueFunction(function=launch_setup),
    ])
