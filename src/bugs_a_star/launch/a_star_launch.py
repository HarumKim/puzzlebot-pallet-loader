import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    world_name = world.rsplit('.', 1)[0]

    pkg = get_package_share_directory('bugs_a_star')
    config_file = os.path.join(pkg, 'config', world_name + '.yaml')

    params = [config_file] if os.path.isfile(config_file) else []

    if not params:
        print(f'[a_star_launch] WARNING: no config found for world "{world_name}", '
              'using hardcoded defaults.')

    return [
        Node(
            package='bugs_a_star',
            executable='a_star_node',
            name='a_star_node',
            output='screen',
            parameters=params,
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='obstacle_avoidance_4.world',
            description='World file name — must match a YAML in bugs_a_star/config/'
        ),
        OpaqueFunction(function=launch_setup),
    ])