import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # World generated from manual obstacles and the SLAM map floor.
    world = 'pista_octavo.world'

    pause = 'false'
    verbosity = '4'
    use_sim_time = 'true'

    # Spawn inside the calibrated coordinate system.
    # Change x/y/yaw if you want another start pose.
    robot_config_list = [
        {
            'name': '',
            'type': 'puzzlebot_jetson_lidar_ed',
            'x': 0.25,
            'y': 0.25,
            'yaw': 0.0,
            'lidar_frame': 'laser_frame',
            'camera_frame': 'camera_link_optical',
            'tof_frame': 'tof_link',
        }
    ]

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('puzzlebot_gazebo'),
                'launch',
                'gazebo_world_launch.py'
            )
        ),
        launch_arguments={
            'world': world,
            'pause': pause,
            'verbosity': verbosity,
        }.items()
    )

    robot_launches = []
    for robot in robot_config_list:
        robot_name = robot['name']
        prefix = f'{robot_name}/' if robot_name != '' else ''

        robot_launches.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('puzzlebot_gazebo'),
                        'launch',
                        'gazebo_puzzlebot_launch.py'
                    )
                ),
                launch_arguments={
                    'robot': robot['type'],
                    'robot_name': robot_name,
                    'x': str(robot.get('x', 0.0)),
                    'y': str(robot.get('y', 0.0)),
                    'yaw': str(robot.get('yaw', 0.0)),
                    'prefix': prefix,
                    'lidar_frame': robot.get('lidar_frame', 'laser_frame'),
                    'camera_frame': robot.get('camera_frame', 'camera_link_optical'),
                    'tof_frame': robot.get('tof_frame', 'tof_link'),
                    'use_sim_time': use_sim_time,
                }.items()
            )
        )

    return LaunchDescription([
        gazebo_launch,
        *robot_launches,
    ])
