from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    voice_recognition = Node(
        package='puzzlebot_pallet_loader',
        executable='voice_recognition',
        name='voice_recognition',
        output='screen',
    )

    fsm_control = Node(
        package='puzzlebot_pallet_loader',
        executable='fsm_control_node',
        name='fsm_control_node',
        output='screen',
    )

    return LaunchDescription([
        voice_recognition,
        fsm_control,
    ])
