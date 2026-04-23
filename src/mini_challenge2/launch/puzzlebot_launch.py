import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import  EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LocalSubstitution

def generate_launch_description():

    # 1. Set the path to the URDF file
    urdf_file_name = 'puzzlebot.urdf'
    urdf = os.path.join(
        get_package_share_directory('mini_challenge2'),
        'urdf',
        urdf_file_name)

    # 2. Open and read the URDF file
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # 3. Create the robot_state_publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 4. Create the joint_state_publisher node
    # jsp_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     output='screen'
    # )

    # Run custom node instead of built-int ROS 2 package
    mover_node = Node(
        package='mini_challenge2',
        executable='circular_movement',
        output='screen'
    )

    # 5. Create the RViz2 node for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )


    shutdown_on_exit = [RegisterEventHandler(
                            OnProcessExit(
                                target_action=node,
                                on_exit=[
                                    LogInfo(msg=(EnvironmentVariable(name='USER'),
                                            ' exited the node')),
                                    EmitEvent(event=Shutdown(
                                        reason='Node Exited'))
                                ]
                            )
                        ) for node in [rsp_node, mover_node]
                    ]
 

    # Ensure full shutdown when SIGINT (Ctrl+C) is received
    shutdown_log = RegisterEventHandler(
                                    OnShutdown(
                                        on_shutdown=[LogInfo(
                                            msg=['Launch was asked to shutdown: ',
                                                LocalSubstitution('event.reason')]
                                        )]
                                    )
                                )

    # 6. Add all nodes to the LaunchDescription
    l_d = LaunchDescription([rsp_node, mover_node, rviz_node, *shutdown_on_exit, shutdown_log])
    return l_d