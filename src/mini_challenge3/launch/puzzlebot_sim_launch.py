#IMPORTS REQUIRED TO SET THE PACKAGE ADDRESS (DIRECTORIES)
import os
from ament_index_python.packages import get_package_share_directory

#IMPORTS REQUIRED FOR Launching Nodes
from launch import LaunchDescription
from launch_ros.actions import Node

#IMPORTS REQUIRED FOR EVENTS AND ACTIONS
from launch.actions import  EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable


def generate_launch_description():
 
    urdf_file_name = 'puzzlebot.urdf'
    urdf_default_path = os.path.join(
                        get_package_share_directory('puzzlebot_sim'),
                        'urdf',
                        urdf_file_name)
   
    with open(urdf_default_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_pub_node = Node(
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            name='robot_state_publisher',
                            output='screen',
                            parameters=[{'robot_description': robot_desc}],
                            )

    # Nodo del simulador cinemático
    puzzlebot_sim_node = Node(
        package='puzzlebot_sim',
        executable='puzzlebot_sim',
        name='puzzlebot_sim',
        output='screen'
    )

    # Nodo de localización por dead reckoning
    localisation_node = Node(
        package='puzzlebot_sim',
        executable='localisation',
        name='localisation',
        output='screen'
    )

    # Nodo que publica joint_states y TF odom -> base_footprint
    joint_state_pub_node = Node(
        package='puzzlebot_sim',
        executable='joint_state_pub',
        name='joint_state_pub',
        output='screen'
    )

    # Nodo de control de posición
    point_stabilisation_node = Node(
        package='puzzlebot_sim',
        executable='point_stabilisation_control',
        name='point_stabilisation_control',
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_pub_node,
        puzzlebot_sim_node,
        localisation_node,
        joint_state_pub_node,
        point_stabilisation_node,
        rviz_node
    ])

    # l_d = LaunchDescription([robot_state_pub_node])

    # return l_d