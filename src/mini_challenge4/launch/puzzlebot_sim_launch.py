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


def create_robot(namespace, initial_x, initial_y, initial_theta, goal_x, goal_y, goal_theta):
    package_name = 'mini_challenge4'

    urdf_file_name = 'puzzlebot.urdf'
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file_name
    )

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc},
            {'frame_prefix': namespace + '/'}
        ]
    )

    puzzlebot_sim_node = Node(
        package=package_name,
        executable='puzzlebot_sim',
        namespace=namespace,
        name='puzzlebot_sim',
        output='screen',
        parameters=[{
            'initial_x': initial_x,
            'initial_y': initial_y,
            'initial_theta': initial_theta
        }]
    )

    localisation_node = Node(
        package=package_name,
        executable='localisation',
        namespace=namespace,
        name='localisation',
        output='screen',
        parameters=[{
            'initial_x': initial_x,
            'initial_y': initial_y,
            'initial_theta': initial_theta
        }]
    )

    joint_state_pub_node = Node(
        package=package_name,
        executable='joint_state_pub',
        namespace=namespace,
        name='joint_state_pub',
        output='screen',
        parameters=[{
            'frame_prefix': namespace
        }]
    )

    point_stabilisation_node = Node(
        package=package_name,
        executable='point_stabilisation_control',
        namespace=namespace,
        name='point_stabilisation_control',
        output='screen',
        parameters=[{
            'goal_x': goal_x,
            'goal_y': goal_y,
            'goal_theta': goal_theta
        }]
    )

    return [
        robot_state_pub_node,
        puzzlebot_sim_node,
        localisation_node,
        joint_state_pub_node,
        point_stabilisation_node
    ]


def generate_launch_description():

    robot1_nodes = create_robot(
        namespace='robot1',
        initial_x=0.0,
        initial_y=0.0,
        initial_theta=0.0,
        goal_x=1.0,
        goal_y=1.0,
        goal_theta=0.0
    )

    robot2_nodes = create_robot(
        namespace='robot2',
        initial_x=0.0,
        initial_y=-1.0,
        initial_theta=0.0,
        goal_x=1.0,
        goal_y=-2.0,
        goal_theta=0.0
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription(robot1_nodes + robot2_nodes + [rviz_node])