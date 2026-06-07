"""
full_navigation_integrated.launch.py

Launch integrado:
  aruco_detector_node
  ekf_aruco_localization_node
  hybrid_a_star_bug0_node  -> /nav/cmd_vel
  qr_align                 -> /qr_align/cmd_vel + /qr_align/status
  fsm_control_node          -> /cmd_vel final

El EKF sigue corrigiendo con ArUcos mientras la FSM decide si /cmd_vel viene
de navegación global o de qr_align.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('puzzlebot_pallet_loader')

    ekf_aruco_params = os.path.join(pkg, 'config', 'ekf_aruco_params.yaml')
    nav_params = os.path.join(pkg, 'config', 'real_robot_nav_params.yaml')

    aruco_detector = Node(
        package='puzzlebot_pallet_loader',
        executable='aruco_detector_node',
        name='aruco_detector_node',
        output='screen',
        parameters=[ekf_aruco_params],
    )

    ekf_localization = Node(
        package='puzzlebot_pallet_loader',
        executable='ekf_aruco_localization_node',
        name='ekf_aruco_localization_node',
        output='screen',
        parameters=[ekf_aruco_params],
    )

    # navigator = Node(
    #     package='puzzlebot_pallet_loader',
    #     executable='hybrid_a_star_bug0_node',
    #     name='hybrid_a_star_bug0_node',
    #     output='screen',
    #     parameters=[
    #         nav_params,
    #         {
    #             'cmd_vel_topic': '/nav/cmd_vel',
    #             'odom_topic': '/ekf_odom',
    #             'scan_topic': '/scan',
    #         },
    #     ],
    # )
    navigator = Node(
        package='puzzlebot_pallet_loader',
        executable='hybrid_a_star_bug0_node',
        name='hybrid_a_star_bug0_node',
        output='screen',
        parameters=[
            nav_params,
            {
                'cmd_vel_topic': '/nav/cmd_vel',
                'odom_topic': '/ekf_odom',
                'enable_keyboard': False,
                'wait_for_localization_convergence': False,
            },
        ],
    )

    qr_align = Node(
        package='puzzlebot_pallet_loader',
        executable='qr_align',
        name='qr_align',
        output='screen',
        parameters=[
            {
                'cmd_vel_topic': '/qr_align/cmd_vel',
                'enable_topic': '/qr_align/enable',
                'status_topic': '/qr_align/status',
                'camera_topic': '/camera/image_raw/compressed',
            }
        ],
    )

    fsm_control = Node(
        package='puzzlebot_pallet_loader',
        executable='fsm_control_node',
        name='fsm_control_node',
        output='screen',
        parameters=[
            {
                'nav_cmd_vel_topic': '/nav/cmd_vel',
                'qr_cmd_vel_topic': '/qr_align/cmd_vel',
                'cmd_vel_topic': '/cmd_vel',
                'qr_status_topic': '/qr_align/status',
                'qr_enable_topic': '/qr_align/enable',
                'aruco_topic': '/aruco_measurements',
                'auto_resume_after_qr_done': False,
                'qr_detection_debounce': 2,
            }
        ],
    )

    return LaunchDescription([
        aruco_detector,
        ekf_localization,
        navigator,
        qr_align,
        fsm_control,
    ])
