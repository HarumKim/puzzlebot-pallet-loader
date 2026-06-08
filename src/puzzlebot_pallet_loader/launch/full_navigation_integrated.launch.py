"""
full_navigation_integrated.launch.py

Launch integrado:
  camera_bridge             -> republica raw en /detection/annotated/compressed
  aruco_detector_node
  ekf_aruco_localization_node
  hybrid_a_star_bug0_node   -> /nav/cmd_vel
  qr_alignP                 -> /qr_align/cmd_vel + /qr_align/status
  yolo_node                 -> /yolo/target + /detection/annotated/compressed
  fsm_control_node          -> /cmd_vel final
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('puzzlebot_pallet_loader')
    ekf_aruco_params = os.path.join(pkg, 'config', 'ekf_aruco_params.yaml')
    nav_params       = os.path.join(pkg, 'config', 'real_robot_nav_params.yaml')

    camera_bridge = Node(
        package='puzzlebot_pallet_loader',
        executable='camera_bridge',
        name='camera_bridge',
        output='screen',
    )

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

    navigator = Node(
        package='puzzlebot_pallet_loader',
        executable='hybrid_a_star_bug0_node',
        name='hybrid_a_star_bug0_node',
        output='screen',
        parameters=[
            nav_params,
            {
                'cmd_vel_topic': '/nav/cmd_vel',
                'odom_topic':    '/ekf_odom',
                'scan_topic':    '/scan',
                'enable_keyboard': False,
                'wait_for_localization_convergence': False,
            },
        ],
    )

    qr_align = Node(
        package='puzzlebot_pallet_loader',
        executable='qr_alignP',
        name='qr_alignP',
        output='screen',
        parameters=[
            {
                'cmd_vel_topic':      '/qr_align/cmd_vel',
                'enable_topic':       '/qr_align/enable',
                'status_topic':       '/qr_align/status',
                'load_enable_topic':  '/qr_align/load_enable',
                'udp_port':           5004,
            }
        ],
    )

    yolo_node = Node(
        package='puzzlebot_pallet_loader',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[
            {
                'enable_topic': '/yolo/enable',
                'qr_topic': '/qr_data',
                'target_topic': '/yolo/target',
                'image_topic': '/camera/image_raw/compressed',
                'annotated_topic': '/yolo/annotated/compressed',
                'detections_topic': '/detections',
                'confidence_threshold': 0.50,
                'center_deadband_ratio': 0.10,
                'require_qr_data': True,
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
                'nav_cmd_vel_topic':     '/nav/cmd_vel',
                'qr_cmd_vel_topic':      '/qr_align/cmd_vel',
                'cmd_vel_topic':         '/cmd_vel',
                'qr_status_topic':       '/qr_align/status',
                'qr_enable_topic':       '/qr_align/enable',
                'yolo_enable_topic': '/yolo/enable',
                'qr_load_enable_topic':  '/qr_align/load_enable',
                'aruco_topic':           '/aruco_measurements',
                'auto_resume_after_qr_done': False,
                'qr_detection_debounce': 2,
                'reverse_distance_m':    0.20,
                'reverse_speed_m_s':     0.03,
                # Drop waypoints: [x1,y1,yaw1, x2,y2,yaw2, x3,y3,yaw3]
                # Ajusta las coordenadas a tu arena
                'drop_waypoints': [
                    1.80, 1.29, 0.0,   # WP_DROP_1 (izquierda)
                    1.80, 0.85, 0.0,   # WP_DROP_2 (centro)
                    1.80, 0.32, 0.0,   # WP_DROP_3 (derecha)
                ],
                'yolo_scan_timeout_sec': 15.0,
            }
        ],
    )

    return LaunchDescription([
        camera_bridge,
        aruco_detector,
        ekf_localization,
        navigator,
        qr_align,
        yolo_node,
        fsm_control,
    ])