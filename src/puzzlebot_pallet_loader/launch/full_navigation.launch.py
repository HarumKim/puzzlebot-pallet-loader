"""
full_navigation.launch.py

Launch completo: Localización EKF-ArUco + Navegación Híbrida A*-Bug0

Levanta:
    1. aruco_detector_node       — detecta marcadores ArUco desde /camera
                                   publica /aruco_measurements
    2. ekf_aruco_localization_node — predicción EKF con encoders +
                                   corrección con ArUco → /odom + TF odom→base_link
    3. hybrid_a_star_bug0_node   — navegación con /odom + /scan → /cmd_vel

Prerequisito:
    Gazebo corriendo con pista_octavo.world (que ya tiene los ArUcos):
        ros2 launch puzzlebot_gazebo bringup_pista_launch.py

Uso:
    ros2 launch puzzlebot_pallet_loader full_navigation.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('puzzlebot_pallet_loader')

    ekf_aruco_params = os.path.join(pkg, 'config', 'ekf_aruco_params.yaml')
    nav_params = os.path.join(pkg, 'config', 'pista_obstacles.yaml')

    # ── 1. ArUco Detector ─────────────────────────────────────────────────────
    # Suscribe: /camera, /camera_info
    # Publica:  /aruco_measurements [marker_id, range, bearing]
    #           /aruco_markers_viz  (esferas en RViz)
    aruco_detector = Node(
        package='puzzlebot_pallet_loader',
        executable='aruco_detector_node',
        name='aruco_detector_node',
        output='screen',
        parameters=[ekf_aruco_params],
    )

    # ── 2. EKF ArUco Localization ─────────────────────────────────────────────
    # Suscribe: /VelocityEncL, /VelocityEncR (predicción)
    #           /aruco_measurements          (corrección)
    # Publica:  /odom  (pose estimada con covarianza)
    #           TF:    odom → base_link
    #           TF:    base_link → laser (estático)
    ekf_localization = Node(
        package='puzzlebot_pallet_loader',
        executable='ekf_aruco_localization_node',
        name='ekf_aruco_localization_node',
        output='screen',
        parameters=[ekf_aruco_params],
    )

    # ── 3. Hybrid A*-Bug0 Navigator ───────────────────────────────────────────
    # Suscribe: /odom  (del EKF)
    #           /scan  (LiDAR)
    # Publica:  /cmd_vel
    navigator = Node(
        package='puzzlebot_pallet_loader',
        executable='hybrid_a_star_bug0_node',
        name='hybrid_a_star_bug0_node',
        output='screen',
        parameters=[
            nav_params,
            {
                'cmd_vel_topic': '/cmd_vel',
                'odom_topic': '/odom',
                'scan_topic': '/scan',
            },
        ],
    )

    return LaunchDescription([
        aruco_detector,
        ekf_localization,
        navigator,
    ])
