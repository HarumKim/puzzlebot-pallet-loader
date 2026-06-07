# ══════════════════════════════════════════════════════════════════════════════
# SECUENCIA DE ARRANQUE — Mission 1
#
# ── JETSON (robot) ────────────────────────────────────────────────────────────
#
#   Terminal 1 — Micro-ROS 
#   Terminal 2 — Publicador de cámara con IP a la PC
#
# ── PC (computadora) ────────────────────────────────────────────────
#
#   Terminal 3 — Interfaz web Flask (iniciar antes que el launch):
#   Terminal 4 — Launch de la misión (este archivo):
# ══════════════════════════════════════════════════════════════════════════════

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Puente de cámara permanente — mantiene el video en la interfaz web
        # entre transiciones de estados donde no hay nodo de visión activo.
        Node(
            package='puzzlebot_pallet_loader',
            executable='camera_bridge',
            name='camera_bridge',
            output='screen',
        ),

        # Máquina de estados principal — lanza y mata qr_alignP y yolo_node
        # en los momentos correctos de la secuencia de misión.
        Node(
            package='puzzlebot_pallet_loader',
            executable='fsm_test',
            name='fsm_test',
            output='screen',
        ),

    ])