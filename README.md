<div align="center">

# Puzzlebot Pallet Loader

**Sistema autónomo de carga de pallets con visión artificial, reconocimiento de voz y control de elevador físico**

Proyecto final — Integración de Robótica y Sistemas Inteligentes · Manchester Robotics · E80

![ROS2](https://img.shields.io/badge/ROS_2-Humble-blue?logo=ros)
![Python](https://img.shields.io/badge/Python-3.10+-yellow?logo=python)
![Platform](https://img.shields.io/badge/Platform-Jetson_Nano_+_PC-green)
![YOLOv8](https://img.shields.io/badge/Vision-YOLOv8-purple)

</div>

---

## Tabla de contenidos

- [Estructura del repositorio](#estructura-del-repositorio)
- [Proyecto final](#proyecto-final--puzzlebot_pallet_loader)
  - [Arquitectura del sistema](#arquitectura-del-sistema)
  - [Nodos principales](#nodos-principales)
  - [Lanzar el sistema](#lanzar-el-sistema)
- [Retos semanales](#retos-semanales)
- [Paquetes de soporte](#paquetes-de-soporte)
- [Requisitos](#requisitos)
- [Hardware](#hardware)
- [Equipo](#equipo)

---

## Estructura del repositorio

```
puzzlebot-pallet-loader/
├── src/
│   ├── puzzlebot_pallet_loader/   # ★ Proyecto final
│   │
│   ├── mini_challenge2/           # Reto 2 — Cinemática básica
│   ├── mini_challenge3/           # Reto 3 — Simulación cinemática
│   ├── mini_challenge4/           # Reto 4 — Simulación multi-robot
│   ├── mini_challenge5/           # Reto 5 — Ruido y perturbaciones
│   ├── mini_challenge6/           # Reto 6 — Navegación Bug
│   ├── mini_challenge7/           # Reto 7 — EKF + ArUco
│   ├── bugs_a_star/               # Bug0 / Bug2 / A* / Hybrid A*
│   ├── slam/                      # SLAM con MCL y mapa de ocupación
│   │
│   ├── camera_dataset/            # Captura y calibración de cámara
│   ├── fpga_lifter_bridge/        # Control del elevador vía SPI / FPGA
│   ├── voz_puzzlebot/             # Reconocimiento de voz offline (HMM)
│   ├── puzzlebot_description/     # URDF y visualización del robot
│   └── puzzlebot_gazebo/          # Simulación en Gazebo
│
├── web-interface/                 # Dashboard Flask — detecciones + mapa
├── voice-recognition/             # Pipeline de entrenamiento HMM
└── docker-compose.yml
```

---

## Proyecto final — `puzzlebot_pallet_loader`

> Sistema autónomo de carga de pallets que combina localización EKF/MCL, detección YOLOv8, alineación por código QR, reconocimiento de voz y control de un elevador físico vía FPGA. Todo el proceso puede monitorearse en tiempo real desde cualquier navegador en la red a través de una interfaz web con video anotado en vivo, mapa de localización y estado de las detecciones.

### Arquitectura del sistema

El sistema se distribuye entre una **Jetson Nano** y una **PC con GPU**, comunicadas vía ROS 2 sobre la misma red.

| Componente | Corre en | Función |
|:---|:---:|:---|
| `camera_publisher` | Jetson Nano | Publica imagen CSI vía GStreamer → `/camera/image_raw/compressed` |
| `yolo_node` | PC | Detección YOLOv8 → `/detections` + `/detection/annotated/compressed` |
| `aruco_detector_node` | PC | Detección de marcadores ArUco para localización fina |
| `ekf_aruco_localization_node` | PC | EKF fusionando odometría + ArUco |
| `mcl_localization_node` | PC | Localización Monte Carlo |
| `hybrid_a_star_bug0_node` | PC | Planificación Hybrid A* + navegación reactiva Bug0 |
| `fsm_control_node` | Jetson / PC | Máquina de estados de la misión completa |
| `qr_alignP` | PC | Alineación visual fina al código QR del pallet |
| `voice_recognition` | PC | Reconocimiento de voz para comandos de misión |
| `lifter_spi_node` | Jetson | Control SPI del elevador (FPGA) |
| `web-interface/app.py` | PC / laptop | Dashboard Flask en `http://<ip>:8002` |


<img width="979" height="651" alt="image" src="https://github.com/user-attachments/assets/80ee10e2-b39d-4fce-a19d-9ca8bf43470c" />

### Nodos principales

<details>
<summary><b>camera_bridge</b> — Puente de imagen para la interfaz web</summary>

Resuscribe la imagen cruda de la cámara y la republica en `/detection/annotated/compressed` para que el dashboard Flask pueda mostrar el video anotado en tiempo real. Se lanza automáticamente con `full_navigation_integrated.launch.py`.
</details>

<details>
<summary><b>fsm_control_node</b> — Máquina de estados de la misión</summary>

Orquesta la misión completa: navegación al pallet → alineación QR → activación del elevador → traslado al destino → descenso. Es el nodo central que coordina todos los demás.
</details>

<details>
<summary><b>qr_alignP</b> — Alineación visual al pallet</summary>

Usa la cámara para detectar el código QR del pallet y generar comandos de velocidad correctivos hasta quedar centrado y a la distancia correcta para el elevador.
</details>

<details>
<summary><b>ekf_aruco_localization_node</b> — Localización EKF</summary>

Fusiona la odometría del robot con la posición estimada a partir de marcadores ArUco usando un Filtro de Kalman Extendido, reduciendo el drift acumulado.
</details>

<details>
<summary><b>hybrid_a_star_bug0_node</b> — Planificación y navegación</summary>

Combina Hybrid A* para planificación global con Bug0 para evasión reactiva de obstáculos no mapeados.
</details>

<details>
<summary><b>voice_recognition</b> — Comandos por voz</summary>

Escucha comandos de voz en tiempo real usando un modelo HMM entrenado offline y publica el comando reconocido al FSM para iniciar o detener misiones.
</details>

<details>
<summary><b>lifter_spi_node</b> — Control del elevador</summary>

Interfaz SPI entre ROS 2 y la FPGA que gobierna el mecanismo de elevación del pallet. Recibe comandos de altura del FSM y los traduce a señales de hardware.
</details>

### Lanzar el sistema

Cada comando se corre en una terminal separada. Reemplaza `<IP_PC>` con la IP de tu PC en la red local.

#### En la Jetson Nano

```bash
# 1. Agente micro-ROS (conecta el firmware del robot con ROS 2)
ros2 launch puzzlebot_ros micro_ros_agent.launch.py

# 2. Publicador de cámara CSI (envía imagen a la PC vía UDP)
ros2 run puzzlebot_pallet_loader camera_publisher \
    --ros-args -p udp_host:=<IP_PC>

# 3. LiDAR SLLIDAR A1
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0

# 4. Control SPI del elevador (FPGA)
ros2 run fpga_lifter_bridge lifter_spi_node
```

#### En la PC

```bash
# 5. Navegación + visión + FSM completo
ros2 launch puzzlebot_pallet_loader full_navigation_integrated.launch.py

# 6. Dashboard web — levanta la interfaz en http://<IP_PC>:8002
docker compose up

# 7. Simulación y debug de localización Monte Carlo (RViz2)
ros2 launch puzzlebot_pallet_loader mcl_rviz_debug.launch.py

# 8. Nodo de reconocimiento de voz — activa detección por comando
ros2 run voz_puzzlebot comando_node
```

---

## Retos semanales

<details>
<summary><b>mini_challenge2</b> — Cinemática básica</summary>

Control de velocidad y movimiento básico del Puzzlebot. Introducción al modelo diferencial y publicación de comandos de velocidad.
</details>

<details>
<summary><b>mini_challenge3</b> — Simulación cinemática</summary>

Simulación del modelo cinemático diferencial del robot en ROS 2 (`puzzlebot_sim.py`). El robot virtual responde a comandos `/cmd_vel` y publica odometría.
</details>

<details>
<summary><b>mini_challenge4</b> — Simulación multi-robot</summary>

Simulación de múltiples Puzzlebots simultáneos con control de posición individual y publicación de estados de juntas para RViz2.
</details>

<details>
<summary><b>mini_challenge5</b> — Ruido y perturbaciones</summary>

Análisis del efecto de ruido y perturbaciones sobre el movimiento circular del robot. Incluye `circular_movement.py` para reproducir trayectorias con y sin perturbaciones.
</details>

<details>
<summary><b>mini_challenge6</b> — Navegación Bug</summary>

Implementación de los algoritmos **Bug0** y **Bug2** para navegación hacia un objetivo evitando obstáculos desconocidos.
</details>

<details>
<summary><b>mini_challenge7</b> — EKF + ArUco</summary>

Localización con Filtro de Kalman Extendido fusionando odometría y marcadores ArUco. Incluye visualización de elipse de covarianza y navegación **Tangent Bug**.
</details>

<details>
<summary><b>bugs_a_star</b> — Comparativa de navegación</summary>

Implementación y comparación de Bug0, Bug2, A* clásico y Hybrid A*-Bug0. Útil como banco de pruebas para distintas estrategias de navegación.
</details>

<details>
<summary><b>slam</b> — Mapeo y localización</summary>

SLAM con localización Monte Carlo (MCL), construcción de mapa de ocupación (`occupancy_grid_mapper.py`) y visualización en tiempo real vía RViz2.
</details>

---

## Paquetes de soporte

| Paquete | Descripción |
|:---|:---|
| `camera_dataset` | Captura de imágenes, calibración de cámara y medición de distancia por QR. Genera datasets de entrenamiento. |
| `fpga_lifter_bridge` | Puente SPI entre ROS 2 y la FPGA del elevador. Incluye diseño Verilog en `/verilog`. |
| `voz_puzzlebot` | Reconocimiento de voz offline con HMM. El modelo (`modelo_voz.pkl`) publica comandos al FSM. |
| `voice-recognition` | Pipeline de entrenamiento HMM: procesamiento de audio, codebook, cuantización vectorial y clasificación. |
| `puzzlebot_description` | URDF del Puzzlebot con meshes para visualización en RViz2. |
| `puzzlebot_gazebo` | Entorno Gazebo con mundos, plugins y modelos del robot para simulación completa. |

---

## Requisitos

**PC (procesamiento)**
- ROS 2 Humble
- Python 3.10+
- `ultralytics` (YOLOv8), `opencv-python`, `torch`
- `flask`, `flask-socketio`

**Jetson Nano**
- Ubuntu 22.04 (imagen comunidad)
- ROS 2 Humble
- CUDA 10.2

---

## Hardware

| Componente | Rol |
|:---|:---|
| Puzzlebot (Manchester Robotics) | Plataforma móvil diferencial |
| Jetson Nano 2GB | Cómputo de borde — cámara, actuadores, SPI |
| Cámara CSI | Visión — transmisión vía GStreamer/UDP |
| FPGA | Control del mecanismo elevador del pallet |
| PC con GPU | Inferencia YOLOv8, localización, planificación |
| Puente H | Control de motores de tracción |

<div align="center">
  <img src="https://github.com/user-attachments/assets/feff99ef-5526-4f72-ba92-62febb318f62" width="800" />
</div>


---

## Equipo

<div align="center">

| Integrante |
|:---:|
| Valeria Meneses |
| Harum Kim |
| Mateo Sánchez |
| Alondra Caspeta |

*Semestre enero – junio 2026*

</div>
