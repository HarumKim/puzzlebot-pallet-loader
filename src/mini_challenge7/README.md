# Mini Challenge 7 — Localización EKF + Navegación Autónoma (Final Challenge MCR2)

Paquete ROS 2 para el reto final del curso MCR2. Implementa dos partes:

- **Parte 1:** Localización con Filtro de Kalman Extendido (EKF) usando encoders + marcadores ArUco.
- **Parte 2:** Navegación autónoma en circuito cerrado por un laberinto con el algoritmo **Tangent Bug**, usando la pose estimada por el EKF.

---

## Arquitectura del sistema

```
Gazebo (sim)
 ├─ /joint_states       → ekf_aruco_localization_node
 ├─ /camera             → aruco_detector_node
 ├─ /camera_info        → aruco_detector_node
 └─ /scan               → tangent_bug_node

aruco_detector_node
 └─ /aruco_measurements → ekf_aruco_localization_node

ekf_aruco_localization_node
 ├─ /ekf_odom           → tangent_bug_node
 └─ TF: odom → base_link → laser

tangent_bug_node
 └─ /cmd_vel            → Gazebo (mueve el robot)

covariance_visualizer_node
 └─ /covariance_ellipse → RViz
```

---

## Nodos

### `aruco_detector_node`
Detecta marcadores ArUco 4×4 en la imagen de la cámara y publica mediciones de distancia y ángulo.

- **Suscribe:** `/camera`, `/camera_info`
- **Publica:** `/aruco_measurements` — `[marker_id, range, bearing]`
- **Config:** `config/aruco_map.yaml` (diccionario, tamaño del marcador 0.15 m)

### `ekf_aruco_localization_node`
EKF de 3 estados `[x, y, θ]`. **No es SLAM** — los landmarks son posiciones fijas conocidas del mapa.

- **Predicción:** modelo cinemático diferencial usando `/joint_states` (velocidades de rueda). Propaga covarianza con Jacobiano G y ruido de proceso Q.
- **Corrección:** cuando llega una medición ArUco, aplica update EKF con modelo de observación `h(μ) = [range, bearing]`. Incluye gate de Mahalanobis (χ² 95%, umbral 5.991) para rechazar outliers.
- **Suscribe:** `/joint_states`, `/aruco_measurements`
- **Publica:** `/ekf_odom` (nav_msgs/Odometry con covarianza 3×3 embebida), TF `odom → base_link`, TF estático `base_link → laser`
- **Config:** `config/ekf_params.yaml`, `config/aruco_map.yaml`

### `tangent_bug_node`
Algoritmo Tangent Bug con dos estados:

**MOTION_TO_GOAL:**
- Cada ciclo calcula el mejor ángulo de steering usando discontinuidades del LIDAR.
- Si el camino al goal está libre → apunta directo al goal.
- Si no → detecta bordes de obstáculo (discontinuidades LIDAR), elige el que minimiza `h = d(robot,T) + d(T,goal)`, y **apunta en esa dirección** (nunca va físicamente al punto T).
- Entra a BOUNDARY_FOLLOW cuando el frente choca (< `obstacle_threshold`).

**BOUNDARY_FOLLOW:**
- Wall-following clásico con detección de esquinas exteriores.
- Sale cuando `bf_traveled > bf_min_travel` Y (camino al goal libre O hay un tangente con `d(T,goal) < d_bf - bf_exit_margin`).
- Elige automáticamente el lado de la pared (izquierda/derecha) según la posición del goal.

**Circuito cerrado:** al llegar al último waypoint, reinicia el índice a 0 y repite indefinidamente (cuenta laps). Log throttled cada ~1 s: `State=..., wp=..., dist=..., front=..., bf_traveled=...`

- **Suscribe:** `/ekf_odom`, `/scan`
- **Publica:** `/cmd_vel`
- **Config:** `config/final_challenge_nav.yaml`

### `covariance_visualizer_node`
Dibuja la elipse de confianza al 95% del EKF para visualización en RViz.

- **Suscribe:** `/ekf_odom`
- **Publica:** `/covariance_ellipse` (visualization_msgs/Marker)

---

## Mundo y waypoints

**Mundo:** `mini_challenge7_avoidance_1.world` — laberinto con 6 paredes internas y 4 marcadores ArUco.

**Posición inicial del robot:** `(1.45, 3.30)`, orientación `π` (apuntando al sur).

**Circuito (5 waypoints, el último cierra en el inicio):**

| # | Nombre  | Posición (x, y)  | Descripción            |
|---|---------|------------------|------------------------|
| 0 | goal_1  | (-3.30, -0.15)   | Oeste, fuera laberinto |
| 1 | goal_2  | ( 1.50, -0.85)   | Este, corredor bajo    |
| 2 | goal_3  | (-1.45,  0.00)   | Centro, corredor medio |
| 3 | goal_4  | ( 1.25,  1.35)   | Zona superior          |
| 4 | inicio  | ( 1.45,  3.30)   | Cierra el circuito     |

**Marcadores ArUco en el mapa:**

| ID | Posición (x, y)   | Yaw (rad) |
|----|-------------------|-----------|
| 0  | (-2.65, -0.10)    | π/2       |
| 1  | ( 1.60, -1.20)    | π         |
| 2  | (-1.90,  0.00)    | 0         |
| 3  | ( 1.50,  1.20)    | π         |

---

## Archivos de configuración

| Archivo | Descripción |
|---------|-------------|
| `config/ekf_params.yaml` | Parámetros físicos del robot, ruido Q/R, pose inicial |
| `config/aruco_map.yaml` | IDs y posiciones de marcadores ArUco + parámetros del detector |
| `config/final_challenge_nav.yaml` | Waypoints, velocidades, umbrales del Tangent Bug |
| `config/camera_calibration.yaml` | Calibración intrínseca de la cámara |

### Parámetros principales del Tangent Bug (`final_challenge_nav.yaml`)

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `obstacle_threshold` | 0.35 m | Distancia frontal para entrar a BF |
| `wall_distance` | 0.35 m | Distancia objetivo al muro en BF |
| `discontinuity_threshold` | 0.35 m | Diferencia mínima entre rayos LIDAR para detectar borde |
| `bf_min_travel` | 0.50 m | Distancia mínima en BF antes de buscar salida |
| `bf_exit_margin` | 0.10 m | Margen extra para la condición de salida de BF |
| `goal_tolerance` | 0.22 m | Radio para considerar waypoint alcanzado |
| `max_linear_speed` | 0.22 m/s | Velocidad lineal máxima |
| `max_angular_speed` | 0.70 rad/s | Velocidad angular máxima |
| `wait_for_localization` | false | Si `true`, espera a que la covarianza EKF converja antes de moverse |

---

## Cómo compilar

Solo la primera vez o cuando cambias código Python:

```bash
cd ~/puzzlebot-pallet-loader
colcon build --symlink-install --packages-select mini_challenge7
source install/setup.bash
```

> Los archivos YAML y SDF **no requieren rebuild** gracias a `--symlink-install`.

---

## Cómo correr

### Parte 1 — Solo EKF + ArUco (sin navegación)

Primero corre Gazebo en la Terminal 1, luego el stack de localización en la Terminal 2.

**Terminal 1 — Simulación:**
```bash
source install/setup.bash
ros2 launch mini_challenge7 final_challenge_sim.launch.py
```

**Terminal 2 — Localización:**
```bash
source install/setup.bash
ros2 launch mini_challenge7 part1_ekf_aruco_launch.py
```

Verifica la pose estimada:
```bash
ros2 topic echo /ekf_odom --field pose.pose
```

Abre RViz para ver la elipse de covarianza:
```bash
rviz2 -d src/mini_challenge7/rviz/final_challenge.rviz
```

---

### Parte 2 — Navegación autónoma completa (Final Challenge)

**Terminal 1 — Simulación:**
```bash
source install/setup.bash
ros2 launch mini_challenge7 final_challenge_sim.launch.py
```

**Terminal 2 — EKF + Navegación + RViz:**
```bash
source install/setup.bash
ros2 launch mini_challenge7 final_challenge_navigation.launch.py
```

Para correr sin RViz:
```bash
ros2 launch mini_challenge7 final_challenge_navigation.launch.py rviz:=false
```

---

## Monitoreo en tiempo real

```bash
# Pose EKF
ros2 topic echo /ekf_odom --field pose.pose

# Covarianza yaw (índice 35 del array 6x6)
ros2 topic echo /ekf_odom --field pose.covariance

# Velocidades enviadas al robot
ros2 topic echo /cmd_vel

# Detecciones ArUco
ros2 topic echo /aruco_measurements

# Ver log del nodo de navegación (Estado, wp, dist, front)
ros2 topic echo /rosout | grep tangent_bug
```

---

## Topics principales

| Topic | Tipo | Dirección | Descripción |
|-------|------|-----------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Gazebo → EKF | Velocidades de rueda |
| `/camera` | `sensor_msgs/Image` | Gazebo → ArUco | Imagen de cámara |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo → Tangent Bug | LIDAR 360° |
| `/aruco_measurements` | `std_msgs/Float32MultiArray` | ArUco → EKF | `[id, range, bearing]` |
| `/ekf_odom` | `nav_msgs/Odometry` | EKF → Tangent Bug | Pose estimada + covarianza |
| `/cmd_vel` | `geometry_msgs/Twist` | Tangent Bug → Gazebo | Comandos de velocidad |
| `/covariance_ellipse` | `visualization_msgs/Marker` | Visualizer → RViz | Elipse 95% confianza |

---

## Estructura del paquete

```
mini_challenge7/
├── config/
│   ├── aruco_map.yaml              # Mapa de marcadores ArUco
│   ├── camera_calibration.yaml     # Calibración cámara
│   ├── ekf_params.yaml             # Parámetros EKF
│   └── final_challenge_nav.yaml    # Waypoints y parámetros Tangent Bug
├── launch/
│   ├── final_challenge_sim.launch.py        # Gazebo + robot
│   ├── final_challenge_navigation.launch.py # EKF + Tangent Bug + RViz
│   └── part1_ekf_aruco_launch.py            # Solo EKF (Parte 1)
├── mini_challenge7/
│   ├── aruco_detector_node.py
│   ├── ekf_aruco_localization_node.py
│   ├── tangent_bug_node.py
│   ├── covariance_visualizer_node.py
│   ├── bug0_navigation_node.py     # Algoritmos alternativos (referencia)
│   ├── bug1_navigation_node.py
│   └── bug2_navigation_node.py
├── rviz/
│   └── final_challenge.rviz
└── meshes/                         # Modelos 3D del Puzzlebot
```
