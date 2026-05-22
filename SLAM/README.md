# 🗺️ Occupancy Grid Mapper

**Creación de mapas 2D de ocupación desde cero** — sin depender de paquetes externos de SLAM.

Este paquete implementa un sistema de mapeo basado en **Occupancy Grid Mapping con poses conocidas**, donde se utiliza la odometría del robot como fuente de localización y el LiDAR 2D para construir el mapa del entorno en tiempo real.

---

## 📐 Arquitectura del Sistema

```
                     ┌──────────────┐
  /scan (LiDAR) ────▶│              │──▶ /map (OccupancyGrid)
                     │  mapper_node │
  /odom  ───────────▶│              │──▶ map.pgm + map.yaml + map.png
                     └──────────────┘         (al hacer Ctrl+C)
```

### Nodos

| Nodo | Ejecutable | Descripción |
|------|-----------|-------------|
| `occupancy_grid_mapper` | `mapper_node` | Construye el mapa de ocupación procesando datos de LiDAR y odometría |
| `odometry_node` | `odometry_node` | Genera la odometría del robot a partir de los encoders |

---

## 🧠 ¿Cómo Funciona?

### 1. Representación Log-Odds
Cada celda del grid almacena un valor en **log-odds** que representa la probabilidad de estar ocupada:

- `0.0` → desconocida (50% probabilidad)
- `valor negativo` → probablemente libre
- `valor positivo` → probablemente ocupada

Esto permite actualizar incrementalmente las celdas sin perder información histórica.

### 2. Bresenham Ray Tracing
Para cada rayo del LiDAR:
1. Se calcula el punto de impacto en coordenadas del mundo usando la pose del robot (odometría)
2. Se aplica el **algoritmo de Bresenham** para trazar una línea desde el robot hasta el punto de impacto
3. Las celdas intermedias se marcan como **libres** (`log_odds_free`)
4. La celda final (punto de impacto) se marca como **ocupada** (`log_odds_occ`)

### 3. Publicación del Mapa
El log-odds grid se convierte al formato estándar de ROS2 `OccupancyGrid`:
- `-1` → celda desconocida
- `0` → celda libre
- `100` → celda ocupada

### 4. Guardado del Mapa
Al presionar `Ctrl+C`, el mapa se guarda automáticamente en tres formatos:
- **`map.pgm`** — Imagen en escala de grises (compatible con `map_server`)
- **`map.yaml`** — Metadatos del mapa (resolución, origen, umbrales)
- **`map.png`** — Imagen PNG para visualización rápida

---

## ⚙️ Parámetros Configurables

| Parámetro | Default | Descripción |
|-----------|---------|-------------|
| `resolution` | `0.05` | Resolución del mapa (metros/celda) |
| `map_width` | `15.0` | Ancho del mapa en metros |
| `map_height` | `15.0` | Alto del mapa en metros |
| `log_odds_occ` | `1.2` | Incremento log-odds al marcar celda como ocupada |
| `log_odds_free` | `-0.4` | Incremento log-odds al marcar celda como libre |
| `log_odds_min` | `-3.0` | Clamp inferior de log-odds |
| `log_odds_max` | `15.0` | Clamp superior de log-odds |
| `map_publish_rate` | `1.0` | Frecuencia de publicación del mapa (Hz) |
| `save_path` | `~/SLAM/maps` | Directorio donde se guardan los archivos del mapa |

---

## 🚀 Cómo Ejecutar

### Prerrequisitos
- ROS2 (Humble / Jazzy)
- Python 3 con `numpy`

### 1. Compilar el paquete

```bash
cd ~/ros2_ws
colcon build --packages-select puzzlebot_pallet_loader
source install/setup.bash
```

### 2. Lanzar la odometría del robot

```bash
ros2 run puzzlebot_pallet_loader odometry_node
```

### 3. Lanzar el mapper

```bash
ros2 run puzzlebot_pallet_loader mapper_node
```

### 4. Visualizar el mapa en RViz2

```bash
rviz2
```
> En RViz, añadir un display de tipo **Map** suscrito al topic `/map`.

### 5. Guardar el mapa

Presiona `Ctrl+C` en la terminal del `mapper_node`. El mapa se guardará automáticamente en `~/SLAM/maps/`.

---

## 📁 Estructura del Paquete

```
puzzlebot_pallet_loader/
├── config/
│   └── slam_params.yaml          # Parámetros para slam_toolbox (alternativo)
├── launch/
│   └── slam_launch.py            # Launch file para slam_toolbox
├── puzzlebot_pallet_loader/
│   ├── occupancy_grid_mapper.py  # 🗺️ Nodo principal del mapper
│   └── odometry_node.py          # 📍 Nodo de odometría
├── package.xml
├── setup.py
└── README.md
```

---

## 📡 Topics

### Suscripciones del `mapper_node`
| Topic | Tipo | QoS | Descripción |
|-------|------|-----|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Best Effort | Datos del LiDAR 2D |
| `/odom` | `nav_msgs/Odometry` | Reliable (depth 10) | Pose del robot |

### Publicaciones del `mapper_node`
| Topic | Tipo | QoS | Descripción |
|-------|------|-----|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | Transient Local + Reliable | Mapa de ocupación |

---

## 💡 Notas

- El mapper asume **odometría como pose conocida** (sin corrección de loop-closing). Para entornos grandes donde el drift acumulado sea un problema, considerar usar `slam_toolbox` con el launch file incluido.
- El robot se posiciona inicialmente en el **centro del mapa**. Si el entorno es mayor a 15×15 m, ajustar `map_width` y `map_height`.
- Los valores de `log_odds_occ` y `log_odds_free` controlan qué tan rápido se establece la certeza de cada celda. Valores más altos en `log_odds_occ` hacen que las paredes se marquen más rápidamente.
