# SLAM Package — Occupancy Grid Mapping

Genera un mapa de ocupación 2D a partir de LiDAR y odometría, sin dependencias externas de SLAM.

## Requisitos

- ROS2 (Humble / Iron / Jazzy)
- Python 3
- numpy (`pip3 install numpy`)

## Compilar

```bash
cd ~/colcon_ws
colcon build --packages-select slam_pkg
source install/setup.bash
```

> **Nota**: asegúrate de que `~/SLAM` esté dentro de `~/colcon_ws/src/` o haz un symlink:
> ```bash
> ln -s ~/SLAM ~/colcon_ws/src/slam_pkg
> ```

## Ejecutar

### 1. Odometría (si no está corriendo ya)
```bash
ros2 run slam_pkg odometry
```

### 2. Mapper
```bash
ros2 run slam_pkg mapper
```

### 3. Visualizar en RViz2
```bash
rviz2
```
En RViz2:
1. **Add** → **By topic** → `/map` → **Map**
2. **Fixed Frame** → `map`
3. Mueve el robot y observa cómo se construye el mapa

### 4. Guardar el mapa
El mapa se guarda automáticamente al presionar `Ctrl+C` en la terminal del mapper.
Se guarda en `~/SLAM/maps/` como:
- `map.pgm` — imagen del mapa
- `map.yaml` — metadatos (resolución, origen)

## Parámetros configurables

Puedes ajustarlos al lanzar el nodo:

```bash
ros2 run slam_pkg mapper --ros-args \
  -p resolution:=0.05 \
  -p map_width:=15.0 \
  -p map_height:=15.0 \
  -p log_odds_occ:=0.65 \
  -p log_odds_free:=-0.40 \
  -p map_publish_rate:=1.0 \
  -p save_path:=/ruta/donde/guardar
```

| Parámetro | Default | Descripción |
|-----------|---------|-------------|
| `resolution` | `0.05` | Metros por celda |
| `map_width` | `15.0` | Ancho del mapa en metros |
| `map_height` | `15.0` | Alto del mapa en metros |
| `log_odds_occ` | `0.65` | Incremento log-odds al marcar celda ocupada |
| `log_odds_free` | `-0.40` | Incremento log-odds al marcar celda libre |
| `map_publish_rate` | `1.0` | Frecuencia de publicación del mapa (Hz) |
| `save_path` | `~/SLAM/maps` | Directorio donde se guarda el mapa |

## Tópicos

| Tópico | Tipo | Dirección |
|--------|------|-----------|
| `/scan` | `sensor_msgs/LaserScan` | ← suscribe |
| `/odom` | `nav_msgs/Odometry` | ← suscribe |
| `/map` | `nav_msgs/OccupancyGrid` | → publica |

## Cómo funciona

1. **Recibe** un escaneo LiDAR y la pose actual del robot (odometría)
2. **Ray tracing** (Bresenham): para cada rayo, traza una línea desde el robot hasta el punto de impacto
3. **Log-odds**: marca celdas intermedias como **libres** y el punto final como **ocupado**
4. **Publica** el mapa acumulado como `OccupancyGrid` en `/map`
