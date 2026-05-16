
# Puzzlebot Pallet Loader - Mini Challenge 6

Este repositorio contiene la solución al **Mini Challenge 6**, el cual consiste en la navegación autónoma con evasión de obstáculos de un robot móvil Puzzlebot utilizando los algoritmos **Bug0** y **Bug2** dentro de un entorno de simulación en **Gazebo** con **ROS 2**.

## Descripción del Desafío

El objetivo principal de este reto es lograr que el robot navegue desde su posición inicial hasta un goal marker esquivando obstáculos estáticos, en cuatro mapas distintos de dificultad progresiva.

Se implementaron dos nodos de navegación reactiva en Python, ambos usando lectura de LiDAR (`/scan`) y odometría ground truth (`/ground_truth`):

### Bug0 (`bug0_node.py`)
- **Ir al Goal:** El robot avanza en línea recta hacia el waypoint objetivo calculando el error de orientación y distancia.
- **Seguimiento de Pared (derecha):** Al detectar un obstáculo frontal (< 0.40 m), activa el modo wall following de mano derecha, manteniendo una distancia lateral objetivo de 0.5 m.
- **Detección de Esquina Exterior:** Identifica esquinas exteriores con el sensor derecho (`-135°` a `-50°`) y el sensor diagonal frontal-derecho (`-60°` a `-30°`), avanzando recto antes de girar para evitar que las llantas rocen la pared.
- **Salida del Modo Pared:** Regresa a Go-to-Goal cuando el frente está despejado (> 0.85 m), el heading hacia el goal está dentro de ±45° y ha recorrido al menos 0.50 m pegado a la pared.

### Bug2 (`bug2_node.py`)
- **M-line:** Define la línea recta entre la posición en que golpeó el obstáculo y el goal. Sale del modo pared únicamente cuando vuelve a cruzar esa línea más cerca del objetivo que cuando entró.
- **Salida por Camino Libre:** Además de la M-line, sale del modo pared si el frente está despejado (> 0.85 m), la orientación hacia el goal es correcta (±45°) y el robot está más cerca del objetivo que al entrar.
- **Detección de Atascamiento:** Si el robot no se desplaza más de 5 cm en 2 segundos consecutivos dentro del modo pared, ejecuta un giro a la izquierda para desengancharse de la pared sin detección del LiDAR.
- **Seguimiento de Pared y Esquinas:** Comparte la misma lógica de wall following, detección de esquinas y corner clearance que Bug0.

Ambos nodos leen el waypoint objetivo y la tolerancia de llegada desde un archivo YAML por mapa:

| Mapa | Modelo goal | Posición (x, y) | `goal_tolerance` |
|------|-------------|-----------------|-----------------|
| obstacle_avoidance_1 | goal_marker_5 | (1.45, 1.2) | 0.20 m |
| obstacle_avoidance_2 | goal_marker_3 | (-1.2, 1.5) | 0.20 m |
| obstacle_avoidance_3 | goal_marker_2 | (0.0, -2.5) | 0.20 m |
| obstacle_avoidance_4 | goal_marker_1 | (0.0, -2.45) | 0.20 m |

## Resultado de la Simulación

<img width="1252" height="871" alt="minichallenge6" src="https://github.com/user-attachments/assets/96fc90df-4ca7-40a9-acab-c1588775be36" />

## ¿Cómo ejecutar la simulación?

1. Compila los paquetes usando `colcon`:
   ```bash
   colcon build --packages-select mini_challenge6 puzzlebot_gazebo
   source install/setup.bash
   ```

2. Selecciona el mapa en `bringup_simulation_simple_launch.py` y lanza la simulación en Gazebo:
   ```bash
   ros2 launch puzzlebot_gazebo bringup_simulation_simple_launch.py
   ```

3. En otra terminal, lanza el nodo de navegación indicando el mismo mapa:
   ```bash
   # Con Bug0:
   ros2 launch mini_challenge6 bug0_launch.py world:=obstacle_avoidance_1.world

   # Con Bug2:
   ros2 launch mini_challenge6 bug2_launch.py world:=obstacle_avoidance_1.world
   ```
   > Sustituye `obstacle_avoidance_1.world` por el mapa correspondiente (`_2`, `_3` o `_4`). El world del launch de Gazebo y el argumento `world:=` deben coincidir.
