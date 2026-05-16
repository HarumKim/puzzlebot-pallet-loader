
# Puzzlebot Pallet Loader - Mini Challenge 5

Este repositorio contiene la solución al **Mini Challenge 5**, el cual consiste en la simulación del modelo cinemático de un robot móvil de tracción diferencial (Puzzlebot) con propagación de incertidumbre, utilizando **ROS 2** y **RViz2**.

## Descripción del Desafío

El objetivo principal de este reto es implementar un modelo cinemático realista que integre ruido en los encoders de las ruedas y propague la incertidumbre de la pose del robot a lo largo del tiempo.

Para lograr esto, se implementó un nodo personalizado en Python (`puzzlebot_sim.py`) que realiza las siguientes tareas a una frecuencia de 100 Hz (dt = 0.01s):
- **Cinemática Inversa con Ruido:** A partir de los comandos de velocidad (`cmd_vel`), calcula las velocidades ideales de cada rueda y les añade ruido gaussiano proporcional a la velocidad, simulando el comportamiento real de los encoders.
- **Integración de Posición (Euler):** Reconstruye las velocidades lineales y angulares del robot a partir de las velocidades ruidosas de las ruedas e integra la pose (`x`, `y`, `θ`) en el tiempo.
- **Propagación de Covarianza:** Mantiene y actualiza una matriz de covarianza 3×3 para la pose `[x, y, θ]` usando el jacobiano linealizado del modelo de movimiento y una matriz de ruido de proceso `Q`, parametrizada por las ganancias `k_linear`, `k_angular` y `k_drift`.
- **Publicación de Odometría:** Publica la pose simulada con su covarianza en el tópico `pose_sim` como mensaje `nav_msgs/Odometry`.
- **Publicación de Velocidades de Rueda:** Publica las velocidades ruidosas `wr` y `wl` como `std_msgs/Float32`.
- **Publicación de Transformaciones (TF):** Transmite la transformación `odom → base_footprint` en tiempo real para visualización en RViz2.

## Resultado de la Simulación

<!-- Inserta aquí la imagen o GIF del resultado (e.g., captura de RViz2 mostrando la trayectoria y la elipse de covarianza) -->
<div align="center">
  <img src="" width="50%" alt="Puzzlebot Mini Challenge 5 Simulation">
</div>

## ¿Cómo ejecutar la simulación?

1. Compila el paquete usando `colcon`:
   ```bash
   colcon build --packages-select mini_challenge5
   source install/setup.bash
   ```
2. Ejecuta el launch file principal:
   ```bash
   ros2 launch mini_challenge5 mini_challenge5.launch.py
   ```
3. Opcionalmente, ajusta los parámetros del modelo al lanzar:
   ```bash
   ros2 launch mini_challenge5 mini_challenge5.launch.py \
     wheel_radius:=0.05 \
     wheel_base:=0.19 \
     noise_gain_right:=0.016 \
     noise_gain_left:=0.016 \
     k_linear:=0.08 \
     k_angular:=0.08 \
     k_drift:=0.04
   ```
