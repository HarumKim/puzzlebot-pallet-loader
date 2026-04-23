
# Puzzlebot Pallet Loader - Mini Challenge 2

Este repositorio contiene la solución al **Mini Challenge 2**, el cual consiste en la simulación de la cinemática de un robot móvil de tracción diferencial (Puzzlebot) utilizando **ROS 2** y **RViz2**.

## Descripción del Desafío

El objetivo principal de este reto es lograr que el modelo URDF del robot describa una trayectoria circular de forma autónoma dentro del entorno de simulación de RViz2, calculando e integrando la cinemática del robot en tiempo real. 

Para lograr esto, se implementó un nodo personalizado en Python (`circular_movement.py`) que realiza las siguientes tareas a una frecuencia de 100 Hz (dt = 0.01s):
- **Cálculo de Cinemática Inversa:** A partir de una velocidad angular generalizada (0.1 rad/s) y un radio de giro (1.0 m), calcula la velocidad que debe tener cada rueda considerando las dimensiones físicas del robot (radio de rueda = 0.05m, distancia entre ruedas = 0.19m).
- **Integración de Posición:** Integra las velocidades angulares de cada rueda a lo largo del tiempo para calcular la posición de los eslabones continuos de las llantas.
- **Publicación de Transformaciones (TF):** Construye y envía los mensajes `TransformStamped` para actualizar el marco de referencia del robot (`base_footprint`) respecto al origen (`odom`) conforme avanza en la trayectoria circular.
- **Publicación de Joint States:** Publica los mensajes de `JointState` para `wheel_r_joint` y `wheel_l_joint` permitiendo visualizar el giro independiente de ambas ruedas.

## Resultado de la Simulación

<div align="center">
  <img src="https://github.com/user-attachments/assets/9f426788-c441-4b04-89bf-9296b4d261c1" width="50%" alt="Puzzlebot Simulation">
</div>

## ¿Cómo ejecutar la simulación?

1. Compila el paquete usando `colcon`:
   ```bash
   colcon build --packages-select mini_challenge2
   source install/setup.bash
   ```
2. Ejecuta el launch file principal:
   ```bash
   ros2 launch mini_challenge2 puzzlebot_launch.py
   ```
