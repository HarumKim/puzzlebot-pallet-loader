# Puzzlebot Pallet Loader - Mini Challenge 3

Este repositorio contiene la solución al **Mini Challenge 3**, el cual consiste en la simulación cinemática y la estabilización en un punto de un robot móvil de tracción diferencial (Puzzlebot) utilizando **ROS 2** y **RViz2**.

## Descripción del Desafío

El objetivo principal de este reto es simular el movimiento del robot a partir de comandos de velocidad, calcular su odometría mediante *dead reckoning* y controlarlo de forma autónoma para que alcance un punto objetivo predefinido dentro del entorno de simulación.

Para lograr esto, el flujo del sistema se divide en los siguientes procesos:
- **Simulación Cinemática:** Convierte los comandos de velocidad del robot (`/cmd_vel`) en velocidades angulares individuales para cada rueda (`/wr`, `/wl`).
- **Localización (Odometría):** Integra las velocidades de las ruedas (`/wr`, `/wl`) para estimar la posición y orientación actual del robot en el espacio, publicándola como odometría (`/odom`).
- **Publicación de Transformaciones (TF) y Joint States:** Utiliza la odometría calculada (`/odom`) para actualizar el marco de referencia del robot (`/tf`) y el estado de rotación de las ruedas (`/joint_states`), permitiendo la visualización precisa en RViz.
- **Control de Estabilización:** Toma la posición actual del robot (`/odom`) y calcula los comandos de velocidad necesarios (`/cmd_vel`) para corregir la trayectoria y dirigir al robot hacia el punto objetivo.

## Resultado de la Simulación

<div align="center">
  <img src="https://github.com/user-attachments/assets/36cb2bd5-9e89-4d6d-afbd-19a6c4461539" width="50%" alt="Puzzlebot Simulation Mini Challenge 3">
</div>

## ¿Cómo ejecutar la simulación?

1. Entra a tu workspace y compila el paquete usando `colcon`:
   
```bash
   cd ~/ros2_ws/puzzlebot-pallet-loader
   colcon build --packages-select mini_challenge3
   source install/setup.bash
