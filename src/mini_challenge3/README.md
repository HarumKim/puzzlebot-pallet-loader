# Mini Challenge 3 - Puzzlebot Simulation and Point Stabilisation

## Descripción general

Este paquete implementa una simulación cinemática del robot diferencial Puzzlebot usando ROS 2. El sistema permite simular el movimiento del robot a partir de comandos de velocidad, calcular su odometría mediante dead reckoning, visualizar el robot en RViz y controlarlo automáticamente para llegar a un punto objetivo.

El flujo general del sistema es:

```text
/cmd_vel → puzzlebot_sim → /wr /wl
/wr /wl → localisation → /odom
/odom → joint_state_pub → /tf + /joint_states
/odom → point_stabilisation_control → /cmd_vel

## Cómo correr el mini challenge

### 1. Entrar al workspace

```bash
cd ~/ros2_ws/puzzlebot-pallet-loader
colcon build --packages-select puzzlebot_sim
source install/setup.bash
ros2 launch puzzlebot_sim puzzlebot_sim_launch.py

<img width="1225" height="907" alt="image" src="https://github.com/user-attachments/assets/36cb2bd5-9e89-4d6d-afbd-19a6c4461539" />
