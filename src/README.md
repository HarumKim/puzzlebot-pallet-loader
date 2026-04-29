# Mini Challenge 3 - Puzzlebot Simulation and Point Stabilisation

## Descripción general

Este paquete implementa una simulación cinemática del robot diferencial Puzzlebot usando ROS 2. El sistema permite simular el movimiento del robot a partir de comandos de velocidad, calcular su odometría mediante dead reckoning, visualizar el robot en RViz y controlarlo automáticamente para llegar a un punto objetivo.

El flujo general del sistema es:

```text
/cmd_vel → puzzlebot_sim → /wr /wl
/wr /wl → localisation → /odom
/odom → joint_state_pub → /tf + /joint_states
/odom → point_stabilisation_control → /cmd_vel