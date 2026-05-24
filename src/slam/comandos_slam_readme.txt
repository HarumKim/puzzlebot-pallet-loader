# SLAM Project — Puzzlebot + LiDAR + RViz

Este documento describe los pasos necesarios para ejecutar el proyecto de SLAM con Puzzlebot, LiDAR, micro-ROS y visualización en RViz.

---

## 1. Conectarse a la Jetson por SSH

Desde tu computadora:

```bash
ssh puzzlebot@10.42.0.1
```

---

## 2. Terminal 1 — Activar micro-ROS en la Jetson

Dentro de la Jetson:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_packages_ws/install/setup.bash
colcon build
source install/setup.bash
```

Lanzar el agente de micro-ROS:

```bash
ros2 launch puzzlebot_ros micro_ros_agent.launch.py
```

---

## 3. Terminal 2 — Activar comunicación con el LiDAR en la Jetson

Abre otra terminal y accede nuevamente a la Jetson por SSH:

```bash
ssh puzzlebot@10.42.0.1
```

Dentro de la Jetson:

```bash
cd ~/ros2_lidar_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Dar permisos al puerto del LiDAR:

```bash
sudo chmod 666 /dev/ttyUSB0
```

Lanzar el nodo del LiDAR:

```bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0
```

---

## 4. Nota importante sobre los puertos USB

El puerto del LiDAR puede cambiar entre ejecuciones. Para revisar los puertos disponibles:

```bash
ls /dev/ttyUSB*
```

Una forma práctica de identificar el puerto correcto es desconectar y volver a conectar el LiDAR, revisando qué puerto desaparece y vuelve a aparecer.

Si el LiDAR aparece en otro puerto, por ejemplo `/dev/ttyUSB1`, cambia los comandos anteriores:

```bash
sudo chmod 666 /dev/ttyUSB1
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB1
```

También se recomienda revisar los nombres persistentes de los dispositivos:

```bash
ls -l /dev/serial/by-id/
```

Esto ayuda a evitar conflictos entre el LiDAR y micro-ROS cuando Linux reasigna dinámicamente `/dev/ttyUSB0` y `/dev/ttyUSB1`. Sin embargo, no debería haber mayor problema ya que el agente de microros se lanza con parámetro directo con el launch file descrito antes:

```python
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=["serial", "-D", "/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_302c79a72bffec1196d26f508ce70331-if00-port0"],
        parameters=[],
        output='screen'
    )
```

---

## 5. Sincronizar timestamps entre la computadora y la Jetson

Desde tu computadora:

```bash
ssh -t puzzlebot@10.42.0.1 "sudo date -s @$(date +%s)"
```

Este comando sincroniza la hora de la Jetson con la hora de tu computadora. Esto ayuda a evitar problemas con timestamps en ROS 2, sensores, logs, TF y visualización.

---

## 6. Terminal 3 — Ejecutar el paquete SLAM con RViz en tu computadora

Desde tu computadora, entra al repositorio actualizado:

```bash
cd ~/puzzlebot-pallet-loader
source /opt/ros/humble/setup.bash
colcon build --packages-select slam
source install/setup.bash
```

Lanzar los nodos de SLAM y RViz:

```bash
ros2 launch slam slam_rviz_launch.py
```

Este launch ejecuta los nodos principales del paquete `slam`, incluyendo odometría, localización por Monte Carlo, mapeo y visualización en RViz.

---

## 7. Flujo resumido de ejecución

1. Entrar por SSH a la Jetson.
2. En la Jetson, lanzar `micro_ros_agent`.
3. En otra terminal de la Jetson, lanzar el LiDAR.
4. Desde tu computadora, sincronizar la hora con la Jetson.
5. En tu computadora, lanzar el paquete `slam` con RViz.

---

## 8. Troubleshooting rápido

### El LiDAR no inicia

Revisa qué puerto está usando:

```bash
ls /dev/ttyUSB*
```

También puedes revisar los mensajes del sistema:

```bash
dmesg | grep -i ttyUSB | tail -20
```

### El puerto está ocupado

Revisa qué proceso está usando el puerto:

```bash
sudo lsof /dev/ttyUSB0
```

Si el puerto correcto es otro, cambia `/dev/ttyUSB0` por el puerto correspondiente.

### RViz no recibe datos

Verifica que los tópicos estén activos:

```bash
ros2 topic list
```

Para revisar si el LiDAR está publicando:

```bash
ros2 topic echo /scan --once
```

### micro-ROS y LiDAR se empalman en el mismo puerto

Revisa los nombres persistentes:

```bash
ls -l /dev/serial/by-id/
```

Después, usa la ruta persistente correspondiente para cada dispositivo en lugar de depender de `/dev/ttyUSB0` o `/dev/ttyUSB1`.
