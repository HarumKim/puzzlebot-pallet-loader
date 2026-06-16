# Web Interface — Puzzlebot Pallet Loader

Dashboard de monitoreo en tiempo real para el sistema de carga de pallets. Accesible desde cualquier navegador en la misma red.

---

## Vista previa

<div align="center">
<img src="https://github.com/user-attachments/assets/4863288a-7a01-4533-ade9-7d60b184fb2a" width="900"/>
</div>

---

## Qué muestra

| Panel | Fuente ROS 2 | Descripción |
|:---|:---|:---|
| Video en vivo | `/detection/annotated/compressed` | Cámara con anotaciones de QR o cámara limpia vía `camera_bridge` |
| Video YOLO | `/yolo/annotated/compressed` | Se activa automáticamente cuando la FSM entra en `YOLO_SCAN` o `NAV_TO_DROP` |
| Mapa MCL | `/map` + `/mcl/particles` + `/ekf_odom` | Mapa de ocupación con partículas (rojo) y pose del robot (verde) a 10 fps |
| Estado de la misión | `/fsm/state` | Etiqueta descriptiva del estado actual del FSM |
| Detecciones | `/detections` | Objetos detectados por YOLO (clase, confianza, bbox) |
| Estado de voz | `/voice/status` | Último comando de voz reconocido |

## Rutas HTTP

| Ruta | Descripción |
|:---|:---|
| `GET /` | Página principal (`result.html`) |
| `GET /video_feed` | Stream MJPEG de la cámara |
| `GET /map_feed` | Stream MJPEG del mapa MCL |
| `GET /api/detections` | JSON con las detecciones actuales |
| `GET /api/fsm_state` | Estado actual del FSM en texto legible |
| `GET /api/voice_status` | Último estado de voz |
| `GET /api/pose` | Pose actual del robot `{x, y, theta_deg}` |
| `POST /voice/start` | Envía comando `start` al nodo de voz |
| `POST /voice/stop` | Envía comando `stop` al nodo de voz |

---

## Estructura

```
web-interface/
├── FLASK-REST-Call-Linux/
│   ├── app.py           # Servidor Flask + nodo ROS 2 interno
│   ├── result.json
│   └── templates/
│       ├── result.html  # Página principal
│       ├── script.js    # Lógica del dashboard
│       └── style.css    # Estilos
└── CPP-LIB-Linux/
    └── src/             # Librería C++ de demostración
```

---

## Cómo correrlo

**Con Docker (recomendado):**

```bash
# Desde la raíz del repositorio
docker compose up
```

Abre en el navegador: `http://<IP-PC>:8002`

**Sin Docker:**

```bash
cd FLASK-REST-Call-Linux
python app.py
```

> Requiere ROS 2 Humble activo en el mismo entorno y los tópicos publicándose en la red.

---

## Tópicos suscritos

```
/detection/annotated/compressed   — imagen principal (CompressedImage)
/yolo/annotated/compressed        — imagen YOLO (CompressedImage)
/map                              — mapa de ocupación (OccupancyGrid)
/mcl/particles                    — partículas MCL (PoseArray)
/ekf_odom                         — odometría EKF (Odometry)
/detections                       — detecciones YOLO (String JSON)
/voice/status                     — estado de voz (String)
/fsm/state                        — estado del FSM (String)
```
