# """
# EKF-SLAM Node — Localización y Mapeo Simultáneo con Filtro de Kalman Extendido

# ¿Qué hace?
#     Implementa EKF-SLAM completo desde cero. El robot NO necesita conocer
#     el mapa de antemano. Descubre landmarks (esquinas de paredes) por sí solo
#     y los usa para corregir su posición en tiempo real.

# ¿Cómo funciona el ciclo completo?
#     1. PREDICCIÓN  — usa encoders para mover la pose estimada (igual que odometry.py)
#                      y propaga la incertidumbre con el Jacobiano ∇g
#     2. DETECCIÓN   — extrae esquinas del escaneo LiDAR con Split-and-Merge
#     3. ASOCIACIÓN  — para cada esquina detectada, decide si es un landmark ya
#                      conocido o uno nuevo, usando distancia de Mahalanobis
#     4. CORRECCIÓN  — actualiza pose Y posiciones de landmarks con el update EKF
#     5. PUBLICACIÓN — publica /odom con covarianza y TF odom→base_link

# ¿Por qué es mejor que el ekf_node.py anterior?
#     ekf_node.py anterior:  necesitaba las posiciones de landmarks hardcodeadas
#     EKF-SLAM (este):       descubre landmarks automáticamente en tiempo real
#                            el estado crece: [x, y, θ, lx1, ly1, lx2, ly2, ...]

# Estado del filtro:
#     μ = [x, y, θ, lx_0, ly_0, lx_1, ly_1, ..., lx_N, ly_N]
#     Σ = matriz de covarianza (3+2N) × (3+2N)

# Suscribe:
#     /VelocityEncL  (std_msgs/Float32)
#     /VelocityEncR  (std_msgs/Float32)
#     /scan          (sensor_msgs/LaserScan)

# Publica:
#     /odom          (nav_msgs/Odometry) — pose con covarianza
#     /landmarks     (visualization_msgs/MarkerArray) — landmarks descubiertos en RViz
#     TF dinámico:  odom → base_link
#     TF dinámico:  map  → odom        (corrección SLAM)
#     TF estático:  base_link → laser  (LiDAR rotado 180°)
# """

# import math
# import numpy as np

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# from std_msgs.msg import Float32, ColorRGBA
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Quaternion, TransformStamped, Point
# from visualization_msgs.msg import Marker, MarkerArray
# from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


# # ─────────────────────────────────────────────────────────────────────
# #  PARÁMETROS EKF-SLAM
# #  Ajusta estos valores si el mapa queda mal
# # ─────────────────────────────────────────────────────────────────────

# # Ruido de proceso Q — qué tanto confías en los encoders
# # Más grande = menos confianza en encoders = más peso al LiDAR
# Q_X     = 0.01    # varianza en x (m²)
# Q_Y     = 0.01    # varianza en y (m²)
# Q_THETA = 0.005   # varianza en θ (rad²)

# # Ruido de medición R — qué tanto confías en el LiDAR
# # Más grande = menos confianza en LiDAR
# R_DIST  = 0.05    # varianza en distancia al landmark (m²)
# R_ANGLE = 0.05    # varianza en ángulo al landmark (rad²)

# # Data association — umbral de Mahalanobis
# # Más alto = más permisivo para asociar con landmarks existentes
# # Más bajo = crea landmarks nuevos más fácilmente
# MAHAL_THRESHOLD_ASSOC = 2.0   # asociar con landmark existente si d < este valor
# MAHAL_THRESHOLD_NEW   = 5.0   # crear landmark nuevo si d > este valor

# # Detector de esquinas
# SEG_DIST_THRESHOLD = 0.15     # distancia max entre puntos del mismo segmento (m)
# MIN_POINTS_SEG     = 5        # mínimo de puntos para considerar un segmento
# MAX_LINE_ERROR     = 0.05     # error máximo de ajuste lineal (m)
# MIN_CORNER_ANGLE   = math.pi / 4   # ángulo mínimo para considerar esquina (45°)
# MAX_CORNER_ANGLE   = 3 * math.pi / 4  # ángulo máximo (135°)


# # ─────────────────────────────────────────────────────────────────────
# #  Utilidades matemáticas
# # ─────────────────────────────────────────────────────────────────────
# def yaw_to_quaternion(yaw):
#     return Quaternion(
#         x=0.0, y=0.0,
#         z=math.sin(yaw / 2.0),
#         w=math.cos(yaw / 2.0)
#     )


# def normalize_angle(a):
#     return math.atan2(math.sin(a), math.cos(a))


# # ─────────────────────────────────────────────────────────────────────
# #  Detector de esquinas con LiDAR — Split and Merge
# #
# #  Proceso:
# #  1. Polar → cartesiano
# #  2. Segmentación por distancia entre puntos consecutivos
# #  3. Ajuste de línea por mínimos cuadrados (SVD)
# #  4. Intersección de segmentos adyacentes con ángulo ~90° → esquina
# #
# #  Retorna: lista de (x, y) en frame del robot
# # ─────────────────────────────────────────────────────────────────────
# def detect_corners(scan_msg):
#     # Paso 1: polar → cartesiano
#     points = []
#     angle = scan_msg.angle_min
#     for r in scan_msg.ranges:
#         if scan_msg.range_min < r < scan_msg.range_max:
#             points.append((r * math.cos(angle), r * math.sin(angle)))
#         angle += scan_msg.angle_increment

#     if len(points) < MIN_POINTS_SEG * 2:
#         return []

#     # Paso 2: segmentación
#     segments = []
#     seg = [points[0]]
#     for i in range(1, len(points)):
#         dx = points[i][0] - points[i-1][0]
#         dy = points[i][1] - points[i-1][1]
#         if math.sqrt(dx*dx + dy*dy) < SEG_DIST_THRESHOLD:
#             seg.append(points[i])
#         else:
#             if len(seg) >= MIN_POINTS_SEG:
#                 segments.append(seg)
#             seg = [points[i]]
#     if len(seg) >= MIN_POINTS_SEG:
#         segments.append(seg)

#     if len(segments) < 2:
#         return []

#     # Paso 3: ajuste de línea por SVD a cada segmento → (a, b, c) con ax+by=c
#     def fit_line(seg):
#         xs = np.array([p[0] for p in seg])
#         ys = np.array([p[1] for p in seg])
#         cx, cy = xs.mean(), ys.mean()
#         M = np.column_stack([xs - cx, ys - cy])
#         _, _, Vt = np.linalg.svd(M)
#         a, b = Vt[1]           # normal a la línea
#         c = a * cx + b * cy
#         err = np.mean([abs(a*p[0] + b*p[1] - c) for p in seg])
#         return a, b, c, err

#     line_params = [fit_line(s) for s in segments]

#     # Paso 4: intersecciones entre segmentos adyacentes → esquinas
#     corners = []
#     for i in range(len(line_params) - 1):
#         a1, b1, c1, e1 = line_params[i]
#         a2, b2, c2, e2 = line_params[i + 1]

#         if e1 > MAX_LINE_ERROR or e2 > MAX_LINE_ERROR:
#             continue

#         det = a1 * b2 - a2 * b1
#         if abs(det) < 1e-6:
#             continue

#         # Ángulo entre normales de los dos segmentos
#         dot   = a1*a2 + b1*b2
#         cross = abs(a1*b2 - a2*b1)
#         angle_between = math.atan2(cross, abs(dot))

#         if not (MIN_CORNER_ANGLE < angle_between < MAX_CORNER_ANGLE):
#             continue

#         # Punto de intersección
#         ix = (c1 * b2 - c2 * b1) / det
#         iy = (a1 * c2 - a2 * c1) / det

#         dist = math.sqrt(ix*ix + iy*iy)
#         if dist < scan_msg.range_max:
#             corners.append((ix, iy))

#     return corners


# # ─────────────────────────────────────────────────────────────────────
# #  Nodo EKF-SLAM
# # ─────────────────────────────────────────────────────────────────────
# class EKFSLAMNode(Node):

#     def __init__(self):
#         super().__init__('ekf_slam')

#         # ── Parámetros del robot ──────────────────────────────────────
#         self.r  = 0.05   # radio de rueda (m)
#         self.L  = 0.19   # distancia entre ruedas (m)
#         self.dt = 0.05   # periodo de integración (s)

#         # ── Estado EKF-SLAM ───────────────────────────────────────────
#         # μ = [x, y, θ]  (crece al descubrir landmarks: +2 por cada uno)
#         self.mu = np.array([0.0, 0.0, 0.0])

#         # Σ = covarianza 3×3 (crece igual que μ)
#         # Robot empieza con pose conocida → 0
#         # Landmarks empiezan con incertidumbre infinita → se llena al agregar
#         self.Sigma = np.zeros((3, 3))

#         # ── Ruidos ───────────────────────────────────────────────────
#         self.Q = np.diag([Q_X, Q_Y, Q_THETA])
#         self.R = np.diag([R_DIST, R_ANGLE])

#         # ── Landmarks descubiertos ───────────────────────────────────
#         # n_landmarks: cuántos landmarks conocemos hasta ahora
#         self.n_landmarks = 0

#         # ── Encoders ─────────────────────────────────────────────────
#         self.w_l = 0.0
#         self.w_r = 0.0

#         # ── QoS ──────────────────────────────────────────────────────
#         qos_enc = QoSProfile(depth=10)
#         qos_enc.reliability = QoSReliabilityPolicy.BEST_EFFORT
#         qos_scan = QoSProfile(depth=5)
#         qos_scan.reliability = QoSReliabilityPolicy.BEST_EFFORT

#         # ── Suscripciones ────────────────────────────────────────────
#         self.create_subscription(Float32,   '/VelocityEncL', self.left_cb,  qos_enc)
#         self.create_subscription(Float32,   '/VelocityEncR', self.right_cb, qos_enc)
#         self.create_subscription(LaserScan, '/scan',         self.scan_cb,  qos_scan)

#         # ── Publicadores ─────────────────────────────────────────────
#         self.odom_pub  = self.create_publisher(Odometry,    '/odom',      10)
#         self.lm_pub    = self.create_publisher(MarkerArray, '/landmarks', 10)
#         self.tf_br     = TransformBroadcaster(self)
#         self.static_br = StaticTransformBroadcaster(self)

#         # TF estático base_link → laser (LiDAR rotado 180° en yaw)
#         self._publish_laser_tf()

#         # Corrección map → odom que calcula el EKF
#         # Al inicio es identidad; se actualiza con cada corrección
#         self.map_correction_x   = 0.0
#         self.map_correction_y   = 0.0
#         self.map_correction_yaw = 0.0

#         # ── Timer de predicción ───────────────────────────────────────
#         self.create_timer(self.dt, self.predict_and_publish)

#         self.get_logger().info('🧠 EKF-SLAM node iniciado — descubrimiento automático de landmarks')

#     # ─────────────────────────────────────────────────────────────────
#     #  Callbacks de encoders
#     # ─────────────────────────────────────────────────────────────────
#     def left_cb(self, msg):
#         self.w_l = float(msg.data)

#     def right_cb(self, msg):
#         self.w_r = float(msg.data)

#     # ─────────────────────────────────────────────────────────────────
#     #  ETAPA 1: PREDICCIÓN
#     #
#     #  Igual que odometry.py en cuanto a mover la pose.
#     #  NUEVO: propaga la covarianza completa del estado (robot + landmarks)
#     #
#     #  El truco del EKF-SLAM es que los landmarks NO se mueven,
#     #  entonces solo la parte del robot en Σ crece durante la predicción.
#     # ─────────────────────────────────────────────────────────────────
#     def predict_and_publish(self):
#         v = self.r * (self.w_r + self.w_l) / 2.0
#         w = self.r * (self.w_r - self.w_l) / self.L

#         x, y, theta = self.mu[0], self.mu[1], self.mu[2]
#         T = v * self.dt  # desplazamiento lineal en este paso

#         # Actualizar pose del robot en μ
#         self.mu[0] = x + T * math.cos(theta)
#         self.mu[1] = y + T * math.sin(theta)
#         self.mu[2] = normalize_angle(theta + w * self.dt)

#         # Jacobiano ∇g del movimiento (solo parte del robot 3×3)
#         grad_g = np.array([
#             [1.0, 0.0, -T * math.sin(theta)],
#             [0.0, 1.0,  T * math.cos(theta)],
#             [0.0, 0.0,  1.0]
#         ])

#         # Propagar covarianza completa
#         # Solo la esquina robot-robot de Σ se ve afectada por el movimiento
#         # Las correlaciones robot-landmark también se actualizan
#         n = 3 + 2 * self.n_landmarks

#         # Matriz F_x: expande grad_g al tamaño completo del estado
#         # [ grad_g  |  0  ]
#         # [   0     |  I  ]
#         F_x = np.zeros((n, n))
#         F_x[:3, :3] = grad_g
#         if self.n_landmarks > 0:
#             F_x[3:, 3:] = np.eye(2 * self.n_landmarks)

#         # Matriz de ruido de proceso expandida
#         Q_full = np.zeros((n, n))
#         Q_full[:3, :3] = self.Q

#         # Σ_pred = F_x · Σ · F_xᵀ + Q_full
#         self.Sigma = F_x @ self.Sigma @ F_x.T + Q_full

#         self._publish(v, w)

#     # ─────────────────────────────────────────────────────────────────
#     #  ETAPA 2 + 3 + 4: DETECCIÓN → ASOCIACIÓN → CORRECCIÓN
#     #
#     #  Se ejecuta con cada escaneo LiDAR.
#     #
#     #  Para cada esquina detectada:
#     #    a) Calcular distancia de Mahalanobis a cada landmark conocido
#     #    b) Si la distancia mínima < MAHAL_THRESHOLD_ASSOC → asociar
#     #       Si la distancia mínima > MAHAL_THRESHOLD_NEW  → nuevo landmark
#     #    c) Realizar update EKF con el landmark asociado
#     # ─────────────────────────────────────────────────────────────────
#     def scan_cb(self, scan_msg):
#         corners = detect_corners(scan_msg)
#         if not corners:
#             return

#         x, y, theta = self.mu[0], self.mu[1], self.mu[2]
#         cos_t, sin_t = math.cos(theta), math.sin(theta)

#         for (cx_r, cy_r) in corners:

#             # Transformar esquina de frame robot → frame mundo
#             cx_w = x + cx_r * cos_t - cy_r * sin_t
#             cy_w = y + cx_r * sin_t + cy_r * cos_t

#             # ── Data Association con Mahalanobis ──────────────────────
#             #
#             # Para cada landmark j conocido, calcular:
#             #   d_j = (z - h_j)ᵀ · S_j⁻¹ · (z - h_j)
#             # donde S_j = ∇h_j · Σ · ∇h_jᵀ + R
#             #
#             # Si min(d_j) < MAHAL_THRESHOLD_ASSOC → asociar con j*
#             # Si min(d_j) > MAHAL_THRESHOLD_NEW   → nuevo landmark
#             # Entre los dos umbrales → ambiguo, ignorar
#             # ─────────────────────────────────────────────────────────

#             best_j    = -1         # índice del landmark más cercano
#             best_dist = float('inf')

#             for j in range(self.n_landmarks):
#                 # Posición del landmark j en μ
#                 lx = self.mu[3 + 2*j]
#                 ly = self.mu[3 + 2*j + 1]

#                 # Diferencias
#                 dx = lx - x
#                 dy = ly - y
#                 p  = dx**2 + dy**2
#                 q  = math.sqrt(p)

#                 if q < 1e-6:
#                     continue

#                 # Medición esperada h(μ, j)
#                 h_pred = np.array([
#                     q,
#                     normalize_angle(math.atan2(dy, dx) - theta)
#                 ])

#                 # Medición real z (distancia y ángulo a la esquina detectada)
#                 z = np.array([
#                     math.sqrt(cx_r**2 + cy_r**2),
#                     normalize_angle(math.atan2(cy_r, cx_r))
#                 ])

#                 # Jacobiano ∇h_j expandido al tamaño completo del estado
#                 # ∂h/∂[x, y, θ, ..., lx_j, ly_j, ...]
#                 n = 3 + 2 * self.n_landmarks
#                 grad_h = np.zeros((2, n))

#                 # Derivadas respecto a la pose del robot
#                 grad_h[0, 0] = -dx / q
#                 grad_h[0, 1] = -dy / q
#                 grad_h[0, 2] =  0.0
#                 grad_h[1, 0] =  dy / p
#                 grad_h[1, 1] = -dx / p
#                 grad_h[1, 2] = -1.0

#                 # Derivadas respecto al landmark j
#                 grad_h[0, 3 + 2*j]     =  dx / q
#                 grad_h[0, 3 + 2*j + 1] =  dy / q
#                 grad_h[1, 3 + 2*j]     = -dy / p
#                 grad_h[1, 3 + 2*j + 1] =  dx / p

#                 # Covarianza de la innovación S_j
#                 S_j = grad_h @ self.Sigma @ grad_h.T + self.R

#                 # Innovación
#                 innov = z - h_pred
#                 innov[1] = normalize_angle(innov[1])

#                 # Distancia de Mahalanobis
#                 try:
#                     d_mah = float(innov.T @ np.linalg.inv(S_j) @ innov)
#                 except np.linalg.LinAlgError:
#                     continue

#                 if d_mah < best_dist:
#                     best_dist = d_mah
#                     best_j    = j

#             # ── Decisión de asociación ────────────────────────────────
#             if best_dist < MAHAL_THRESHOLD_ASSOC:
#                 # Asociar con landmark existente best_j
#                 j_use = best_j

#             elif best_dist > MAHAL_THRESHOLD_NEW:
#                 # Nuevo landmark — agregar al estado
#                 j_use = self.n_landmarks
#                 self._add_landmark(cx_w, cy_w)

#             else:
#                 # Zona ambigua — ignorar esta medición
#                 continue

#             # ── Update EKF con landmark j_use ─────────────────────────
#             self._ekf_update(j_use, cx_r, cy_r)

#         # Publicar landmarks en RViz
#         self._publish_landmarks()

#     # ─────────────────────────────────────────────────────────────────
#     #  Agregar nuevo landmark al estado
#     #
#     #  Cuando se descubre una esquina nueva:
#     #  1. Añadir su posición estimada a μ
#     #  2. Expandir Σ con alta incertidumbre inicial para el nuevo landmark
#     #     y ceros en las correlaciones cruzadas con el robot y otros landmarks
#     # ─────────────────────────────────────────────────────────────────
#     def _add_landmark(self, lx_w, ly_w):
#         # Expandir μ
#         self.mu = np.append(self.mu, [lx_w, ly_w])

#         # Expandir Σ — nuevo bloque de 2×2 con alta incertidumbre
#         n_old = 3 + 2 * self.n_landmarks
#         n_new = n_old + 2

#         Sigma_new = np.zeros((n_new, n_new))
#         Sigma_new[:n_old, :n_old] = self.Sigma

#         # Alta incertidumbre inicial en posición del nuevo landmark
#         Sigma_new[n_old,     n_old]     = 1e6
#         Sigma_new[n_old + 1, n_old + 1] = 1e6

#         self.Sigma = Sigma_new
#         self.n_landmarks += 1

#         self.get_logger().info(
#             f'🔵 Nuevo landmark #{self.n_landmarks} en ({lx_w:.2f}, {ly_w:.2f})')

#     # ─────────────────────────────────────────────────────────────────
#     #  Update EKF con landmark j
#     #
#     #  Fórmulas estándar del EKF:
#     #    K = Σ · ∇hᵀ · (∇h · Σ · ∇hᵀ + R)⁻¹
#     #    μ = μ + K · (z - h(μ))
#     #    Σ = (I - K · ∇h) · Σ
#     # ─────────────────────────────────────────────────────────────────
#     def _ekf_update(self, j, cx_r, cy_r):
#         x, y, theta = self.mu[0], self.mu[1], self.mu[2]
#         lx = self.mu[3 + 2*j]
#         ly = self.mu[3 + 2*j + 1]

#         dx = lx - x
#         dy = ly - y
#         p  = dx**2 + dy**2
#         q  = math.sqrt(p)

#         if q < 1e-6:
#             return

#         # Medición esperada
#         h_pred = np.array([
#             q,
#             normalize_angle(math.atan2(dy, dx) - theta)
#         ])

#         # Medición real
#         z = np.array([
#             math.sqrt(cx_r**2 + cy_r**2),
#             normalize_angle(math.atan2(cy_r, cx_r))
#         ])

#         # Innovación
#         innov    = z - h_pred
#         innov[1] = normalize_angle(innov[1])

#         # Jacobiano ∇h expandido
#         n = 3 + 2 * self.n_landmarks
#         grad_h = np.zeros((2, n))

#         grad_h[0, 0] = -dx / q
#         grad_h[0, 1] = -dy / q
#         grad_h[0, 2] =  0.0
#         grad_h[1, 0] =  dy / p
#         grad_h[1, 1] = -dx / p
#         grad_h[1, 2] = -1.0

#         grad_h[0, 3 + 2*j]     =  dx / q
#         grad_h[0, 3 + 2*j + 1] =  dy / q
#         grad_h[1, 3 + 2*j]     = -dy / p
#         grad_h[1, 3 + 2*j + 1] =  dx / p

#         # Covarianza de innovación
#         S = grad_h @ self.Sigma @ grad_h.T + self.R

#         # Ganancia de Kalman
#         try:
#             K = self.Sigma @ grad_h.T @ np.linalg.inv(S)
#         except np.linalg.LinAlgError:
#             return

#         # Actualizar estado completo (robot + todos los landmarks)
#         delta    = K @ innov
#         self.mu += delta
#         self.mu[2] = normalize_angle(self.mu[2])

#         # Actualizar covarianza
#         I          = np.eye(n)
#         self.Sigma = (I - K @ grad_h) @ self.Sigma

#         # Actualizar corrección map → odom
#         # La corrección es la diferencia acumulada entre la pose corregida (EKF)
#         # y la pose integrada por odometría pura
#         # Como mu ya incluye la corrección, la dejamos reflejada en map→odom
#         self.map_correction_x   = 0.0   # simplificación: map y odom comparten origen
#         self.map_correction_y   = 0.0   # el EKF corrige en odom directamente
#         self.map_correction_yaw = 0.0

#     # ─────────────────────────────────────────────────────────────────
#     #  Publicar /odom y TF
#     # ─────────────────────────────────────────────────────────────────
#     def _publish(self, v, w):
#         now = self.get_clock().now().to_msg()
#         q   = yaw_to_quaternion(self.mu[2])

#         odom = Odometry()
#         odom.header.stamp    = now
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id  = 'base_link'

#         odom.pose.pose.position.x  = self.mu[0]
#         odom.pose.pose.position.y  = self.mu[1]
#         odom.pose.pose.position.z  = 0.0
#         odom.pose.pose.orientation = q

#         odom.twist.twist.linear.x  = v
#         odom.twist.twist.angular.z = w

#         # Covarianza de la pose (parte robot de Σ)
#         odom.pose.covariance[0]  = self.Sigma[0, 0]
#         odom.pose.covariance[1]  = self.Sigma[0, 1]
#         odom.pose.covariance[6]  = self.Sigma[1, 0]
#         odom.pose.covariance[7]  = self.Sigma[1, 1]
#         odom.pose.covariance[35] = self.Sigma[2, 2]

#         self.odom_pub.publish(odom)

#         # TF dinámico: odom → base_link
#         tf_odom = TransformStamped()
#         tf_odom.header.stamp    = now
#         tf_odom.header.frame_id = 'odom'
#         tf_odom.child_frame_id  = 'base_link'

#         tf_odom.transform.translation.x = self.mu[0]
#         tf_odom.transform.translation.y = self.mu[1]
#         tf_odom.transform.translation.z = 0.0
#         tf_odom.transform.rotation      = q

#         # TF dinámico: map → odom
#         # Esta es la corrección SLAM — diferencia entre pose real (map)
#         # y pose integrada por odometría (odom)
#         # Al inicio es identidad; se va corrigiendo con cada update EKF
#         tf_map = TransformStamped()
#         tf_map.header.stamp    = now
#         tf_map.header.frame_id = 'map'
#         tf_map.child_frame_id  = 'odom'

#         tf_map.transform.translation.x = self.map_correction_x
#         tf_map.transform.translation.y = self.map_correction_y
#         tf_map.transform.translation.z = 0.0
#         tf_map.transform.rotation = yaw_to_quaternion(self.map_correction_yaw)

#         self.tf_br.sendTransform([tf_odom, tf_map])

#     # ─────────────────────────────────────────────────────────────────
#     #  TF estático base_link → laser
#     # ─────────────────────────────────────────────────────────────────
#     def _publish_laser_tf(self):
#         tf_static = TransformStamped()
#         tf_static.header.stamp    = self.get_clock().now().to_msg()
#         tf_static.header.frame_id = 'base_link'
#         tf_static.child_frame_id  = 'laser'

#         tf_static.transform.translation.x = 0.07
#         tf_static.transform.translation.y = 0.0
#         tf_static.transform.translation.z = 0.205

#         yaw = math.pi  # 180° porque el LiDAR está montado al revés
#         tf_static.transform.rotation.x = 0.0
#         tf_static.transform.rotation.y = 0.0
#         tf_static.transform.rotation.z = math.sin(yaw / 2.0)
#         tf_static.transform.rotation.w = math.cos(yaw / 2.0)

#         self.static_br.sendTransform(tf_static)
#         self.get_logger().info('📡 TF estático base_link → laser publicado (yaw=180°)')
#         print('holaaaaa')

#     # ─────────────────────────────────────────────────────────────────
#     #  Publicar landmarks como esferas en RViz
#     # ─────────────────────────────────────────────────────────────────
#     def _publish_landmarks(self):
#         ma = MarkerArray()
#         now = self.get_clock().now().to_msg()

#         for j in range(self.n_landmarks):
#             m = Marker()
#             m.header.stamp    = now
#             m.header.frame_id = 'odom'
#             m.ns              = 'landmarks'
#             m.id              = j
#             m.type            = Marker.SPHERE
#             m.action          = Marker.ADD

#             m.pose.position.x = float(self.mu[3 + 2*j])
#             m.pose.position.y = float(self.mu[3 + 2*j + 1])
#             m.pose.position.z = 0.1
#             m.pose.orientation.w = 1.0

#             m.scale.x = m.scale.y = m.scale.z = 0.15

#             m.color.r = 0.0
#             m.color.g = 0.8
#             m.color.b = 1.0
#             m.color.a = 0.9

#             ma.markers.append(m)

#         self.lm_pub.publish(ma)


# # ─────────────────────────────────────────────────────────────────────
# #  main
# # ─────────────────────────────────────────────────────────────────────
# def main(args=None):
#     rclpy.init(args=args)
#     node = EKFSLAMNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


"""
Odometry Node — Dead Reckoning con Encoders

Integra las velocidades de los encoders del Puzzlebot para calcular
la pose (x, y, θ) usando el modelo cinemático diferencial.

Suscribe:
    /VelocityEncL  (std_msgs/Float32)
    /VelocityEncR  (std_msgs/Float32)

Publica:
    /odom          (nav_msgs/Odometry) — pose integrada
    TF dinámico:   odom → base_link
    TF estático:   base_link → laser (LiDAR rotado 180°)
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


# ─────────────────────────────────────────────────────────────────────
#  Utilidades
# ─────────────────────────────────────────────────────────────────────
def yaw_to_quaternion(yaw):
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0)
    )


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


# ─────────────────────────────────────────────────────────────────────
#  Nodo de Odometría (Dead Reckoning)
# ─────────────────────────────────────────────────────────────────────
class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry_node')

        # ── Parámetros del robot ──────────────────────────────────────
        self.r  = 0.05   # radio de rueda (m)
        self.L  = 0.19   # distancia entre ruedas (m)
        self.dt = 0.05   # periodo de integración (s)

        # ── Estado de la pose ─────────────────────────────────────────
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # ── Encoders ─────────────────────────────────────────────────
        self.w_l = 0.0
        self.w_r = 0.0

        # ── QoS ──────────────────────────────────────────────────────
        qos_enc = QoSProfile(depth=10)
        qos_enc.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # ── Suscripciones ────────────────────────────────────────────
        self.create_subscription(Float32, '/VelocityEncL', self.left_cb,  qos_enc)
        self.create_subscription(Float32, '/VelocityEncR', self.right_cb, qos_enc)

        # ── Publicadores ─────────────────────────────────────────────
        self.odom_pub  = self.create_publisher(Odometry, '/odom', 10)
        self.tf_br     = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)

        # TF estático base_link → laser (LiDAR rotado 180° en yaw)
        self._publish_laser_tf()

        # ── Timer de integración ─────────────────────────────────────
        self.create_timer(self.dt, self.integrate_and_publish)

        self.get_logger().info('🔧 Odometry node iniciado — dead reckoning puro')

    # ─────────────────────────────────────────────────────────────────
    #  Callbacks de encoders
    # ─────────────────────────────────────────────────────────────────
    def left_cb(self, msg):
        self.w_l = float(msg.data)

    def right_cb(self, msg):
        self.w_r = float(msg.data)

    # ─────────────────────────────────────────────────────────────────
    #  Integración cinemática diferencial
    # ─────────────────────────────────────────────────────────────────
    def integrate_and_publish(self):
        v = self.r * (self.w_r + self.w_l) / 2.0
        w = self.r * (self.w_r - self.w_l) / self.L

        # Actualizar pose
        self.x     += v * self.dt * math.cos(self.theta)
        self.y     += v * self.dt * math.sin(self.theta)
        self.theta  = normalize_angle(self.theta + w * self.dt)

        self._publish(v, w)

    # ─────────────────────────────────────────────────────────────────
    #  Publicar /odom y TF odom → base_link
    # ─────────────────────────────────────────────────────────────────
    def _publish(self, v, w):
        now = self.get_clock().now().to_msg()
        q   = yaw_to_quaternion(self.theta)

        # Mensaje Odometry
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.position.z  = 0.0
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = w

        # Covarianza básica (crece con el tiempo — dead reckoning acumula error)
        odom.pose.covariance[0]  = 0.01   # xx
        odom.pose.covariance[7]  = 0.01   # yy
        odom.pose.covariance[35] = 0.005  # θθ

        self.odom_pub.publish(odom)

        # TF dinámico: odom → base_link
        tf_odom = TransformStamped()
        tf_odom.header.stamp    = now
        tf_odom.header.frame_id = 'odom'
        tf_odom.child_frame_id  = 'base_link'

        tf_odom.transform.translation.x = self.x
        tf_odom.transform.translation.y = self.y
        tf_odom.transform.translation.z = 0.0
        tf_odom.transform.rotation      = q

        self.tf_br.sendTransform(tf_odom)

    # ─────────────────────────────────────────────────────────────────
    #  TF estático base_link → laser
    # ─────────────────────────────────────────────────────────────────
    def _publish_laser_tf(self):
        tf_static = TransformStamped()
        tf_static.header.stamp    = self.get_clock().now().to_msg()
        tf_static.header.frame_id = 'base_link'
        tf_static.child_frame_id  = 'laser'

        tf_static.transform.translation.x = 0.07
        tf_static.transform.translation.y = 0.0
        tf_static.transform.translation.z = 0.205

        yaw = math.pi  # 180° porque el LiDAR está montado al revés
        tf_static.transform.rotation.x = 0.0
        tf_static.transform.rotation.y = 0.0
        tf_static.transform.rotation.z = math.sin(yaw / 2.0)
        tf_static.transform.rotation.w = math.cos(yaw / 2.0)

        self.static_br.sendTransform(tf_static)
        self.get_logger().info('📡 TF estático base_link → laser publicado (yaw=180°)')


# ─────────────────────────────────────────────────────────────────────
#  main
# ─────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
