"""
EKF ArUco Localization Node — Localización con Filtro de Kalman Extendido

Estado:  μ = [x, y, θ]     (3×1)
Covarianza: Σ ∈ ℝ³ˣ³

NO es EKF-SLAM: los landmarks (ArUco) tienen posiciones fijas conocidas
y NO se agregan al estado.

Predicción:
    Usa encoders (/VelocityEncL, /VelocityEncR) con modelo cinemático diferencial.
    Propaga covarianza con Jacobiano G y ruido de proceso Q.

Corrección:
    Cuando llega una medición ArUco (/aruco_measurements), busca el landmark
    en el mapa conocido y aplica update EKF con modelo de observación h(μ)=[r, β].

Suscribe:
    /VelocityEncL       (std_msgs/Float32)          — encoder izquierdo
    /VelocityEncR       (std_msgs/Float32)          — encoder derecho
    /aruco_measurements (std_msgs/Float32MultiArray) — [marker_id, range, bearing]

Publica:
    /ekf_odom           (nav_msgs/Odometry)          — pose estimada con covarianza
    TF dinámico:  odom → base_link
    TF estático:  base_link → laser
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


# ─────────────────────────────────────────────────────────────────────
#  Utilidades matemáticas
# ─────────────────────────────────────────────────────────────────────
def yaw_to_quaternion(yaw):
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0)
    )


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class EKFArucoLocalizationNode(Node):

    def __init__(self):
        super().__init__('ekf_aruco_localization_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('q_x', 0.01)
        self.declare_parameter('q_y', 0.01)
        self.declare_parameter('q_theta', 0.005)
        self.declare_parameter('r_range', 0.05)
        self.declare_parameter('r_bearing', 0.05)
        self.declare_parameter('mahalanobis_threshold', 5.991)
        self.declare_parameter('ekf_odom_topic', '/ekf_odom')
        self.declare_parameter('aruco_topic', '/aruco_measurements')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('initial_cov_x', 0.0)
        self.declare_parameter('initial_cov_y', 0.0)
        self.declare_parameter('initial_cov_yaw', 0.0)

        # ArUco map parameters
        self.declare_parameter('marker_ids', [0, 1, 2, 3])
        self.declare_parameter('marker_x', [2.5, 2.5, -0.5, -0.5])
        self.declare_parameter('marker_y', [-0.5, 2.5, 2.5, -0.5])
        self.declare_parameter('marker_theta', [3.14159, 4.71239, 0.0, 1.5708])

        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_base').value
        self.dt = self.get_parameter('dt').value

        q_x = self.get_parameter('q_x').value
        q_y = self.get_parameter('q_y').value
        q_theta = self.get_parameter('q_theta').value
        r_range = self.get_parameter('r_range').value
        r_bearing = self.get_parameter('r_bearing').value
        self._mahal_thresh = self.get_parameter('mahalanobis_threshold').value

        ekf_odom_topic = self.get_parameter('ekf_odom_topic').value
        aruco_topic = self.get_parameter('aruco_topic').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        laser_frame = self.get_parameter('laser_frame').value

        # ── Build ArUco landmark map ──────────────────────────────────
        ids = self.get_parameter('marker_ids').value
        xs = self.get_parameter('marker_x').value
        ys = self.get_parameter('marker_y').value
        # Dict: marker_id → (x, y)
        self._landmarks = {}
        for i, mid in enumerate(ids):
            self._landmarks[int(mid)] = (float(xs[i]), float(ys[i]))
        self.get_logger().info(f'📍 Loaded {len(self._landmarks)} ArUco landmarks: {self._landmarks}')

        # ── EKF state ─────────────────────────────────────────────────
        # μ = [x, y, θ]
        self.mu = np.array([
            float(self.get_parameter('initial_x').value),
            float(self.get_parameter('initial_y').value),
            float(self.get_parameter('initial_yaw').value),
        ])
        # Σ = 3×3 covariance
        self.Sigma = np.diag([
            float(self.get_parameter('initial_cov_x').value),
            float(self.get_parameter('initial_cov_y').value),
            float(self.get_parameter('initial_cov_yaw').value),
        ])

        # ── Noise matrices ────────────────────────────────────────────
        self.Q = np.diag([q_x, q_y, q_theta])
        self.R = np.diag([r_range, r_bearing])

        # ── Encoder velocities ────────────────────────────────────────
        self.w_l = 0.0
        self.w_r = 0.0

        # ── QoS ──────────────────────────────────────────────────────
        qos_enc = QoSProfile(depth=10)
        qos_enc.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # ── Subscriptions ─────────────────────────────────────────────
        self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10)
        self.create_subscription(
            Float32MultiArray, aruco_topic, self._aruco_cb, 10)

        # ── Publishers ────────────────────────────────────────────────
        self._odom_pub = self.create_publisher(Odometry, ekf_odom_topic, 10)
        self._tf_br = TransformBroadcaster(self)
        self._static_br = StaticTransformBroadcaster(self)

        # TF estático base_link → laser (LiDAR rotado 180° en yaw)
        self._publish_laser_tf(laser_frame)

        # ── Prediction timer ──────────────────────────────────────────
        self.create_timer(self.dt, self._predict_and_publish)

        self.get_logger().info(
            f'🧠 EKF localization started — r={self.r}, L={self.L}, dt={self.dt}')

    # ─────────────────────────────────────────────────────────────────
    #  Joint states callback (replaces encoder callbacks for Gazebo)
    # ─────────────────────────────────────────────────────────────────
    def _joint_states_cb(self, msg: JointState):
        if not msg.name or not msg.velocity:
            return

        for i, name in enumerate(msg.name):
            if 'wheel_left_joint' in name:
                self.w_l = float(msg.velocity[i])
            elif 'wheel_right_joint' in name:
                self.w_r = float(msg.velocity[i])

    # ─────────────────────────────────────────────────────────────────
    #  PREDICTION STEP
    #
    #  Uses encoder velocities with differential drive kinematics.
    #  Propagates covariance: Σ_pred = G · Σ · Gᵀ + Q
    # ─────────────────────────────────────────────────────────────────
    def _predict_and_publish(self):
        v = self.r * (self.w_r + self.w_l) / 2.0
        w = self.r * (self.w_r - self.w_l) / self.L

        x, y, theta = self.mu[0], self.mu[1], self.mu[2]
        T = v * self.dt  # linear displacement this step

        # Update pose
        self.mu[0] = x + T * math.cos(theta)
        self.mu[1] = y + T * math.sin(theta)
        self.mu[2] = normalize_angle(theta + w * self.dt)

        # Jacobian G of the motion model
        G = np.array([
            [1.0, 0.0, -T * math.sin(theta)],
            [0.0, 1.0,  T * math.cos(theta)],
            [0.0, 0.0,  1.0]
        ])

        # Propagate covariance
        self.Sigma = G @ self.Sigma @ G.T + self.Q

        # Publish
        self._publish_odom(v, w)

    # ─────────────────────────────────────────────────────────────────
    #  CORRECTION STEP
    #
    #  Called when an ArUco measurement arrives.
    #  Observation model: h(μ) = [r, β]
    #    r = sqrt(dx² + dy²)
    #    β = atan2(dy, dx) - θ
    #
    #  Update equations:
    #    y = z - h(μ)
    #    S = H · Σ · Hᵀ + R
    #    K = Σ · Hᵀ · S⁻¹
    #    μ = μ + K · y
    #    Σ = (I - K · H) · Σ
    # ─────────────────────────────────────────────────────────────────
    def _aruco_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            return

        marker_id = int(msg.data[0])
        z_range = msg.data[1]
        z_bearing = msg.data[2]

        # Look up landmark in known map
        if marker_id not in self._landmarks:
            self.get_logger().debug(
                f'ArUco #{marker_id} not in known map — ignoring')
            return

        lx, ly = self._landmarks[marker_id]
        x, y, theta = self.mu[0], self.mu[1], self.mu[2]

        # Predicted measurement h(μ)
        dx = lx - x
        dy = ly - y
        p = dx**2 + dy**2
        q = math.sqrt(p)

        if q < 1e-6:
            return  # avoid division by zero

        h_pred = np.array([
            q,
            normalize_angle(math.atan2(dy, dx) - theta)
        ])

        # Actual measurement
        z = np.array([z_range, z_bearing])

        # Innovation
        innov = z - h_pred
        innov[1] = normalize_angle(innov[1])

        # Jacobian H of the observation model
        H = np.array([
            [-dx / q, -dy / q,  0.0],
            [ dy / p, -dx / p, -1.0]
        ])

        # Innovation covariance
        S = H @ self.Sigma @ H.T + self.R

        # Mahalanobis gate — reject outliers
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        d_mahal = float(innov.T @ S_inv @ innov)

        if d_mahal > self._mahal_thresh:
            self.get_logger().debug(
                f'ArUco #{marker_id} rejected — Mahalanobis={d_mahal:.2f} '
                f'> {self._mahal_thresh:.2f}')
            return

        # Kalman gain
        K = self.Sigma @ H.T @ S_inv

        # State update
        self.mu = self.mu + K @ innov
        self.mu[2] = normalize_angle(self.mu[2])

        # Covariance update
        I = np.eye(3)
        self.Sigma = (I - K @ H) @ self.Sigma

        self.get_logger().info(
            f'✅ ArUco #{marker_id} correction — '
            f'range={z_range:.2f}m, Mahal={d_mahal:.2f}, '
            f'pose=({self.mu[0]:.2f}, {self.mu[1]:.2f}, '
            f'{math.degrees(self.mu[2]):.1f}°)')

    # ─────────────────────────────────────────────────────────────────
    #  Publish /ekf_odom and TF
    # ─────────────────────────────────────────────────────────────────
    def _publish_odom(self, v, w):
        now = self.get_clock().now().to_msg()
        q = yaw_to_quaternion(self.mu[2])

        # Odometry message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame

        odom.pose.pose.position.x = float(self.mu[0])
        odom.pose.pose.position.y = float(self.mu[1])
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # Embed 3×3 Σ into 6×6 covariance (indices: 0=x, 1=y, 5=yaw)
        # Row-major 6×6: cov[i*6+j]
        odom.pose.covariance[0] = float(self.Sigma[0, 0])   # xx
        odom.pose.covariance[1] = float(self.Sigma[0, 1])   # xy
        odom.pose.covariance[5] = float(self.Sigma[0, 2])   # xθ
        odom.pose.covariance[6] = float(self.Sigma[1, 0])   # yx
        odom.pose.covariance[7] = float(self.Sigma[1, 1])   # yy
        odom.pose.covariance[11] = float(self.Sigma[1, 2])  # yθ
        odom.pose.covariance[30] = float(self.Sigma[2, 0])  # θx
        odom.pose.covariance[31] = float(self.Sigma[2, 1])  # θy
        odom.pose.covariance[35] = float(self.Sigma[2, 2])  # θθ

        self._odom_pub.publish(odom)

        # TF dinámico: odom → base_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self._odom_frame
        tf_msg.child_frame_id = self._base_frame

        tf_msg.transform.translation.x = float(self.mu[0])
        tf_msg.transform.translation.y = float(self.mu[1])
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = q

        self._tf_br.sendTransform(tf_msg)

    # ─────────────────────────────────────────────────────────────────
    #  TF estático base_link → laser (LiDAR rotado 180°)
    # ─────────────────────────────────────────────────────────────────
    def _publish_laser_tf(self, laser_frame):
        tf_static = TransformStamped()
        tf_static.header.stamp = self.get_clock().now().to_msg()
        tf_static.header.frame_id = self._base_frame
        tf_static.child_frame_id = laser_frame

        tf_static.transform.translation.x = 0.07
        tf_static.transform.translation.y = 0.0
        tf_static.transform.translation.z = 0.205

        yaw = math.pi  # 180° — LiDAR montado al revés
        tf_static.transform.rotation.x = 0.0
        tf_static.transform.rotation.y = 0.0
        tf_static.transform.rotation.z = math.sin(yaw / 2.0)
        tf_static.transform.rotation.w = math.cos(yaw / 2.0)

        self._static_br.sendTransform(tf_static)
        self.get_logger().info('📡 TF estático base_link → laser publicado (yaw=180°)')


def main(args=None):
    rclpy.init(args=args)
    node = EKFArucoLocalizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
