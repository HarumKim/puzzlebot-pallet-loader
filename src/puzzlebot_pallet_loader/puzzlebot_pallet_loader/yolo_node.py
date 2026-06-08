#!/usr/bin/env python3
"""
yolo_node.py — YOLO target detector gated by the mission FSM.

Goal:
  - Stay idle until /yolo/enable == true.
  - Use /qr_data to know which client class must be searched.
  - Process /camera/image_raw/compressed only while enabled.
  - Publish the selected drop waypoint in /yolo/target for fsm_control_node.
  - Publish annotated image on a topic that does NOT collide with qr_alignP.
"""

import json
import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String
from ultralytics import YOLO


# QR value -> YOLO class name.
# Ajusta estos nombres si tu best.pt usa nombres distintos.
DEFAULT_QR_TO_CLASS = {
    'Emezon': 'Client1',
    'Walmar': 'Client2',
    'Popsi':  'Client3',
}

# Position of detected target in the image -> drop waypoint name used by FSM.
DEFAULT_DIRECTION_TO_WP = {
    'IZQUIERDA': 'WP_DROP_1',
    'CENTRO':    'WP_DROP_2',
    'DERECHA':   'WP_DROP_3',
}


class DetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('enable_topic', '/yolo/enable')
        self.declare_parameter('qr_topic', '/qr_data')
        self.declare_parameter('target_topic', '/yolo/target')
        self.declare_parameter('image_topic', '/camera/image_raw/compressed')
        self.declare_parameter('annotated_topic', '/yolo/annotated/compressed')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('confidence_threshold', 0.50)
        self.declare_parameter('center_deadband_ratio', 0.10)
        self.declare_parameter('require_qr_data', True)

        # JSON strings to make launch/YAML overrides possible.
        self.declare_parameter('qr_to_class_json', json.dumps(DEFAULT_QR_TO_CLASS))
        self.declare_parameter('direction_to_waypoint_json', json.dumps(DEFAULT_DIRECTION_TO_WP))

        enable_topic = self.get_parameter('enable_topic').value
        qr_topic = self.get_parameter('qr_topic').value
        target_topic = self.get_parameter('target_topic').value
        image_topic = self.get_parameter('image_topic').value
        annotated_topic = self.get_parameter('annotated_topic').value
        detections_topic = self.get_parameter('detections_topic').value

        self._confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self._center_deadband_ratio = float(
            self.get_parameter('center_deadband_ratio').value
        )
        self._require_qr_data = bool(
            self.get_parameter('require_qr_data').value
        )

        self._qr_to_class = self._load_json_mapping(
            'qr_to_class_json',
            DEFAULT_QR_TO_CLASS
        )
        self._direction_to_wp = self._load_json_mapping(
            'direction_to_waypoint_json',
            DEFAULT_DIRECTION_TO_WP
        )

        # Case-insensitive helper maps.
        self._qr_to_class_lut = {
            str(k).strip().lower(): str(v).strip()
            for k, v in self._qr_to_class.items()
        }

        # ── Internal state ────────────────────────────────────────────
        self._enabled = False
        self._qr_value = None
        self._target_class = None
        self._last_target_id = None

        # ── Model ─────────────────────────────────────────────────────
        model_path = os.path.join(
            get_package_share_directory('puzzlebot_pallet_loader'),
            'best.pt'
        )
        self._model = YOLO(model_path)
        self.get_logger().info(f'Modelo YOLO cargado: {model_path}')
        self.get_logger().info(f'Clases del modelo: {self._model.names}')

        # ── Subscriptions ─────────────────────────────────────────────
        self.create_subscription(Bool, enable_topic, self._enable_cb, 10)
        self.create_subscription(String, qr_topic, self._qr_cb, 10)
        self.create_subscription(CompressedImage, image_topic, self._image_callback, 10)

        # ── Publishers ────────────────────────────────────────────────
        self._pub_annotated = self.create_publisher(
            CompressedImage,
            annotated_topic,
            10
        )
        self._pub_detections = self.create_publisher(
            String,
            detections_topic,
            10
        )
        self._pub_target = self.create_publisher(
            String,
            target_topic,
            10
        )

        self.get_logger().info(
            'YOLO listo. '
            f'enable_topic={enable_topic}, qr_topic={qr_topic}, '
            f'image_topic={image_topic}, target_topic={target_topic}, '
            f'annotated_topic={annotated_topic}, enabled={self._enabled}'
        )

    # ─────────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────────
    def _load_json_mapping(self, parameter_name: str, default_value: dict) -> dict:
        raw = self.get_parameter(parameter_name).value

        try:
            parsed = json.loads(raw)
            if isinstance(parsed, dict):
                return parsed
            self.get_logger().warn(
                f'Parameter {parameter_name} is not a JSON object. Using default.'
            )
        except Exception as exc:
            self.get_logger().warn(
                f'Could not parse {parameter_name}: {exc}. Using default.'
            )

        return dict(default_value)

    def _publish_annotated(self, frame, header):
        ret, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ret:
            return

        out = CompressedImage()
        out.header = header
        out.format = 'jpeg'
        out.data = buf.tobytes()
        self._pub_annotated.publish(out)

    def _publish_detections(self, detections, target_det):
        msg = String()
        msg.data = json.dumps({
            'detections': detections,
            'count': len(detections),
            'qr': self._qr_value,
            'target_class': self._target_class,
            'target_detection': target_det,
        })
        self._pub_detections.publish(msg)

    def _direction_from_bbox(self, bbox, frame_width: int):
        x1, _, x2, _ = bbox
        cx = (float(x1) + float(x2)) / 2.0
        frame_cx = frame_width / 2.0
        err_x = cx - frame_cx

        if abs(err_x) < frame_width * self._center_deadband_ratio:
            direction = 'CENTRO'
        elif err_x < 0.0:
            direction = 'IZQUIERDA'
        else:
            direction = 'DERECHA'

        return direction, err_x, cx

    # ─────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────
    def _enable_cb(self, msg: Bool):
        new_value = bool(msg.data)

        if new_value == self._enabled:
            return

        self._enabled = new_value

        if self._enabled:
            self._last_target_id = None
            self.get_logger().info('YOLO enabled by FSM.')
        else:
            self.get_logger().info('YOLO disabled by FSM.')

    def _qr_cb(self, msg: String):
        qr_raw = msg.data.strip()
        if not qr_raw or qr_raw == self._qr_value:
            return

        self._qr_value = qr_raw
        self._target_class = self._qr_to_class_lut.get(qr_raw.lower())
        self._last_target_id = None

        if self._target_class:
            self.get_logger().info(
                f'QR recibido: "{qr_raw}" → buscando clase YOLO: "{self._target_class}"'
            )
        else:
            self.get_logger().warn(
                f'QR "{qr_raw}" no existe en qr_to_class. '
                f'Llaves disponibles: {list(self._qr_to_class.keys())}'
            )

    def _image_callback(self, msg: CompressedImage):
        # Important: this prevents YOLO from using CPU/GPU/camera data before WP2.
        if not self._enabled:
            return

        if self._require_qr_data and not self._target_class:
            self._no_qr_frames = getattr(self, '_no_qr_frames', 0) + 1
            if self._no_qr_frames % 30 == 1:
                self.get_logger().warn(
                    f'YOLO activo pero sin clase objetivo — '
                    f'qr_value="{self._qr_value}", target_class=None. '
                    f'Claves validas: {list(self._qr_to_class.keys())}. '
                    f'Esperando /qr_data...'
                )
            return
        self._no_qr_frames = 0

        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        results = self._model(frame, verbose=False)[0]
        annotated = results.plot()
        detections = []
        target_det = None

        for box in results.boxes:
            cls_id = int(box.cls[0])
            cls_name = str(self._model.names[cls_id])
            confidence = round(float(box.conf[0]), 3)
            bbox = [round(float(v), 1) for v in box.xyxy[0].tolist()]

            det = {
                'class': cls_name,
                'confidence': confidence,
                'bbox': bbox,
            }
            detections.append(det)

            if confidence < self._confidence_threshold:
                continue

            # Primary mode: search only the class selected by QR.
            if self._target_class:
                if cls_name.strip().lower() != self._target_class.strip().lower():
                    continue

            # Fallback mode if require_qr_data=False: choose best detection.
            if target_det is None or confidence > target_det['confidence']:
                target_det = det

        frame_width = annotated.shape[1]

        if self._target_class:
            cv2.putText(
                annotated,
                f'Buscando: {self._target_class}',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2
            )
        elif self._qr_value:
            cv2.putText(
                annotated,
                f'QR "{self._qr_value}" sin clase YOLO asignada',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 100, 255),
                2
            )
        else:
            cv2.putText(
                annotated,
                'YOLO activo: esperando /qr_data...',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (100, 100, 100),
                2
            )

        if target_det is not None:
            direction, err_x, cx = self._direction_from_bbox(
                target_det['bbox'],
                frame_width
            )
            waypoint = self._direction_to_wp.get(direction, 'WP_DROP_2')

            x1, y1, x2, y2 = [int(v) for v in target_det['bbox']]
            cy = int((y1 + y2) / 2)

            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 3)
            cv2.circle(annotated, (int(cx), cy), 8, (0, 255, 0), -1)

            label = (
                f'{target_det["class"]} '
                f'({target_det["confidence"]:.0%}) - {direction} -> {waypoint}'
            )
            cv2.putText(
                annotated,
                label,
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2
            )

            payload = {
                'qr': self._qr_value,
                'target': target_det['class'],
                'target_class': self._target_class,
                'direction': direction,
                'waypoint': waypoint,
                'err_x': round(float(err_x), 1),
                'confidence': target_det['confidence'],
                'bbox': target_det['bbox'],
            }

            # Publish repeatedly enough for reliability, but avoid terminal spam.
            target_id = (
                f'{payload["qr"]}:{payload["target"]}:'
                f'{payload["direction"]}:{payload["waypoint"]}'
            )

            target_msg = String()
            target_msg.data = json.dumps(payload)
            self._pub_target.publish(target_msg)

            if target_id != self._last_target_id:
                self._last_target_id = target_id
                self.get_logger().info(
                    f'YOLO target confirmado: QR={self._qr_value} '
                    f'-> {payload["target"]} -> {direction} -> {waypoint} '
                    f'conf={payload["confidence"]:.2f}'
                )
        elif self._target_class:
            cv2.putText(
                annotated,
                f'{self._target_class} NO DETECTADO',
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        self._publish_annotated(annotated, msg.header)
        self._publish_detections(detections, target_det)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()

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