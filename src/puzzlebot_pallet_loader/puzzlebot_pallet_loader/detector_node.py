import cv2
import json
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
import os


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        model_path = os.path.join(
            get_package_share_directory('puzzlebot_pallet_loader'), 'best.pt'
        )
        self._model = YOLO(model_path)
        self.get_logger().info(f'Modelo YOLO cargado: {model_path}')

        self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self._image_callback, 10)

        self._pub_annotated = self.create_publisher(
            CompressedImage, '/detection/annotated/compressed', 10)
        self._pub_detections = self.create_publisher(
            String, '/detections', 10)

    def _image_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        results = self._model(frame, verbose=False)[0]
        annotated = results.plot()

        ret, buf = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ret:
            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = buf.tobytes()
            self._pub_annotated.publish(out)

        detections = []
        for box in results.boxes:
            cls_id = int(box.cls[0])
            detections.append({
                'class': self._model.names[cls_id],
                'confidence': round(float(box.conf[0]), 3),
                'bbox': [round(float(v), 1) for v in box.xyxy[0].tolist()],
            })
        det_msg = String()
        det_msg.data = json.dumps({'detections': detections, 'count': len(detections)})
        self._pub_detections.publish(det_msg)

        # --- ArUco (placeholder para más adelante) ---
        # aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        # corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict)
        # ...


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
