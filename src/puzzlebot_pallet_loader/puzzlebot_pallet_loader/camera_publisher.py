import subprocess
import threading
import rclpy
import numpy as np
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
from rclpy.node import Node
from sensor_msgs.msg import Image

"""
-- TERMINAL 1 --
gst-launch-1.0 udpsrc port=5004 ! tsdemux ! h264parse ! decodebin ! queue ! autovideosink sync=false

-- TERMINAL 2 -- SE CORRE DENTRO DE LA JETSON!!
cd ~/puzzlebot-pallet-loader
colcon build --packages-select puzzlebot_pallet_loader
source install/setup.bash

-- CAMBIAR POR IP DE TU COMPU -- 
ros2 run puzzlebot_pallet_loader camera_publisher --ros-args -p udp_host:=10.42.0.160 
"""

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        Gst.init(None)

        self.declare_parameter('udp_host', '10.42.0.160')      
        self.declare_parameter('udp_port', 5004)
        self.declare_parameter('ros_fps', 10)

        udp_host = self.get_parameter('udp_host').value
        udp_port = self.get_parameter('udp_port').value
        ros_fps  = self.get_parameter('ros_fps').value

        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self._latest_frame = None
        self._lock = threading.Lock()

        pipeline_str = (
            "nvarguscamerasrc sensor-mode=4 ee-mode=0 tnr-mode=0 ! "
            "video/x-raw(memory:NVMM),width=640,height=480,framerate=30/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw(memory:NVMM),width=640,height=480 ! "
            "tee name=t "
            f"t. ! queue ! nvv4l2h264enc bitrate=1500000 insert-sps-pps=true iframeinterval=30 ! "
            f"h264parse ! mpegtsmux ! udpsink host={udp_host} port={udp_port} sync=false "
            "t. ! queue ! nvvidconv ! "
            "video/x-raw,width=640,height=480,format=BGRx ! "
            "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
        )

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name('sink')
        self.appsink.connect('new-sample', self._on_new_sample)

        self.create_timer(1.0 / ros_fps, self._publish_ros_frame)

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error('No se pudo iniciar el pipeline GStreamer!')
        else:
            self.get_logger().info(
                f'Cámara CSI iniciada — stream UDP→{udp_host}:{udp_port}, '
                f'ROS topic a {ros_fps}fps'
            )

    def _on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        caps = sample.get_caps()
        struct = caps.get_structure(0)
        width = struct.get_value('width')
        height = struct.get_value('height')

        ok, map_info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR

        frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape(height, width, 4)
        frame_bgr = np.ascontiguousarray(frame[:, :, :3])
        buf.unmap(map_info)

        with self._lock:
            self._latest_frame = frame_bgr

        return Gst.FlowReturn.OK

    def _publish_ros_frame(self):
        with self._lock:
            if self._latest_frame is None:
                return
            frame = self._latest_frame

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()

    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass

    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
