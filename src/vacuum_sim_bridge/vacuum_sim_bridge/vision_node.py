"""my_robot_vision_node (plain Node on purpose, see README): /camera (rgb8) and /camera_depth (32FC1, metres) from kinect/rgb and kinect/depth."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from vacuum_sim_bridge.camera_core import convert_depth, convert_rgb
from vacuum_sim_bridge.node_support import (declare_connection, declare_profile, make_client,
                                            static_transform)


class VisionNode(Node):
    def __init__(self, client=None, **kwargs):
        super().__init__('my_robot_vision_node', **kwargs)
        declare_connection(self)
        self.declare_parameter('rate_hz', 10.0)
        # Default off until verified live. The kinect sensors auto-render every simulation step (about 1.2 MB
        # leaked per step); when the sensors are set to explicit handling they only render when asked, so this
        # makes the node call handleVisionSensor on both before each frame.
        self.declare_parameter('explicit_sensor_handling', False)
        self._explicit = bool(self.get_parameter('explicit_sensor_handling').value)
        self._p = declare_profile(self)
        self._client = client if client is not None else make_client(self)
        self._rgb = self._depth = None
        self._pub = self.create_publisher(Image, '/camera', 10)
        self._pub_depth = self.create_publisher(Image, '/camera_depth', 10)
        self._tf = static_transform(self, self._p.base_frame, self._p.camera_frame, self._p.camera_xyz,
                                    self._p.camera_quat)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)

    def _image(self, stamp, array, encoding):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self._p.camera_frame
        msg.height, msg.width = int(array.shape[0]), int(array.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = int(array.strides[0])
        msg.data = array.tobytes()
        return msg

    def tick(self):
        try:
            if self._rgb is None:
                self._rgb = self._client.get_object(self._p.rgb_sensor_path)
                self._depth = self._client.get_object(self._p.depth_sensor_path)
            if self._explicit:
                self._client.handle_vision_sensor(self._rgb)
                self._client.handle_vision_sensor(self._depth)
            rgb = self._client.read_rgb(self._rgb)
            depth = self._client.read_depth(self._depth)
        except Exception as exc:
            self._rgb = None
            self.get_logger().warning(f'simulator not ready: {exc}', throttle_duration_sec=5.0)
            return
        stamp = self.get_clock().now().to_msg()      # one stamp for both images
        if rgb is not None:
            arr = convert_rgb(rgb[0], rgb[1], rgb[2], self._p)
            if arr is None:
                self.get_logger().warning('unexpected rgb buffer size', throttle_duration_sec=5.0)
            else:
                self._pub.publish(self._image(stamp, arr, 'rgb8'))
        if depth is not None:
            arr = convert_depth(depth[0], depth[1], depth[2], self._p)
            if arr is None:
                self.get_logger().warning('unexpected depth buffer size', throttle_duration_sec=5.0)
            else:
                self._pub_depth.publish(self._image(stamp, arr, '32FC1'))


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
