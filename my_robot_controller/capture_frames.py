#!/usr/bin/env python3
"""Save still frames from the rover camera as JPEGs.

Subscribes
    /camera/image (sensor_msgs/Image)
    /odom         (nav_msgs/Odometry)

A shot is taken once the rover has travelled ``spacing`` metres since the
previous one, so the pictures come from genuinely different parts of the
city.  Images are written to ``output_dir`` as ``city_shot_01.jpg`` ...

The container has no cv_bridge, so the raw RGB buffer is reshaped with numpy
and handed to Pillow, which writes the JPEG.
"""

import os

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage


class FrameCapture(Node):

    def __init__(self):
        super().__init__('frame_capture')

        self.output_dir = self.declare_parameter(
            'output_dir', os.path.expanduser('~/.ros/captures')).value
        self.count = int(self.declare_parameter('count', 3).value)
        self.spacing = self.declare_parameter('spacing', 45.0).value
        self.first_after = self.declare_parameter('first_after', 3.0).value
        self.quality = int(self.declare_parameter('jpeg_quality', 92).value)

        os.makedirs(self.output_dir, exist_ok=True)

        self._image = None
        self._last_xy = None
        self._travelled = 0.0
        self._next_at = self.first_after
        self._saved = 0

        self.create_subscription(RosImage, '/camera/image', self._on_image, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_timer(0.5, self._maybe_capture)

        self.get_logger().info(
            f'Capture node ready: {self.count} shots every {self.spacing:.0f} m '
            f'into {self.output_dir}')

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        if self._last_xy is not None:
            self._travelled += ((p.x - self._last_xy[0]) ** 2
                                + (p.y - self._last_xy[1]) ** 2) ** 0.5
        self._last_xy = (p.x, p.y)

    def _on_image(self, msg):
        self._image = msg

    def _to_rgb(self, msg):
        if msg.encoding not in ('rgb8', 'bgr8'):
            return None
        channels = 3
        expected = msg.height * msg.width * channels
        if len(msg.data) < expected:
            return None
        array = np.frombuffer(bytes(msg.data[:expected]), dtype=np.uint8)
        array = array.reshape(msg.height, msg.width, channels)
        if msg.encoding == 'bgr8':
            array = array[:, :, ::-1]
        return array

    def _maybe_capture(self):
        if self._saved >= self.count or self._image is None:
            return
        if self._travelled < self._next_at:
            return

        array = self._to_rgb(self._image)
        if array is None:
            self.get_logger().warn(
                f'Unsupported image encoding: {self._image.encoding}',
                throttle_duration_sec=5.0)
            return

        name = f'city_shot_{self._saved + 1:02d}.jpg'
        path = os.path.join(self.output_dir, name)
        Image.fromarray(array).save(path, 'JPEG', quality=self.quality)
        self._saved += 1
        self._next_at += self.spacing
        self.get_logger().info(
            f'Saved {path} after {self._travelled:.1f} m '
            f'({self._saved}/{self.count})')

        if self._saved >= self.count:
            self.get_logger().info(
                f'All {self.count} shots captured in {self.output_dir}')


def main(args=None):
    rclpy.init(args=args)
    node = FrameCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
