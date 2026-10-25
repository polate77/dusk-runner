#!/usr/bin/env python3
"""Keyboard teleoperation for the differential drive robot.

W / S   : forward / backward
A / D   : turn left / right
SPACE   : stop
Q / ESC : quit

Publishes geometry_msgs/msg/Twist on /cmd_vel at 20 Hz. When no key is held a
zero Twist is published, so the robot stops as soon as a key is released.
"""

import threading

import rclpy
from geometry_msgs.msg import Twist
from pynput import keyboard
from rclpy.node import Node


class TeleopNode(Node):
    LINEAR_SPEED = 0.3      # m/s
    ANGULAR_SPEED = 1.0     # rad/s
    PUBLISH_PERIOD = 0.05   # s -> 20 Hz

    def __init__(self):
        super().__init__('teleop_node')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self._pressed = set()
        self._lock = threading.Lock()
        self.running = True

        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self.listener.start()

        self.timer = self.create_timer(self.PUBLISH_PERIOD, self._publish)

        self.get_logger().info(
            'Teleop ready - W/S: forward/back, A/D: turn, SPACE: stop, Q: quit')

    # ------------------------------------------------------------ keyboard
    @staticmethod
    def _as_char(key):
        """Return a lowercase character for a key, or None for other keys."""
        try:
            return key.char.lower()
        except AttributeError:
            pass
        if key == keyboard.Key.space:
            return ' '
        if key == keyboard.Key.esc:
            return '\x1b'
        return None

    def _on_press(self, key):
        char = self._as_char(key)
        if char is None:
            return
        if char in ('q', '\x1b'):
            self.get_logger().info('Quit requested')
            self.running = False
            self.listener.stop()
            return
        with self._lock:
            self._pressed.add(char)

    def _on_release(self, key):
        char = self._as_char(key)
        if char is None:
            return
        with self._lock:
            self._pressed.discard(char)

    # ------------------------------------------------------------ publish
    def _publish(self):
        with self._lock:
            pressed = set(self._pressed)

        msg = Twist()
        if ' ' not in pressed:          # SPACE = emergency stop
            if 'w' in pressed:
                msg.linear.x += self.LINEAR_SPEED
            if 's' in pressed:
                msg.linear.x -= self.LINEAR_SPEED
            if 'a' in pressed:
                msg.angular.z += self.ANGULAR_SPEED
            if 'd' in pressed:
                msg.angular.z -= self.ANGULAR_SPEED

        self.publisher.publish(msg)

    def stop(self):
        self.listener.stop()
        if rclpy.ok():
            self.publisher.publish(Twist())

    def destroy_node(self):
        self.listener.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
