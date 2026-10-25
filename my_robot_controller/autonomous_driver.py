#!/usr/bin/env python3
"""Autonomous obstacle-avoiding driver for the delivery rover.

Subscribes
    /lidar/scan   (sensor_msgs/LaserScan)
Publishes
    /cmd_vel      (geometry_msgs/Twist) at ``publish_rate`` Hz

Behaviour
---------
* front cone clear  -> drive forward at ``linear_speed``
* obstacle closer than ``stop_distance`` in the front cone
      -> stop, pick the side with more free space, rotate in place
      -> resume when the front is clear again (``clear_distance`` gives
         hysteresis so the rover does not chatter at the threshold)
* no scan for ``scan_timeout`` seconds -> stop (fail safe)

Exploration
-----------
Left alone, the rover would follow one avenue until it leaves the map, so
after ``explore_after`` seconds without a turn it deliberately rotates for
``min_turn_time`` seconds and picks a new heading.  Set ``explore_after`` to
0.0 for pure "drive straight until blocked" behaviour.
"""

import math
import random

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class AutonomousDriver(Node):
    FORWARD = 'forward'
    TURNING = 'turning'

    def __init__(self):
        super().__init__('autonomous_driver')

        self.linear_speed = self.declare_parameter('linear_speed', 0.5).value
        self.turn_speed = self.declare_parameter('turn_speed', 1.2).value
        self.stop_distance = self.declare_parameter('stop_distance', 2.0).value
        self.clear_distance = self.declare_parameter('clear_distance', 2.6).value
        self.front_half = self.declare_parameter('front_cone', 0.61).value
        self.side_limit = self.declare_parameter('side_cone', 1.75).value
        self.scan_timeout = self.declare_parameter('scan_timeout', 1.0).value
        self.explore_after = self.declare_parameter('explore_after', 12.0).value
        self.min_turn_time = self.declare_parameter('min_turn_time', 0.9).value
        rate = self.declare_parameter('publish_rate', 20.0).value

        self._state = self.FORWARD
        self._turn_sign = 1.0
        self._turn_started = None
        self._last_scan = None
        self._last_scan_time = None
        self._last_turn_time = self.get_clock().now()

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/lidar/scan', self._on_scan, 10)
        self.create_timer(1.0 / rate, self._control)

        self.get_logger().info(
            f'Autonomous driver ready: {self.linear_speed:.2f} m/s, stop at '
            f'{self.stop_distance:.2f} m, explore every {self.explore_after:.1f} s')

    # ------------------------------------------------------------------ scan
    def _on_scan(self, msg):
        self._last_scan = msg
        self._last_scan_time = self.get_clock().now()

    def _scan_age(self):
        if self._last_scan_time is None:
            return math.inf
        return (self.get_clock().now() - self._last_scan_time).nanoseconds * 1e-9

    def _sector_minima(self, scan):
        """Nearest finite range in the front, left and right sectors."""
        front = left = right = math.inf
        angle = scan.angle_min
        for distance in scan.ranges:
            if math.isfinite(distance):
                if abs(angle) <= self.front_half:
                    front = min(front, distance)
                elif self.front_half < angle <= self.side_limit:
                    left = min(left, distance)
                elif -self.side_limit <= angle < -self.front_half:
                    right = min(right, distance)
            angle += scan.angle_increment
        return front, left, right

    # --------------------------------------------------------------- control
    def _publish(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)

    def _begin_turn(self, now, direction, front, left, right, reason):
        self._state = self.TURNING
        self._turn_sign = 1.0 if direction else -1.0
        self._turn_started = now
        self.get_logger().info(
            f'TURN {self._turn_sign:+.0f} ({reason}): front={front:.2f} m, '
            f'left={left:.2f} m, right={right:.2f} m')

    def _control(self):
        now = self.get_clock().now()

        if self._last_scan is None or self._scan_age() > self.scan_timeout:
            self._publish(0.0, 0.0)
            return

        front, left, right = self._sector_minima(self._last_scan)

        if self._state == self.FORWARD:
            if front < self.stop_distance:
                self._begin_turn(now, left >= right, front, left, right, 'obstacle')
            else:
                idle = (now - self._last_turn_time).nanoseconds * 1e-9
                if self.explore_after > 0.0 and idle >= self.explore_after:
                    self._begin_turn(now, random.random() < 0.5,
                                     front, left, right, 'explore')
                else:
                    self._publish(self.linear_speed, 0.0)
                    return

        # ------------------------------------------------------ turning
        elapsed = (now - self._turn_started).nanoseconds * 1e-9
        if front >= self.clear_distance and elapsed >= self.min_turn_time:
            self._state = self.FORWARD
            self._last_turn_time = now
            self.get_logger().info(f'FORWARD (front clear at {front:.2f} m)')
            self._publish(self.linear_speed, 0.0)
            return

        # If the side we picked is walled in, swing the other way.
        if self._turn_sign > 0 and left < 0.6 * self.stop_distance:
            self._turn_sign = -1.0
        elif self._turn_sign < 0 and right < 0.6 * self.stop_distance:
            self._turn_sign = 1.0

        self._publish(0.0, self._turn_sign * self.turn_speed)

    def stop(self):
        if rclpy.ok():
            self._publish(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
