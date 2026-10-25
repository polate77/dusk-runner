#!/usr/bin/env python3
"""Cinematic third-person chase camera for the Gazebo GUI.

The GUI's ``CameraTracking`` plugin listens on ``/gui/track`` for
``gz.msgs.CameraTrack`` messages.  This node publishes one at ``rate`` Hz with
an offset that is rotated into the rover's heading, so the 3D view sits behind
and above the rover and swings around with it - the chase camera you get in a
racing game - with no mouse input required.

The rover's world pose is read straight from Gazebo's
``/world/<world>/dynamic_pose/info`` topic (only moving entities are
published there, so the message stays small even with ~150 static models).

Gazebo's CameraTracking already rotates the offset with the target heading,
so by default the raw body-frame offset is published unchanged and the plugin
keeps the camera behind the rover as it turns.  Set ``rotating:=true`` only if
a future version of the plugin expects a world-frame offset instead.

The plugin's follow_offset is measured from the look-at point rather than from
the target, so the camera settles roughly 4.1 m further back than the configured
``distance`` (measured: 4.0 -> 8.1 m, 7.0 -> 11.2 m).  The defaults below are
therefore the requested offset, which puts the view about 7.6 m behind and
3.2 m above the rover.
"""

import math

import rclpy
from rclpy.node import Node

from gz.msgs10.cameratrack_pb2 import CameraTrack
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ChaseCamera(Node):

    def __init__(self):
        super().__init__('chase_camera')

        self.world = self.declare_parameter('world', 'city').value
        self.target = self.declare_parameter('target', 'my_robot').value
        self.back = self.declare_parameter('distance', 3.5).value
        self.side = self.declare_parameter('side', 0.0).value
        self.height = self.declare_parameter('height', 3.2).value
        self.pgain = self.declare_parameter('pgain', 2.5).value
        self.rate = self.declare_parameter('rate', 15.0).value
        self.rotating = bool(self.declare_parameter('rotating', False).value)

        self._pose = None

        self._gz = GzNode()
        self._gui_pub = self._gz.advertise('/gui/track', CameraTrack)
        self._pose_sub = self._gz.subscribe(
            Pose_V, f'/world/{self.world}/dynamic_pose/info', self._on_pose)
        self.create_timer(1.0 / self.rate, self._publish)

        self.get_logger().info(
            f'Chase camera ready: following {self.target} '
            f'(offset {self.back:.1f} back / {self.height:.1f} up, '
            f'~{self.back + 4.1:.1f} m stand-off)')

    def _on_pose(self, msg):
        for pose in msg.pose:
            if pose.name == self.target:
                self._pose = pose
                return

    def _publish(self):
        if self._pose is None:
            return

        yaw = yaw_from_quaternion(self._pose.orientation) if self.rotating else 0.0
        c, s = math.cos(yaw), math.sin(yaw)
        # Body-frame offset (back along -x, lateral along y) rotated to world.
        dx = -self.back * c - self.side * s
        dy = -self.back * s + self.side * c

        track = CameraTrack()
        track.track_mode = CameraTrack.FOLLOW
        track.follow_target.name = self.target
        track.follow_target.type = Entity.MODEL
        track.follow_offset.x = dx
        track.follow_offset.y = dy
        track.follow_offset.z = self.height
        track.follow_pgain = self.pgain
        self._gui_pub.publish(track)

    def stop(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ChaseCamera()
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
