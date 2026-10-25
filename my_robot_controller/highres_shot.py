#!/usr/bin/env python3
"""Save frames from a Gazebo camera topic as PNGs.

    ros2 run my_robot_controller highres_shot \
        --topic /camera/photo --output ~/hero.png

Talking to Gazebo directly (rather than through ros_gz_bridge) means the
high-resolution ``photo_camera`` sensor is only active while this script is
subscribed: with ``always_on=false`` Gazebo parks it the rest of the time, so
the extra pixels cost nothing during normal driving.

The still can be timed, which is what makes the portfolio shots repeatable:

``--wait-near X,Y,R``
    hold off until ``my_robot`` is within R metres of a world point.
``--wait-traffic MIN,MAX``
    hold off until a car is between MIN and MAX metres ahead of the rover and
    inside ``--traffic-fov`` degrees of its heading, so the frame is
    guaranteed to contain moving traffic rather than an empty road.
``--wait-stop``
    hold off until the rover has been stationary for about half a second,
    i.e. the LiDAR safety layer is holding it.
``--wait-obstacle MAXDIST``
    hold off until a car or pedestrian is within MAXDIST metres and inside
    ``--obstacle-fov`` degrees of the rover's heading.

When a stop or obstacle condition is used the camera is subscribed *before*
the wait, so the 1080p sensor is already warm and the saved frame is the one
the LiDAR was looking at when it stopped the robot.

``--count``/``--interval`` grab a short burst; files are numbered
``name_0.png``, ``name_1.png`` ...
"""

import argparse
import collections
import math
import sys
import time

from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
from PIL import Image as PilImage

CAR_PREFIX = 'traffic_car_'
OBSTACLE_PREFIXES = ('traffic_car_', 'pedestrian_')


def parse_pair(text, names):
    parts = [float(v) for v in text.split(',')]
    if len(parts) != len(names):
        raise argparse.ArgumentTypeError(
            'expected ' + ','.join(names))
    return parts


def point_arg(text):
    return parse_pair(text, ('X', 'Y', 'R'))


def range_arg(text):
    return parse_pair(text, ('MIN', 'MAX'))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Trigger:
    """Watch /world/.../pose/info until the framing conditions are met."""

    def __init__(self, point=None, traffic=None, fov_deg=30.0, stop=False,
                 obstacle=None, obstacle_fov_deg=70.0):
        self.point = point
        self.traffic = traffic
        self.fov = math.radians(fov_deg)
        self.stop = stop
        self.obstacle = obstacle
        self.obstacle_fov = math.radians(obstacle_fov_deg)
        self.fired = False
        self.report = ''
        self.history = collections.deque()

    def _stationary(self, robot, now):
        """True when the rover has not moved for the last ~0.4 s."""
        self.history.append((now, robot.position.x, robot.position.y))
        while self.history and now - self.history[0][0] > 0.9:
            self.history.popleft()
        if not self.history or now - self.history[0][0] < 0.40:
            return False
        origin = self.history[0]
        return all(math.hypot(x - origin[1], y - origin[2]) < 0.06
                   for _, x, y in self.history)

    def _closest_obstacle(self, message, robot, yaw):
        best = None
        for pose in message.pose:
            if not pose.name.startswith(OBSTACLE_PREFIXES):
                continue
            dx = pose.position.x - robot.position.x
            dy = pose.position.y - robot.position.y
            distance = math.hypot(dx, dy)
            if distance > self.obstacle:
                continue
            bearing = wrap_angle(math.atan2(dy, dx) - yaw)
            if abs(bearing) > self.obstacle_fov:
                continue
            if best is None or distance < best[1]:
                best = (pose.name, distance, bearing)
        return best

    def on_pose(self, message):
        robot = next((p for p in message.pose if p.name == 'my_robot'), None)
        if robot is None:
            return

        if self.point is not None:
            target_x, target_y, radius = self.point
            if math.hypot(robot.position.x - target_x,
                          robot.position.y - target_y) > radius:
                return

        now = time.monotonic()
        if self.stop and not self._stationary(robot, now):
            return

        yaw = yaw_from_quaternion(robot.orientation)
        obstacle_report = None
        if self.obstacle is not None:
            best = self._closest_obstacle(message, robot, yaw)
            if best is None:
                return
            obstacle_report = (f'{best[0]} at {best[1]:.1f} m, '
                               f'{math.degrees(best[2]):+.0f} deg')

        traffic_report = None
        if self.traffic is not None:
            low, high = self.traffic
            for pose in message.pose:
                if not pose.name.startswith(CAR_PREFIX):
                    continue
                dx = pose.position.x - robot.position.x
                dy = pose.position.y - robot.position.y
                distance = math.hypot(dx, dy)
                if not low <= distance <= high:
                    continue
                bearing = wrap_angle(math.atan2(dy, dx) - yaw)
                if abs(bearing) > self.fov:
                    continue
                traffic_report = (f'{pose.name} at {distance:.1f} m, '
                                  f'{math.degrees(bearing):+.0f} deg')
                break
            if traffic_report is None:
                return

        self.fired = True
        parts = [report for report in (obstacle_report, traffic_report) if report]
        if self.stop:
            parts.append('robot stationary')
        self.report = ', '.join(parts) or 'robot in position'


def wait_for_trigger(node, trigger, topic, timeout):
    if not node.subscribe(Pose_V, topic, trigger.on_pose):
        print(f'error: could not subscribe to {topic}', file=sys.stderr)
        return False
    deadline = time.time() + timeout
    while time.time() < deadline and not trigger.fired:
        time.sleep(0.1)
    return trigger.fired


def save_frame(msg, path):
    width, height = msg.width, msg.height
    data = bytes(msg.data)
    pixels = width * height
    if len(data) == pixels * 3:
        mode = 'RGB'
    elif len(data) == pixels * 4:
        mode = 'RGBA'
    else:
        print(f'error: unexpected frame size {len(data)} for '
              f'{width}x{height} (format {msg.pixel_format_type})',
              file=sys.stderr)
        return False
    image = PilImage.frombytes(mode, (width, height), data)
    if mode == 'RGBA':
        image = image.convert('RGB')
    image.save(path)
    print(f'wrote {path} ({width}x{height})', flush=True)
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/camera/photo',
                        help='Gazebo image topic')
    parser.add_argument('--output', required=True, help='PNG file to write')
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='give up after this many seconds')
    parser.add_argument('--warmup', type=float, default=1.5,
                        help='discard frames for this long while the sensor '
                             'settles, then keep the next one')
    parser.add_argument('--wait-near', type=point_arg,
                        default=None, metavar='X,Y,R',
                        help='wait until my_robot is within R metres of (X, Y)')
    parser.add_argument('--wait-traffic', type=range_arg,
                        default=None, metavar='MIN,MAX',
                        help='wait until a car ahead is this far away, in the '
                             'camera forward cone')
    parser.add_argument('--traffic-fov', type=float, default=30.0,
                        help='half-angle in degrees for --wait-traffic')
    parser.add_argument('--wait-stop', action='store_true',
                        help='wait until the rover has been held stationary')
    parser.add_argument('--wait-obstacle', type=float, default=None,
                        metavar='MAXDIST',
                        help='wait until a car or pedestrian is this close and '
                             'roughly ahead')
    parser.add_argument('--obstacle-fov', type=float, default=70.0,
                        help='half-angle in degrees for --wait-obstacle')
    parser.add_argument('--wait-timeout', type=float, default=600.0,
                        help='how long to wait for the framing conditions')
    parser.add_argument('--pose-topic', default='/world/city/pose/info',
                        help='Gazebo pose topic used by the wait options')
    parser.add_argument('--count', type=int, default=1,
                        help='number of frames to save')
    parser.add_argument('--interval', type=float, default=1.0,
                        help='seconds between saved frames')
    args = parser.parse_args()

    node = Node()
    latest = {'frame': None}

    def on_image(msg):
        latest['frame'] = msg

    waiting = (args.wait_near is not None or args.wait_traffic is not None
               or args.wait_stop or args.wait_obstacle is not None)
    # A stop shot happens in a window of a second or two, so wake the big
    # sensor while we are still driving towards the scene.
    prewarm = args.wait_stop or args.wait_obstacle is not None

    if prewarm:
        if not node.subscribe(Image, args.topic, on_image):
            print(f'error: could not subscribe to {args.topic}', file=sys.stderr)
            return 1
        print('warming camera while waiting for framing conditions...',
              flush=True)

    if waiting:
        trigger = Trigger(args.wait_near, args.wait_traffic, args.traffic_fov,
                          args.wait_stop, args.wait_obstacle,
                          args.obstacle_fov)
        print('waiting for framing conditions...', flush=True)
        if not wait_for_trigger(node, trigger, args.pose_topic,
                                args.wait_timeout):
            print('error: framing conditions never met', file=sys.stderr)
            return 1
        print(f'framing ready ({trigger.report})', flush=True)
        if not prewarm:
            print('waking photo camera', flush=True)

    if not prewarm:
        if not node.subscribe(Image, args.topic, on_image):
            print(f'error: could not subscribe to {args.topic}', file=sys.stderr)
            return 1

    start = time.time()
    deadline = start + args.timeout
    saved = 0
    last_save = 0.0
    while saved < args.count and time.time() < deadline:
        time.sleep(0.05)
        msg = latest['frame']
        if msg is None:
            continue
        now = time.time()
        if now - start < args.warmup:
            continue
        if saved and now - last_save < args.interval:
            continue

        # Consume the frame so the next slot always gets a fresh one.
        latest['frame'] = None
        if args.count == 1:
            path = args.output
        else:
            stem, ext = args.output.rsplit('.', 1)
            path = f'{stem}_{saved}.{ext}'
        if not save_frame(msg, path):
            return 1
        saved += 1
        last_save = now

    if not saved:
        print(f'error: no frame on {args.topic} within {args.timeout:.0f}s',
              file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
