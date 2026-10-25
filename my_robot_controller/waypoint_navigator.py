#!/usr/bin/env python3
"""Waypoint navigation for the delivery rover.

Subscribes
    /odom        (nav_msgs/Odometry)   - position and distance to waypoints
    /imu         (sensor_msgs/Imu)     - chassis heading (see "Heading" below)
    /lidar/scan  (sensor_msgs/LaserScan) - front clearance safety stop

Publishes
    /cmd_vel     (geometry_msgs/Twist) at ``control_rate`` Hz
    /route       (nav_msgs/Path)       - the planned route, for RViz

Behaviour
---------
The rover is given a list of waypoints that sit on the city road centrelines.
For the current target it computes the distance and the bearing error from
odometry, then:

* bearing error above ``turn_in_place_angle`` -> pivot on the spot until the
  error drops below ``align_done_angle``; corners stay sharp instead of the
  rover arcing wide into the block behind the intersection
* small bearing error  -> drive forward, with the speed falling off as the
  heading error grows, so the rover eases through curves
* approaching a corner (within ``slow_radius``) -> speed is ramped down to
  ``corner_speed`` so the turn happens slowly and smoothly
* on a straight road      -> full ``cruise_speed``

A waypoint counts as reached within ``waypoint_tolerance`` metres.

Frames
------
Gazebo's differential-drive odometry starts at zero with heading 0 at the
spawn point, while waypoints are naturally written in world coordinates.
``start_pose`` (the spawn pose) is used to rotate/translate the route into
the odom frame once at start-up:

    odom = R(-yaw0) * (world - origin)

so the same route keeps working as long as the launch file spawns the rover
in the same place.

Heading
-------
This rover is a four-wheel skid-steer: in an in-place pivot the wheels scrub
sideways, so wheel odometry over-reports the yaw it just turned by 1.2-1.5x
(measured: odom says 90 degrees when the chassis has really turned ~60).  That
is what used to make the rover cut the first corner into a building.

Heading is therefore taken from the IMU (chassis rotation, no slip).  Note that
Gazebo Harmonic reports the IMU orientation relative to the pose the model was
spawned in, so its yaw is already the odom-frame heading.  ROS IMUs normally
report world yaw instead.  The first sample is used to tell the two apart -
at spawn the odom-frame heading must be zero - so the node works with either
convention:

    imu_yaw_odom = wrap(imu_yaw - offset)   # offset 0 or yaw0

/odom supplies the wheel speed, and the position is dead-reckoned from that
speed along the IMU heading - the same fusion an EKF with encoders + gyro
performs.  Dead reckoning is needed because Gazebo's own odometry integrates
position using its slip-corrupted yaw, so after one pivot its pose estimate is
rotated by up to 30 degrees relative to the world and every waypoint looks
misplaced.  Wheel speed on straights matches ground truth (measured ratio
1.00), and pivots contribute no translation, so this estimate stays tight.

With ``use_imu`` false - or if no IMU message ever arrives - the node falls
back to the raw /odom pose and still drives, just with the old corner-cutting
behaviour.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan

# Route in world coordinates: a clockwise loop around four city blocks.
# Every leg runs along a road centreline (roads at x, y = -30 / 0 / 30).
DEFAULT_ROUTE = [
    0.0, 30.0,     # north up the x = 0 avenue
    30.0, 30.0,    # east along y = 30
    30.0, -30.0,   # south down x = 30
    -30.0, -30.0,  # west along y = -30
    -30.0, 30.0,   # north up x = -30
]

# Spawn pose used by robot.launch.py: x, y, yaw.
DEFAULT_START = [0.0, -11.0, 1.5708]


def wrap_angle(angle):
    """Wrap an angle into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        self.cruise_speed = self.declare_parameter('cruise_speed', 1.2).value
        self.corner_speed = self.declare_parameter('corner_speed', 0.45).value
        self.heading_gain = self.declare_parameter('heading_gain', 1.8).value
        self.max_angular = self.declare_parameter('max_angular', 1.5).value
        self.turn_in_place_angle = self.declare_parameter(
            'turn_in_place_angle', 0.35).value
        self.waypoint_tolerance = self.declare_parameter(
            'waypoint_tolerance', 0.8).value
        self.align_done_angle = self.declare_parameter(
            'align_done_angle', 0.15).value
        self.slow_radius = self.declare_parameter('slow_radius', 7.0).value
        self.min_clearance = self.declare_parameter('min_clearance', 0.55).value
        self.front_half = self.declare_parameter('front_cone', 0.61).value
        self.control_rate = self.declare_parameter('control_rate', 20.0).value
        self.loop = bool(self.declare_parameter('loop', True).value)
        self.use_imu = bool(self.declare_parameter('use_imu', True).value)
        start = self.declare_parameter('start_pose', DEFAULT_START).value
        route = list(self.declare_parameter('route', DEFAULT_ROUTE).value)

        self._origin = (float(start[0]), float(start[1]), float(start[2]))
        self._route_world = [(float(route[i]), float(route[i + 1]))
                             for i in range(0, len(route) - 1, 2)]
        self._route = [self._to_odom(*wp) for wp in self._route_world]
        if not self._route:
            raise ValueError('route parameter is empty')

        self._index = 0
        self._pose = None
        self._imu_yaw = None
        self._imu_offset = None
        # Dead-reckoned position in the odom frame (wheel speed x IMU yaw).
        self._dead_x = 0.0
        self._dead_y = 0.0
        self._last_stamp = None
        self._front_range = math.inf
        self._finished = False
        self._aligning = False
        self._heading_warned = False
        self._start_time = self.get_clock().now()

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._path_pub = self.create_publisher(Path, '/route', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Imu, '/imu', self._on_imu, 50)
        self.create_subscription(LaserScan, '/lidar/scan', self._on_scan, 10)
        self.create_timer(1.0 / self.control_rate, self._control)
        self.create_timer(2.0, self._publish_path)

        self.get_logger().info(
            f'Waypoint navigator ready: {len(self._route)} waypoints, '
            f'cruise {self.cruise_speed:.2f} m/s, corners '
            f'{self.corner_speed:.2f} m/s, loop={self.loop}, '
            f'heading source={"imu" if self.use_imu else "odom"}')
        for i, (x, y) in enumerate(self._route_world):
            self.get_logger().info(
                f'  waypoint {i + 1}: world ({x:.1f}, {y:.1f})')

    # ------------------------------------------------------------- frames
    def _to_odom(self, wx, wy):
        ox, oy, yaw0 = self._origin
        dx, dy = wx - ox, wy - oy
        c, s = math.cos(yaw0), math.sin(yaw0)
        return (c * dx + s * dy, -s * dx + c * dy)

    # --------------------------------------------------------- callbacks
    def _on_odom(self, msg):
        p = msg.pose.pose
        yaw = yaw_from_quaternion(p.orientation)
        self._pose = (p.position.x, p.position.y, yaw)

        # Dead reckoning: Gazebo's odom pose yaw slips during pivots, but the
        # forward wheel speed it reports is good, so integrate that speed
        # along the (accurate) IMU heading instead of trusting odom's pose.
        stamp = msg.header.stamp
        now = stamp.sec + stamp.nanosec * 1e-9
        if self._last_stamp is not None:
            dt = now - self._last_stamp
            if 0.0 < dt < 0.5:
                heading = yaw if self._imu_yaw is None else self._imu_yaw
                speed = msg.twist.twist.linear.x
                self._dead_x += speed * math.cos(heading) * dt
                self._dead_y += speed * math.sin(heading) * dt
        self._last_stamp = now

    def _on_imu(self, msg):
        if not self.use_imu:
            return
        yaw = yaw_from_quaternion(msg.orientation)
        if self._imu_offset is None:
            # Decide once, from the first sample, whether the IMU reports
            # world yaw (offset = yaw0) or a pose relative to the spawn
            # point (offset = 0).  At spawn the heading must be zero in the
            # odom frame, so the convention that gives zero wins.
            yaw0 = self._origin[2]
            if abs(wrap_angle(yaw - yaw0)) < abs(wrap_angle(yaw)):
                self._imu_offset = yaw0
                convention = 'world frame'
            else:
                self._imu_offset = 0.0
                convention = 'spawn-relative (Gazebo Harmonic)'
            self.get_logger().info(
                f'IMU heading convention detected: {convention}')
        self._imu_yaw = wrap_angle(yaw - self._imu_offset)

    def _on_scan(self, msg):
        front = math.inf
        angle = msg.angle_min
        for distance in msg.ranges:
            if math.isfinite(distance) and abs(angle) <= self.front_half:
                front = min(front, distance)
            angle += msg.angle_increment
        self._front_range = front

    # ---------------------------------------------------------- control
    def _publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._cmd_pub.publish(msg)

    def _control(self):
        if self._pose is None or self._finished:
            return
        if self._front_range < self.min_clearance:
            self._publish_cmd(0.0, 0.0)
            self.get_logger().warn(
                f'Obstacle at {self._front_range:.2f} m - holding position',
                throttle_duration_sec=2.0)
            return

        x, y, odom_yaw = self._pose
        if self._imu_yaw is not None:
            yaw = self._imu_yaw
            x, y = self._dead_x, self._dead_y
        else:
            yaw = odom_yaw
            elapsed = (self.get_clock().now()
                       - self._start_time).nanoseconds / 1e9
            if not self._heading_warned and elapsed > 2.0:
                self._heading_warned = True
                self.get_logger().warn(
                    'No /imu message yet - falling back to wheel odometry '
                    'for heading (pivot turns will overshoot).')
        tx, ty = self._route[self._index]
        dx, dy = tx - x, ty - y
        distance = math.hypot(dx, dy)

        if distance < self.waypoint_tolerance:
            self._advance()
            if self._finished:
                return
            tx, ty = self._route[self._index]
            dx, dy = tx - x, ty - y
            distance = math.hypot(dx, dy)

        error = wrap_angle(math.atan2(dy, dx) - yaw)
        angular = max(-self.max_angular,
                      min(self.max_angular, self.heading_gain * error))

        # Align on the spot before driving.  The hysteresis (start turning at
        # turn_in_place_angle, stop at align_done_angle) keeps the rover from
        # chattering between steering and pivoting, and guarantees a crisp
        # 90-degree corner instead of a wide arc into the block behind it.
        if self._aligning:
            if abs(error) > self.align_done_angle:
                self._publish_cmd(0.0, angular)
                return
            self._aligning = False
        elif abs(error) > self.turn_in_place_angle:
            self._aligning = True
            self._publish_cmd(0.0, angular)
            return

        # Forward speed falls off with heading error, and is ramped down as
        # the rover closes on a waypoint so corners are taken slowly.
        blend = 1.0 - 0.7 * (abs(error) / self.turn_in_place_angle)
        linear = self.cruise_speed * blend
        if distance < self.slow_radius:
            approach = distance / self.slow_radius
            linear = min(linear,
                         self.corner_speed
                         + (self.cruise_speed - self.corner_speed) * approach)
        self._publish_cmd(linear, angular)

    def _advance(self):
        wx, wy = self._route_world[self._index]
        self.get_logger().info(
            f'Waypoint {self._index + 1}/{len(self._route)} reached '
            f'(world {wx:.1f}, {wy:.1f})')
        self._index += 1
        if self._index >= len(self._route):
            if self.loop:
                self._index = 0
                self.get_logger().info('Route complete - starting next lap')
            else:
                self._finished = True
                self._publish_cmd(0.0, 0.0)
                self.get_logger().info('Route complete - stopping')

    # ------------------------------------------------------------- route
    def _publish_path(self):
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in self._route:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._path_pub.publish(path)

    def stop(self):
        if rclpy.ok():
            self._publish_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
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
