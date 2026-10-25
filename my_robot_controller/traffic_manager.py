#!/usr/bin/env python3
"""Spawn and drive simple traffic and pedestrians around the city.

    ros2 run my_robot_controller traffic_manager

Cars and pedestrians are Fuel models spawned as *static* entities and then
teleported along lane or crosswalk polylines ten times per simulated second.
Static bodies cost the physics engine nothing, yet the rover's rendering-based
LiDAR still sees them, so the planner really does have to react to moving
obstacles.  Motion is integrated in simulation time, so the traffic keeps the
same speed relative to the rover when the real-time factor drops.

Right-hand traffic: every car lane is 2.9 m to the right of a road centreline,
which keeps the rover's centreline route free while putting oncoming traffic
close enough for the LiDAR to notice.
"""

import math
import sys
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

WORLD = 'city'
CREATE_SERVICE = f'/world/{WORLD}/create/blocking'
# Harmonic exposes two services: /set_pose takes a single gz.msgs.Pose,
# /set_pose_vector takes the Pose_V batch this script sends.
SET_POSE_SERVICE = f'/world/{WORLD}/set_pose_vector'
CLOCK_TOPIC = f'/world/{WORLD}/clock'

FUEL = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/'
MODEL_URI = {
    'car': FUEL + 'Prius%20Hybrid',
    'person': FUEL + 'Walking%20person',
}

# Yaw is measured from +X in the world.  The Prius mesh points along -Y
# (its mass sits slightly aft, and the rear hatch faces +Y), the Walking
# person along +Y, so the two offsets have opposite signs.
CAR_YAW_OFFSET = math.pi / 2.0
PERSON_YAW_OFFSET = -math.pi / 2.0

CAR_SPEED = 5.5          # m/s, a relaxed city cruise
WALK_SPEED = 1.15        # m/s
UPDATE_PERIOD = 0.10     # simulation seconds between pose updates


class Actor:
    """A static model following a closed polyline at a constant speed."""

    def __init__(self, name, kind, path, speed, phase=0.0, z=0.02,
                 yaw_offset=0.0):
        self.name = name
        self.kind = kind
        self.path = path
        self.speed = speed
        self.z = z
        self.yaw_offset = yaw_offset
        self.lengths = []
        self.total = 0.0
        for (x0, y0), (x1, y1) in zip(path, path[1:] + path[:1]):
            length = math.hypot(x1 - x0, y1 - y0)
            self.lengths.append(length)
            self.total += length
        self.distance = phase * self.total

    def advance(self, dt):
        self.distance = (self.distance + self.speed * dt) % self.total
        return self.pose()

    def pose(self):
        """Interpolate position and heading at the current path distance."""
        remaining = self.distance
        for index, length in enumerate(self.lengths):
            if remaining <= length or length < 1e-9:
                start = self.path[index]
                end = self.path[(index + 1) % len(self.path)]
                ratio = remaining / length if length > 1e-9 else 0.0
                x = start[0] + (end[0] - start[0]) * ratio
                y = start[1] + (end[1] - start[1]) * ratio
                heading = math.atan2(end[1] - start[1], end[0] - start[0])
                return x, y, heading + self.yaw_offset
            remaining -= length
        # Numerical edge case: wrap to the start rather than raise.
        start = self.path[0]
        end = self.path[1 % len(self.path)]
        heading = math.atan2(end[1] - start[1], end[0] - start[0])
        return start[0], start[1], heading + self.yaw_offset


def car_loop(x0, y0, x1, y1, lane=2.9):
    """Counter-clockwise loop around a block, offset to the right-hand lane.

    The four lane segments are joined across each intersection, so the car
    takes a short diagonal through the junction just like a real turn.
    """
    return [
        (x0, y0 - lane), (x1, y0 - lane), (x1 + lane, y0),
        (x1 + lane, y1), (x1, y1 + lane), (x0, y1 + lane),
        (x0 - lane, y1), (x0 - lane, y0),
    ]


def build_actors():
    actors = []

    # Two cars circulating the two central blocks, plus one on the outer
    # block, all in the right-hand lane.
    loop_ab = car_loop(0.0, 0.0, 30.0, 30.0)
    loop_bc = car_loop(-30.0, 0.0, 0.0, 30.0)
    loop_cd = car_loop(30.0, 0.0, 60.0, 30.0)
    for name, loop, phase in (
            ('traffic_car_0', loop_ab, 0.05),
            ('traffic_car_1', loop_ab, 0.55),
            ('traffic_car_2', loop_bc, 0.30),
            ('traffic_car_3', loop_bc, 0.80),
            ('traffic_car_4', loop_cd, 0.45)):
        actors.append(Actor(name, 'car', loop, CAR_SPEED, phase,
                            yaw_offset=CAR_YAW_OFFSET))

    # Pedestrians: two on the central zebra crossings (they walk right across
    # the rover's route), two strolling the park paths.
    peds = [
        ('pedestrian_0', [(-6.0, -5.6), (6.0, -5.6)], 0.10),
        ('pedestrian_1', [(5.6, -6.0), (5.6, 6.0)], 0.60),
        ('pedestrian_2', [(45.0, 35.0), (45.0, 55.0)], 0.25),
        ('pedestrian_3', [(-45.0, -35.0), (-45.0, -55.0)], 0.70),
    ]
    for name, path, phase in peds:
        # A two-point closed path walks out and back.
        actors.append(Actor(name, 'person', path, WALK_SPEED, phase,
                            z=0.03, yaw_offset=PERSON_YAW_OFFSET))
    return actors


def spawn(node, actor):
    request = EntityFactory()
    request.name = actor.name
    request.sdf = (
        '<sdf version="1.9"><include>'
        f'<uri>{MODEL_URI[actor.kind]}</uri>'
        f'<name>{actor.name}</name>'
        '<pose>0 0 0 0 0 0</pose>'
        '<static>true</static>'
        '</include></sdf>'
    )
    # The Walking person mesh is ~26 MB, so a cold spawn can outlast a short
    # timeout even though the entity does appear; retry rather than give up.
    for _ in range(3):
        ok, _ = node.request(CREATE_SERVICE, request, EntityFactory, Boolean,
                             15000)
        if ok:
            return True
        time.sleep(0.5)
    print(f'warning: spawn of {actor.name} was not acknowledged', flush=True)
    return False


def pose_message(actors):
    message = Pose_V()
    for actor in actors:
        x, y, yaw = actor.pose()
        pose = message.pose.add()
        pose.name = actor.name
        pose.position.x = x
        pose.position.y = y
        pose.position.z = actor.z
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
    return message


def main():
    node = Node()
    actors = build_actors()

    print(f'traffic manager: {len(actors)} actors in world "{WORLD}"', flush=True)
    # The launch file starts this before the city has finished loading, so
    # retry until the create service answers, then spawn the rest.
    deadline = time.time() + 120.0
    while time.time() < deadline:
        if spawn(node, actors[0]):
            break
        time.sleep(1.0)
    else:
        print('error: city never became available', flush=True)
        return 1
    for actor in actors[1:]:
        spawn(node, actor)

    # Give the factory a moment, then set every actor to its start pose.
    time.sleep(0.5)

    sim_now = None
    last_update = None
    # A slow model can still be loading when the factory returns, so keep
    # asking until the whole batch can be placed.
    deadline = time.time() + 30.0
    while time.time() < deadline:
        ok, _ = node.request(SET_POSE_SERVICE, pose_message(actors), Pose_V,
                             Boolean, 5000)
        if ok:
            print('traffic manager: initial poses applied', flush=True)
            break
        time.sleep(1.0)
    else:
        print('warning: set_pose_vector was not acknowledged', flush=True)

    def on_clock(message):
        nonlocal sim_now
        # gz Clock carries gz msgs Time: header, system, real, sim.
        sim_now = message.sim.sec + message.sim.nsec * 1e-9

    node.subscribe(Clock, CLOCK_TOPIC, on_clock)

    try:
        while True:
            time.sleep(0.02)
            if sim_now is None:
                continue
            if last_update is None:
                last_update = sim_now
                continue
            dt = sim_now - last_update
            if dt < UPDATE_PERIOD:
                continue
            # Clamp: a paused or stalled simulation must not teleport traffic.
            dt = min(dt, 0.5)
            last_update = sim_now
            for actor in actors:
                actor.advance(dt)
            node.request(SET_POSE_SERVICE, pose_message(actors), Pose_V,
                         Boolean, 100)
    except KeyboardInterrupt:
        print('traffic manager: stopped', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
