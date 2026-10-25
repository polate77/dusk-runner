#!/usr/bin/env python3
"""Spawn a temporary cinematic camera rig behind the rover.

    ros2 run my_robot_controller photo_rig \
        --output ~/closeup.png --wait-near 0,8,6

The rover only carries forward-facing cameras, so a third-person shot needs a
camera that is not part of the robot.  This script spawns a static,
geometry-free model with a 1920x1080 camera, locks it to a point behind and
above the rover (Pose_V at about 7 Hz), saves the requested frames and removes
the rig again.  Nothing is left in the world and the rover's URDF is
untouched, so the framing can be tuned by simply running the command again.

``--back`` / ``--up`` / ``--pitch`` / ``--fov`` control the shot.  The defaults
put the camera 3.2 m behind and 1.5 m above the robot, angled 14 degrees down
with a 57 degree horizontal field of view: the rover sits in the lower middle
of the frame with the street ahead still readable.
"""

import argparse
import math
import sys
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
from PIL import Image as PilImage

WORLD = 'city'
CREATE_SERVICE = f'/world/{WORLD}/create/blocking'
REMOVE_SERVICE = f'/world/{WORLD}/remove'
SET_POSE_SERVICE = f'/world/{WORLD}/set_pose_vector'
POSE_TOPIC = f'/world/{WORLD}/pose/info'
RIG_TOPIC = '/photo/rig'
ROVER = 'my_robot'
RIG = 'photo_rig'


def parse_triple(text):
    parts = [float(v) for v in text.split(',')]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError('expected X,Y,R')
    return parts


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rig_pose(rover_x, rover_y, rover_z, yaw, back, up, pitch):
    """World pose for a camera looking at the rover from behind and above."""
    x = rover_x - back * math.cos(yaw)
    y = rover_y - back * math.sin(yaw)
    z = rover_z + up
    # q = q_yaw * q_pitch: yaw about world Z, then pitch about the camera's
    # own Y (positive pitch looks down, since the camera faces +X).
    sy, cy = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    sp, cp = math.sin(pitch / 2.0), math.cos(pitch / 2.0)
    qx = -sy * sp
    qy = cy * sp
    qz = sy * cp
    qw = cy * cp
    return x, y, z, qx, qy, qz, qw


def rig_sdf(width, height, fov, x, y, z, pitch, yaw):
    return (
        '<sdf version="1.9"><model name="photo_rig">'
        '<static>true</static>'
        f'<pose>{x:.4f} {y:.4f} {z:.4f} 0 {pitch:.4f} {yaw:.4f}</pose>'
        '<link name="mount">'
        '<sensor name="rig_camera" type="camera">'
        f'<topic>{RIG_TOPIC}</topic>'
        '<update_rate>15</update_rate>'
        '<always_on>true</always_on>'
        '<visualize>false</visualize>'
        '<camera>'
        f'<horizontal_fov>{fov:.4f}</horizontal_fov>'
        f'<image><width>{width}</width><height>{height}</height></image>'
        '<clip><near>0.1</near><far>300</far></clip>'
        '</camera>'
        '</sensor>'
        '</link>'
        '</model></sdf>'
    )


def remove_rig(node):
    request = Entity()
    request.name = RIG
    request.type = Entity.MODEL
    ok, _ = node.request(REMOVE_SERVICE, request, Entity, Boolean, 5000)
    return ok


def spawn_rig(node, sdf, state):
    # A rig left over from an interrupted run would make the factory refuse
    # the name, so clear it first.
    remove_rig(node)
    time.sleep(0.4)
    state['rig'] = False
    for _ in range(3):
        request = EntityFactory()
        request.name = RIG
        request.sdf = sdf
        ok, _ = node.request(CREATE_SERVICE, request, EntityFactory, Boolean,
                             20000)
        if ok:
            return True
        # While another 1080p camera is initialising, the factory can time out
        # even though the model is created, so accept it once it shows up in
        # the pose stream.
        deadline = time.time() + 6.0
        while time.time() < deadline:
            if state['rig']:
                return True
            time.sleep(0.2)
    return False


def pose_message(x, y, z, qx, qy, qz, qw):
    message = Pose_V()
    pose = message.pose.add()
    pose.name = RIG
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return message


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
    parser.add_argument('--output', required=True, help='PNG file to write')
    parser.add_argument('--back', type=float, default=3.2,
                        help='metres behind the rover')
    parser.add_argument('--up', type=float, default=1.5,
                        help='metres above the rover origin')
    parser.add_argument('--pitch', type=float, default=14.0,
                        help='downward tilt in degrees')
    parser.add_argument('--fov', type=float, default=57.0,
                        help='horizontal field of view in degrees')
    parser.add_argument('--width', type=int, default=1920)
    parser.add_argument('--height', type=int, default=1080)
    parser.add_argument('--wait-near', type=parse_triple, default=None,
                        metavar='X,Y,R',
                        help='wait until my_robot is within R metres of (X, Y) '
                             'before spawning the rig')
    parser.add_argument('--wait-timeout', type=float, default=300.0,
                        help='how long to wait for --wait-near')
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='give up after this many seconds')
    parser.add_argument('--warmup', type=float, default=1.8,
                        help='discard frames for this long while the sensor '
                             'settles')
    parser.add_argument('--count', type=int, default=1,
                        help='number of frames to save')
    parser.add_argument('--interval', type=float, default=0.6,
                        help='seconds between saved frames')
    args = parser.parse_args()

    state = {'pose': None, 'rig': False}
    latest = {'frame': None}

    def on_pose(message):
        robot = None
        for pose in message.pose:
            if pose.name == RIG:
                state['rig'] = True
            elif pose.name == ROVER:
                robot = pose
        if robot is None:
            return
        state['pose'] = (robot.position.x, robot.position.y,
                         robot.position.z,
                         yaw_from_quaternion(robot.orientation))

    def on_image(msg):
        latest['frame'] = msg

    node = Node()
    if not node.subscribe(Pose_V, POSE_TOPIC, on_pose):
        print(f'error: could not subscribe to {POSE_TOPIC}', file=sys.stderr)
        return 1
    if not node.subscribe(Image, RIG_TOPIC, on_image):
        print(f'error: could not subscribe to {RIG_TOPIC}', file=sys.stderr)
        return 1

    deadline = time.time() + 30.0
    while state['pose'] is None and time.time() < deadline:
        time.sleep(0.1)
    if state['pose'] is None:
        print('error: no pose for my_robot', file=sys.stderr)
        return 1

    if args.wait_near is not None:
        target_x, target_y, radius = args.wait_near
        print('waiting for the rover to reach the shot...', flush=True)
        deadline = time.time() + args.wait_timeout
        while time.time() < deadline:
            x, y, _, _ = state['pose']
            if math.hypot(x - target_x, y - target_y) <= radius:
                break
            time.sleep(0.1)
        else:
            print('error: rover never reached the shot', file=sys.stderr)
            return 1

    x, y, z, yaw = state['pose']
    pitch = math.radians(args.pitch)
    fov = math.radians(args.fov)
    pose = rig_pose(x, y, z, yaw, args.back, args.up, pitch)
    sdf = rig_sdf(args.width, args.height, fov, pose[0], pose[1], pose[2],
                  pitch, yaw)
    if not spawn_rig(node, sdf, state):
        print('error: could not spawn the camera rig', file=sys.stderr)
        return 1
    print(f'photo rig: spawned behind the rover at ({pose[0]:.1f}, '
          f'{pose[1]:.1f}, {pose[2]:.1f})', flush=True)

    start = time.time()
    deadline = start + args.timeout
    saved = 0
    last_save = 0.0
    last_pose = 0.0
    try:
        while saved < args.count and time.time() < deadline:
            now = time.time()
            if now - last_pose >= 0.14:
                last_pose = now
                rx, ry, rz, ryaw = state['pose']
                px, py, pz, qx, qy, qz, qw = rig_pose(
                    rx, ry, rz, ryaw, args.back, args.up, pitch)
                node.request(SET_POSE_SERVICE,
                             pose_message(px, py, pz, qx, qy, qz, qw),
                             Pose_V, Boolean, 60)

            msg = latest['frame']
            if (msg is not None and now - start >= args.warmup
                    and (not saved or now - last_save >= args.interval)):
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
            time.sleep(0.04)
    finally:
        remove_rig(node)

    if not saved:
        print(f'error: no frame on {RIG_TOPIC} within {args.timeout:.0f}s',
              file=sys.stderr)
        return 1
    print('photo rig: removed', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
