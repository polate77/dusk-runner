#!/usr/bin/env python3
"""Generate ``worlds/my_city.world`` - a golden-hour city for Gazebo Harmonic.

Deterministic (fixed random seed).  Re-run any time with:

    python3 scripts/generate_city.py

Layout
------
* 5 x 5 road grid (30 m pitch, 8 m wide) with asphalt, edge lines, lane dashes
  and a zebra crossing at the central intersection.
* 16 city blocks: 14 hold buildings, 2 are parks with a pond, paths, benches
  and real trees.
* Downtown blocks use detailed procedural towers; five blocks use the
  photorealistic ``Apartment`` model from Gazebo Fuel to break up the skyline.
* Oak and pine trees, park benches and parked cars are downloaded from
  Gazebo Fuel the first time this script runs and then cached under
  ``~/.gz/fuel``.  Pass ``--no-fuel`` to generate the box-only fallback world
  (useful offline).
* Golden-hour lighting: a low, warm sun, a cool sky fill, lit windows and
  warm street lamps.  Long shadows sell the time of day.

The GUI is configured with the CameraTracking plugin following ``my_robot``
so the 3D view chases the rover without any mouse input.
"""

import argparse
import json
import math
import os
import random
import re
import urllib.parse
import urllib.request
import zipfile

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_PATH = os.path.join(PKG_DIR, 'worlds', 'my_city.world')

ROADS = (-60.0, -30.0, 0.0, 30.0, 60.0)
ROAD_HALF = 4.0
WORLD_HALF = 80.0
ROAD_LEN = 2.0 * WORLD_HALF
BLOCK_CENTRES = [(-45.0 + 30.0 * i, -45.0 + 30.0 * j)
                 for i in range(4) for j in range(4)]
PARKS = {(45.0, 45.0), (-45.0, -45.0)}
# The four blocks around the central intersection, plus the block next to the
# north-east park, are rebuilt with the Fuel apartment model.
APARTMENT_BLOCKS = {(-15.0, 15.0), (15.0, 15.0), (-15.0, -15.0),
                    (15.0, -15.0), (15.0, 45.0)}

RNG = random.Random(20261025)

# ------------------------------------------------------------- Fuel assets
FUEL_SERVER = 'https://fuel.gazebosim.org/1.0'
FUEL_OWNER = 'OpenRobotics'
FUEL_ASSETS = {
    'oak': 'Oak tree',
    'pine': 'Pine Tree',
    'bench': 'FoodCourtBenchLong',
    'car': 'Prius Hybrid',
    'apartment': 'Apartment',
    'person': 'Walking person',
}
# Keep the download bounded: these are public, but a corrupt or hostile zip
# should never be able to fill the disk.
MAX_MODEL_BYTES = 60 * 1024 * 1024


def fuel_uri(key):
    name = urllib.parse.quote(FUEL_ASSETS[key])
    return f'{FUEL_SERVER}/{FUEL_OWNER}/models/{name}'


def fuel_available(timeout=8):
    """True if the Fuel catalogue answers; decides Fuel vs procedural."""
    try:
        with urllib.request.urlopen(fuel_uri('oak'), timeout=timeout):
            return True
    except Exception:
        return False


def _safe_extract(zip_path, dest):
    """Extract a Fuel zip, refusing absolute paths and ``..`` escapes."""
    dest = os.path.realpath(dest)
    with zipfile.ZipFile(zip_path) as archive:
        for member in archive.infolist():
            target = os.path.realpath(os.path.join(dest, member.filename))
            if target != dest and not target.startswith(dest + os.sep):
                raise RuntimeError(f'unsafe path in model zip: {member.filename}')
        archive.extractall(dest)


def _fix_dae_textures(model_dir):
    """Make bare Collada image names resolvable.

    Fuel's Apartment DAE names its textures ``apartment_diffuse.jpg`` with no
    path, which Gazebo's Collada loader cannot find; models that write
    ``../materials/textures/<file>`` (the Walking person) load fine.  Rewrite
    the bare names in place whenever the asset is cached, so the repair
    survives restarts and re-runs.
    """
    texture_dir = os.path.join(model_dir, 'materials', 'textures')
    mesh_dir = os.path.join(model_dir, 'meshes')
    if not (os.path.isdir(texture_dir) and os.path.isdir(mesh_dir)):
        return
    for entry in os.listdir(mesh_dir):
        if not entry.endswith('.dae'):
            continue
        path = os.path.join(mesh_dir, entry)
        with open(path, encoding='utf-8', errors='ignore') as handle:
            text = handle.read()

        def repair(match):
            ref = match.group(1)
            if '/' in ref or '\\' in ref:
                return match.group(0)       # already relative or absolute
            if not os.path.isfile(os.path.join(texture_dir, ref)):
                return match.group(0)       # a Collada image id, not a file
            return f'<init_from>../materials/textures/{ref}</init_from>'

        fixed = re.sub(r'<init_from>([^<]+)</init_from>', repair, text)
        if fixed != text:
            with open(path, 'w', encoding='utf-8') as handle:
                handle.write(fixed)


def _repair_cached_models(root):
    for name in os.listdir(root):
        model_dir = os.path.join(root, name)
        if not os.path.isdir(model_dir):
            continue
        for version in os.listdir(model_dir):
            version_dir = os.path.join(model_dir, version)
            if os.path.isdir(version_dir):
                _fix_dae_textures(version_dir)


def prewarm_fuel(keys, cache_root=None):
    """Download the Fuel models into the cache Gazebo itself reads.

    ``gz`` stores models as ``~/.gz/fuel/fuel.gazebosim.org/<owner>/models/
    <name>/<version>``.  Filling that layout here means the first launch does
    not have to reach the network, and the generator is the thing that
    downloads the assets (the world file only references Fuel URIs).
    """
    cache_root = cache_root or os.path.expanduser('~/.gz/fuel')
    root = os.path.join(cache_root, 'fuel.gazebosim.org', FUEL_OWNER.lower(),
                        'models')
    for key in keys:
        name = FUEL_ASSETS[key]
        dest_root = os.path.join(root, name.lower())
        if os.path.isdir(dest_root) and any(os.scandir(dest_root)):
            print(f'  fuel: {name} (cached)')
            continue
        qname = urllib.parse.quote(name)
        meta_url = f'{FUEL_SERVER}/{FUEL_OWNER}/models/{qname}'
        try:
            with urllib.request.urlopen(meta_url, timeout=20) as response:
                meta = json.loads(response.read())
            version = meta['version']
            zip_url = f'{meta_url}/{version}/{qname}.zip'
            with urllib.request.urlopen(zip_url, timeout=180) as response:
                payload = response.read(MAX_MODEL_BYTES + 1)
            if len(payload) > MAX_MODEL_BYTES:
                raise RuntimeError('model archive exceeds size limit')
            os.makedirs(dest_root, exist_ok=True)
            zip_path = os.path.join(dest_root, f'v{version}.zip')
            with open(zip_path, 'wb') as handle:
                handle.write(payload)
            _safe_extract(zip_path, os.path.join(dest_root, str(version)))
            os.remove(zip_path)
            print(f'  fuel: {name} v{version} ({len(payload) / 1e6:.1f} MB)')
        except Exception as exc:                       # noqa: BLE001
            print(f'  fuel: could not fetch {name}: {exc}')
    if os.path.isdir(root):
        _repair_cached_models(root)


# ---------------------------------------------------------------- primitives


def num(value):
    """Compact fixed-point formatting for SDF numbers."""
    if abs(value) < 1e-9:
        return '0'
    return f'{value:.3f}'.rstrip('0').rstrip('.')


def rgba(r, g, b, a=1.0):
    return f'{num(r)} {num(g)} {num(b)} {num(a)}'


def material(ambient, diffuse=None, specular=(0.18, 0.18, 0.18), emissive=None,
             roughness=None, metalness=None):
    """SDF material, optionally with a physically based metal/roughness pair."""
    diffuse = diffuse if diffuse is not None else ambient
    lines = [
        '<material>',
        f'<ambient>{rgba(*ambient)}</ambient>',
        f'<diffuse>{rgba(*diffuse)}</diffuse>',
        f'<specular>{rgba(*specular)}</specular>',
    ]
    if emissive is not None:
        lines.append(f'<emissive>{rgba(*emissive)}</emissive>')
    if roughness is not None or metalness is not None:
        lines.extend([
            '<pbr>',
            '<metal>',
            f'<roughness>{num(roughness if roughness is not None else 0.8)}</roughness>',
            f'<metalness>{num(metalness if metalness is not None else 0.0)}</metalness>',
            '</metal>',
            '</pbr>',
        ])
    lines.append('</material>')
    return '\n          '.join(lines)


def visual(name, pose, geometry, mat):
    return (f'<visual name="{name}">\n'
            f'          <pose>{pose}</pose>\n'
            f'          <geometry>{geometry}</geometry>\n'
            f'          {mat}\n'
            f'        </visual>')


def box(size):
    return f'<box><size>{size}</size></box>'


def cylinder(radius, length):
    return f'<cylinder><radius>{num(radius)}</radius><length>{num(length)}</length></cylinder>'


def sphere(radius):
    return f'<sphere><radius>{num(radius)}</radius></sphere>'


# ------------------------------------------------------------------ palette
# The ground materials stay on the cheap classic model: they cover most of
# the screen, and PBR shading there costs ~20% of the software frame budget.
ASPHALT = material((0.105, 0.105, 0.115), specular=(0.03, 0.03, 0.03))
PAINT = material((0.86, 0.85, 0.80), specular=(0.12, 0.12, 0.12))
CURB = material((0.62, 0.61, 0.58))
WALK = material((0.71, 0.70, 0.67))
GRASS = material((0.17, 0.33, 0.13), (0.21, 0.41, 0.16),
                 specular=(0.05, 0.06, 0.04))
LAWN = material((0.24, 0.44, 0.18), (0.28, 0.50, 0.21),
                specular=(0.05, 0.06, 0.04))
TRUNK = material((0.29, 0.20, 0.11), roughness=0.9)
LEAF = material((0.15, 0.32, 0.12), (0.19, 0.38, 0.15),
                specular=(0.05, 0.05, 0.05), roughness=0.85)
LEAF_DARK = material((0.10, 0.25, 0.10), (0.14, 0.30, 0.13),
                     specular=(0.05, 0.05, 0.05), roughness=0.85)
LEAF_AUTUMN = material((0.52, 0.26, 0.06), (0.66, 0.35, 0.08),
                       specular=(0.05, 0.05, 0.05), roughness=0.85)
BENCH = material((0.26, 0.17, 0.09), roughness=0.8)
METAL_DARK = material((0.17, 0.18, 0.19), specular=(0.30, 0.30, 0.32),
                      roughness=0.45, metalness=0.35)
LAMP_GLOW = material((1.0, 0.90, 0.62), (1.0, 0.90, 0.62),
                     specular=(0.4, 0.4, 0.4), emissive=(1.0, 0.78, 0.42))
STONE = material((0.44, 0.42, 0.39), roughness=0.85)
WATER = material((0.08, 0.24, 0.34), (0.12, 0.32, 0.44),
                 specular=(0.7, 0.7, 0.7), roughness=0.08, metalness=0.30)

BUILDING_COLOURS = [
    (0.80, 0.77, 0.72),
    (0.55, 0.62, 0.72),
    (0.74, 0.63, 0.52),
    (0.42, 0.50, 0.58),
    (0.86, 0.82, 0.74),
    (0.34, 0.38, 0.44),
    (0.68, 0.50, 0.40),
    (0.60, 0.66, 0.62),
    (0.78, 0.72, 0.60),
    (0.47, 0.42, 0.40),
    (0.30, 0.36, 0.40),
    (0.88, 0.86, 0.82),
]

# Accent colour for cornices, canopies and corner columns.
ACCENTS = [
    (0.85, 0.33, 0.09),
    (0.10, 0.42, 0.55),
    (0.56, 0.12, 0.14),
    (0.18, 0.48, 0.28),
    (0.88, 0.70, 0.20),
]

# ------------------------------------------------------------------ builders


def fuel_include(key, name, x, y, z=0.0, yaw=0.0, scale=1.0):
    """A Fuel model referenced by URI.

    Gz downloads and caches it on first use; ``generate_city.py`` pre-warms
    the same cache so a normal launch never waits on the network.
    """
    scale_tag = ''
    if abs(scale - 1.0) > 1e-6:
        scale_tag = (f'\n      <scale>{num(scale)} {num(scale)} '
                     f'{num(scale)}</scale>')
    return f'''
    <include>
      <uri>{fuel_uri(key)}</uri>
      <name>{name}</name>
      <pose>{num(x)} {num(y)} {num(z)} 0 0 {num(yaw)}</pose>
      <static>true</static>{scale_tag}
    </include>'''


def building_model(name, x, y, w, d, h, colour, accent, rnd):
    """One building: podium, glass bands, cornice, roof and rooftop details."""
    parts = [
        visual('body', f'0 0 {num(h / 2)} 0 0 0',
               box(f'{num(w)} {num(d)} {num(h)}'),
               material(colour, specular=(0.16, 0.16, 0.18),
                        roughness=0.62, metalness=0.08)),
        visual('podium', '0 0 1.1 0 0 0',
               box(f'{num(w + 0.10)} {num(d + 0.10)} 2.2'),
               material((0.07, 0.09, 0.11), (0.10, 0.13, 0.16),
                        specular=(0.45, 0.47, 0.50),
                        roughness=0.22, metalness=0.40)),
        # Entrance canopy and a slim accent column on the street corner.
        visual('canopy', f'0 {num(-d / 2 - 0.40)} 2.55 0 0 0',
               box(f'{num(w * 0.55)} 0.90 0.18'),
               material(accent, specular=(0.25, 0.25, 0.25), roughness=0.45)),
        visual('corner_column',
               f'{num(w / 2 + 0.04)} {num(d / 2 + 0.04)} {num(h / 2)} 0 0 0',
               box(f'0.16 0.16 {num(h)}'),
               material(accent, specular=(0.25, 0.25, 0.25), roughness=0.45)),
    ]

    z = 3.3
    floor = 0
    while z + 0.7 < h - 1.0:
        # At dusk roughly half the offices are still lit.
        lit = rnd.random() < 0.32
        emissive = (0.46, 0.30, 0.12) if lit else (0.06, 0.09, 0.12)
        parts.append(visual(
            f'glass_{floor}', f'0 0 {num(z)} 0 0 0',
            box(f'{num(w + 0.06)} {num(d + 0.06)} 1.05'),
            material((0.075, 0.095, 0.115), (0.10, 0.13, 0.16),
                     specular=(0.55, 0.60, 0.65), emissive=emissive,
                     roughness=0.10, metalness=0.55)))
        z += 3.0
        floor += 1

    # Accent cornice just below the roof line.
    parts.append(visual('cornice', f'0 0 {num(h - 0.55)} 0 0 0',
                        box(f'{num(w + 0.50)} {num(d + 0.50)} 0.30'),
                        material(accent, specular=(0.2, 0.2, 0.2),
                                 roughness=0.55)))

    parts.append(visual('roof', f'0 0 {num(h + 0.15)} 0 0 0',
                        box(f'{num(w + 0.30)} {num(d + 0.30)} 0.30'),
                        material((0.24, 0.24, 0.25), roughness=0.85)))
    if rnd.random() < 0.35:
        parts.append(visual('roof_garden', f'0 0 {num(h + 0.34)} 0 0 0',
                            box(f'{num(w * 0.55)} {num(d * 0.55)} 0.16'),
                            LAWN))
    if h > 15.0:
        parts.append(visual('roof_unit', f'0 0 {num(h + 0.95)} 0 0 0',
                            box(f'{num(w * 0.42)} {num(d * 0.42)} 1.30'),
                            material((0.31, 0.31, 0.33), roughness=0.55,
                                     metalness=0.30)))
    if h > 23.0:
        parts.append(visual('antenna', f'0 0 {num(h + 2.3)} 0 0 0',
                            cylinder(0.07, 1.6),
                            material((0.62, 0.62, 0.64), roughness=0.35,
                                     metalness=0.55)))

    return f'''
    <model name="{name}">
      <static>true</static>
      <pose>{num(x)} {num(y)} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <pose>0 0 {num(h / 2)} 0 0 0</pose>
          <geometry>{box(f'{num(w)} {num(d)} {num(h)}')}</geometry>
        </collision>
        {''.join(parts)}
      </link>
    </model>'''


def streetlight_model(name, x, y, yaw, with_light):
    parts = [
        visual('foundation', '0 0 0.15 0 0 0', cylinder(0.18, 0.30), METAL_DARK),
        visual('pole', '0 0 3.0 0 0 0', cylinder(0.075, 6.0), METAL_DARK),
        visual('arm', '0.55 0 5.95 0 0 0', box('1.10 0.09 0.09'), METAL_DARK),
        visual('head', '1.05 0 5.83 0 0 0', box('0.42 0.20 0.10'), LAMP_GLOW),
    ]

    light = ''
    if with_light:
        light = '''
        <light type="point" name="lamp">
          <pose>1.05 0 5.70 0 0 0</pose>
          <diffuse>1.0 0.80 0.52 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
          <attenuation>
            <range>18</range>
            <constant>1.0</constant>
            <linear>0.08</linear>
            <quadratic>0.015</quadratic>
          </attenuation>
          <cast_shadows>false</cast_shadows>
        </light>'''

    return f'''
    <model name="{name}">
      <static>true</static>
      <pose>{num(x)} {num(y)} 0 0 0 {num(yaw)}</pose>
      <link name="link">
        {''.join(parts)}{light}
      </link>
    </model>'''


def tree_model(name, x, y, rnd):
    """Procedural fallback tree (used with --no-fuel or an offline machine)."""
    scale = rnd.uniform(0.85, 1.25)
    trunk_h = 2.4 * scale
    canopy_r = 1.35 * scale
    roll = rnd.random()
    leaf = LEAF_AUTUMN if roll < 0.15 else (LEAF_DARK if roll < 0.55 else LEAF)
    return f'''
    <model name="{name}">
      <static>true</static>
      <pose>{num(x)} {num(y)} 0 0 0 0</pose>
      <link name="link">
        {visual('trunk', f'0 0 {num(trunk_h / 2)} 0 0 0', cylinder(0.14 * scale, trunk_h), TRUNK)}
        {visual('canopy', f'0 0 {num(trunk_h + canopy_r * 0.75)} 0 0 0', sphere(canopy_r), leaf)}
        {visual('canopy_top', f'0 0 {num(trunk_h + canopy_r * 1.45)} 0 0 0',
                sphere(canopy_r * 0.72), leaf)}
      </link>
    </model>'''


def bench(name, x, y, yaw, use_fuel):
    if use_fuel:
        return fuel_include('bench', name, x, y, 0.07, yaw)
    seat = visual('seat', f'0 0 0.52 0 0 {num(yaw)}', box('1.7 0.48 0.09'), BENCH)
    back = visual('back', f'0 0 0.82 0 0 {num(yaw)}', box('1.7 0.10 0.50'), BENCH)
    return f'''
    <model name="{name}">
      <static>true</static>
      <pose>{num(x)} {num(y)} 0 0 0 0</pose>
      <link name="link">
        {seat}
        {back}
      </link>
    </model>'''


def segments():
    """Road intervals between intersections (plus the two outer stubs)."""
    out = [(ROADS[i] + ROAD_HALF, ROADS[i + 1] - ROAD_HALF)
           for i in range(len(ROADS) - 1)]
    out.append((-WORLD_HALF, ROADS[0] - ROAD_HALF))
    out.append((ROADS[-1] + ROAD_HALF, WORLD_HALF))
    return [s for s in out if s[1] - s[0] > 1.0]


def road_parts():
    parts = []
    for i, g in enumerate(ROADS):
        parts.append(visual(f'asphalt_h_{i}', f'0 {num(g)} 0.012 0 0 0',
                            box(f'{num(ROAD_LEN)} {num(2 * ROAD_HALF)} 0.02'), ASPHALT))
        parts.append(visual(f'asphalt_v_{i}', f'{num(g)} 0 0.012 0 0 0',
                            box(f'{num(2 * ROAD_HALF)} {num(ROAD_LEN)} 0.02'), ASPHALT))

        for side in (-1, 1):
            offset = g + side * (ROAD_HALF - 0.35)
            for j, (a, b) in enumerate(segments()):
                parts.append(visual(
                    f'edge_h_{i}_{side}_{j}', f'{num((a + b) / 2)} {num(offset)} 0.028 0 0 0',
                    box(f'{num(b - a)} 0.12 0.01'), PAINT))
                parts.append(visual(
                    f'edge_v_{i}_{side}_{j}', f'{num(offset)} {num((a + b) / 2)} 0.028 0 0 0',
                    box(f'0.12 {num(b - a)} 0.01'), PAINT))

        for j, (a, b) in enumerate(segments()):
            p = a + 2.0
            k = 0
            while p + 3.0 <= b:
                centre = p + 1.5
                parts.append(visual(
                    f'dash_h_{i}_{j}_{k}', f'{num(centre)} {num(g)} 0.030 0 0 0',
                    box('3.0 0.14 0.01'), PAINT))
                parts.append(visual(
                    f'dash_v_{i}_{j}_{k}', f'{num(g)} {num(centre)} 0.030 0 0 0',
                    box('0.14 3.0 0.01'), PAINT))
                p += 8.0
                k += 1

    # zebra crossing on the four approaches of the central intersection
    for idx, (sx, sy) in enumerate(((0, 1), (0, -1), (1, 0), (-1, 0))):
        for k in range(7):
            if sy:
                x = -3.6 + 1.2 * k
                pose = f'{num(x)} {num(sy * 5.6)} 0.030 0 0 0'
                geo = box('0.62 2.6 0.01')
            else:
                y = -3.6 + 1.2 * k
                pose = f'{num(sx * 5.6)} {num(y)} 0.030 0 0 0'
                geo = box('2.6 0.62 0.01')
            parts.append(visual(f'zebra_{idx}_{k}', pose, geo, PAINT))
    return parts


def block_parts():
    parts = []
    for idx, (cx, cy) in enumerate(BLOCK_CENTRES):
        if (cx, cy) in PARKS:
            parts.append(visual(f'park_{idx}', f'{num(cx)} {num(cy)} 0.025 0 0 0',
                                box('22.4 22.4 0.05'), GRASS))
            # Crossing footpaths and a pond set off the centre.
            parts.append(visual(f'path_x_{idx}', f'{num(cx)} {num(cy)} 0.062 0 0 0',
                                box('22.2 2.0 0.02'), WALK))
            parts.append(visual(f'path_y_{idx}', f'{num(cx)} {num(cy)} 0.064 0 0 0',
                                box('2.0 22.2 0.02'), WALK))
            px, py = cx + 4.5, cy + 4.5
            parts.append(visual(f'pond_rim_{idx}', f'{num(px)} {num(py)} 0.14 0 0 0',
                                cylinder(3.0, 0.28), STONE))
            parts.append(visual(f'pond_{idx}', f'{num(px)} {num(py)} 0.19 0 0 0',
                                cylinder(2.6, 0.06), WATER))
        else:
            parts.append(visual(f'curb_{idx}', f'{num(cx)} {num(cy)} 0.020 0 0 0',
                                box('22.6 22.6 0.04'), CURB))
            parts.append(visual(f'walk_{idx}', f'{num(cx)} {num(cy)} 0.045 0 0 0',
                                box('22.0 22.0 0.05'), WALK))
            parts.append(visual(f'lawn_{idx}', f'{num(cx)} {num(cy)} 0.062 0 0 0',
                                box('18.0 18.0 0.03'), LAWN))
            for k, (ox, oy, sx, sy) in enumerate(((0.0, 9.9, 18.0, 1.1),
                                                  (0.0, -9.9, 18.0, 1.1),
                                                  (9.9, 0.0, 1.1, 18.0),
                                                  (-9.9, 0.0, 1.1, 18.0))):
                parts.append(visual(f'planting_{idx}_{k}',
                                    f'{num(cx + ox)} {num(cy + oy)} 0.085 0 0 0',
                                    box(f'{num(sx)} {num(sy)} 0.07'), GRASS))
    return parts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-fuel', action='store_true',
                        help='generate the box-only world, without Fuel assets')
    parser.add_argument('--no-prewarm', action='store_true',
                        help='skip downloading models into the gz cache')
    args = parser.parse_args()

    use_fuel = not args.no_fuel and fuel_available()
    if use_fuel and not args.no_prewarm:
        print('pre-warming Gazebo Fuel cache...')
        prewarm_fuel(['oak', 'pine', 'bench', 'car', 'apartment', 'person'])
    elif not use_fuel:
        print('Fuel unavailable - falling back to procedural assets')

    models = []

    # ---------------------------------------------------------- buildings
    b_index = 0
    for block_index, (cx, cy) in enumerate(BLOCK_CENTRES):
        if (cx, cy) in PARKS:
            continue
        if use_fuel and (cx, cy) in APARTMENT_BLOCKS:
            # One photoreal mid-rise fills the block; rotate for variety.
            yaw = (block_index % 4) * math.pi / 2.0
            models.append(fuel_include(
                'apartment', f'apartment_{block_index}', cx, cy, 0.0, yaw))
            continue
        for dx, dy in ((-5.5, -5.5), (5.5, 5.5)):
            w = RNG.uniform(6.5, 9.2)
            d = RNG.uniform(6.5, 9.2)
            h = RNG.uniform(7.0, 31.0)
            colour = BUILDING_COLOURS[(block_index * 2 + b_index)
                                      % len(BUILDING_COLOURS)]
            accent = ACCENTS[(block_index + b_index) % len(ACCENTS)]
            models.append(building_model(
                f'building_{b_index}', cx + dx, cy + dy, w, d, h, colour,
                accent, random.Random(1000 + b_index)))
            b_index += 1

    # -------------------------------------------------------- streetlights
    sl_index = 0
    inner = [g for g in ROADS if abs(g) <= 45.0]
    for gx in inner:
        for gy in inner:
            for dx in (-1, 1):
                for dy in (-1, 1):
                    x = gx + dx * 5.2
                    y = gy + dy * 5.2
                    yaw = 3.14159 if dx > 0 else 0.0
                    models.append(streetlight_model(
                        f'streetlight_{sl_index}', x, y, yaw,
                        with_light=(gx == 0.0 and gy == 0.0)))
                    sl_index += 1

    # --------------------------------------------------------------- trees
    t_index = 0
    for block_index, (cx, cy) in enumerate(BLOCK_CENTRES):
        if (cx, cy) in PARKS:
            # Park planting: broad oaks on the corners, compact pines along
            # the paths, all clear of the pond in the NE corner.
            spots = [('oak', (-8.0, -8.0)), ('oak', (-8.0, 8.0)),
                     ('oak', (8.0, -8.0)), ('pine', (0.0, 8.2)),
                     ('pine', (8.2, 0.0)), ('pine', (-8.0, -1.5)),
                     ('pine', (-1.5, -8.0))]
        elif (cx, cy) in APARTMENT_BLOCKS and use_fuel:
            # The apartment fills the lot to the sidewalk; no street trees.
            spots = []
        else:
            # Native pines are ~3 m across: they fit the planting strip.
            spots = [('pine', (-9.0, 0.0)), ('pine', (9.0, 0.0)),
                     ('pine', (0.0, -9.0)), ('pine', (0.0, 9.0))]

        for k, (key, (dx, dy)) in enumerate(spots):
            x, y = cx + dx, cy + dy
            if use_fuel:
                models.append(fuel_include(
                    key, f'tree_{t_index}', x, y, 0.0,
                    RNG.uniform(0, 2 * math.pi)))
            elif key == 'pine':
                models.append(tree_model(f'tree_{t_index}', x, y, RNG))
            else:
                models.append(tree_model(f'tree_{t_index}', x, y, RNG))
            t_index += 1

    # ------------------------------------------------------------- benches
    bench_index = 0
    for cx, cy in sorted(PARKS):
        for bx, by, yaw in ((-6.2, 2.2, 0.0), (6.2, -2.2, 0.0),
                            (2.2, 6.4, 1.5708), (-2.2, -6.4, 1.5708)):
            models.append(bench(f'bench_{bench_index}', cx + bx, cy + by,
                                yaw, use_fuel))
            bench_index += 1

    # ---------------------------------------------------------- parked cars
    if use_fuel:
        parked = [
            (2.9, -22.0, 0.0), (2.9, -34.0, 0.0),
            (-2.9, -17.0, math.pi), (-2.9, -40.0, math.pi),
            (12.0, 32.9, math.pi / 2), (24.0, 27.1, -math.pi / 2),
            (18.0, 2.9, math.pi / 2), (-20.0, -2.9, -math.pi / 2),
            (32.9, 12.0, 0.0), (-32.9, -18.0, math.pi),
            (44.0, -32.9, math.pi / 2),
        ]
        for index, (x, y, yaw) in enumerate(parked):
            models.append(fuel_include(
                'car', f'parked_car_{index}', x, y, 0.02, yaw))

    # -------------------------------------------------------------- output
    world = f'''<?xml version="1.0" ?>
<!-- Golden-hour city for the autonomous delivery rover.
     Generated by scripts/generate_city.py - edit that file, not this one.
     Fuel assets are cached under ~/.gz/fuel on first generation. -->
<sdf version="1.9">
  <world name="city">

    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>

    <!-- Golden hour: the key light is a low, warm sun from the west; a cool
         fill from the opposite side keeps the shadow sides blue instead of
         black.  Long shadows are what sell the time of day. -->
    <scene>
      <ambient>0.42 0.31 0.28 1</ambient>
      <background>0.97 0.68 0.46 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 40 0 0 0</pose>
      <diffuse>1.00 0.66 0.38 1</diffuse>
      <specular>0.95 0.60 0.30 1</specular>
      <direction>-0.86 0.38 -0.34</direction>
    </light>

    <light type="directional" name="sky_fill">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 40 0 0 0</pose>
      <diffuse>0.30 0.36 0.55 1</diffuse>
      <specular>0.05 0.06 0.08 1</specular>
      <direction>0.55 -0.45 -0.70</direction>
    </light>

    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.46 0.35 0.32</ambient_light>
        <background_color>0.97 0.68 0.46</background_color>
        <camera_pose>0 -20 5.0 -0.35 1.5708</camera_pose>
      </plugin>

      <plugin filename="GzSceneManager" name="Scene Manager">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>

      <plugin filename="InteractiveViewControl" name="Interactive view control">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>

      <!-- Chase camera: the 3D view follows my_robot without mouse input.
           chase_camera.py keeps /gui/track updated with a heading-relative
           offset so the view stays behind the rover like a racing game. -->
      <plugin filename="CameraTracking" name="Camera Tracking">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
        <follow_target>my_robot</follow_target>
        <follow_offset>-3.5 0 3.2</follow_offset>
        <follow_pgain>2.5</follow_pgain>
      </plugin>

      <plugin filename="WorldControl" name="World control">
        <gz-gui>
          <title>World control</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">72</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="left" target="left"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <play_pause>true</play_pause>
        <step>true</step>
        <start_paused>false</start_paused>
        <use_event>true</use_event>
      </plugin>

      <plugin filename="WorldStats" name="World stats">
        <gz-gui>
          <title>World stats</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">110</property>
          <property type="double" key="width">290</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <sim_time>true</sim_time>
        <real_time>true</real_time>
        <real_time_factor>true</real_time_factor>
        <iterations>true</iterations>
      </plugin>

      <plugin filename="ImageDisplay" name="Camera feed">
        <gz-gui>
          <title>Rover camera</title>
          <property type="bool" key="showTitleBar">true</property>
          <property type="bool" key="resizable">true</property>
          <property type="double" key="height">240</property>
          <property type="double" key="width">320</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
        </gz-gui>
        <topic>/camera/image</topic>
      </plugin>
    </gui>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <pose>0 0 -0.1 0 0 0</pose>
          <geometry>{box('220 220 0.2')}</geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <pose>0 0 -0.1 0 0 0</pose>
          <geometry>{box('220 220 0.2')}</geometry>
          {GRASS}
        </visual>
      </link>
    </model>

    <model name="city_ground">
      <static>true</static>
      <link name="link">
        {''.join(road_parts())}
        {''.join(block_parts())}
      </link>
    </model>
{''.join(models)}
  </world>
</sdf>
'''

    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    with open(OUT_PATH, 'w', encoding='utf-8') as handle:
        handle.write(world)
    print(f'wrote {os.path.abspath(OUT_PATH)} ({len(world) / 1024:.1f} KiB, '
          f'{len(models)} models, fuel={"on" if use_fuel else "off"})')


if __name__ == '__main__':
    main()
