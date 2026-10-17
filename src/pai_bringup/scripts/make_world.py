#!/usr/bin/env python3
"""Generate the Gazebo Fortress workcell SDF from a scene YAML.

    make_world.py scene_default.yaml workcell.sdf

World frame = robot base, table top at z = 0, floor at z = -0.75.
"""
import math
import sys

import yaml

TABLE = {'x': 0.35, 'y': 0.0, 'sx': 1.2, 'sy': 1.4, 'h': 0.75}
WALL = 0.006  # bin wall thickness
FLOOR_Z = -TABLE['h']


def parking_xy(index):
    """Off-table parking spot (on the floor, out of the camera view) for pool object #index."""
    return 2.0 + 0.15 * (index % 6), 1.6 + 0.15 * (index // 6)


def graspable_names(scene):
    """Every movable part, parked or not: these get a detachable-joint grasp plugin on the robot."""
    return [o['name'] for o in scene.get('objects', [])]


def rgba(c, a=1.0):
    return f'{c[0]} {c[1]} {c[2]} {a}'


def material(c):
    return (f'<material><ambient>{rgba(c)}</ambient><diffuse>{rgba(c)}</diffuse>'
            f'<specular>0.2 0.2 0.2 1</specular></material>')


def geometry(shape, size):
    if shape == 'box':
        return f'<box><size>{size[0]} {size[1]} {size[2]}</size></box>'
    if shape == 'cylinder':
        return f'<cylinder><radius>{size[0]}</radius><length>{size[1]}</length></cylinder>'
    raise ValueError(f'unknown shape {shape}')


def height(shape, size):
    return size[2] if shape == 'box' else size[1]


def inertia(shape, size, m):
    if shape == 'box':
        x, y, z = size
        return (m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12)
    r, h = size
    i = m * (3 * r * r + h * h) / 12
    return (i, i, m * r * r / 2)


def box_part(name, pose, size, color, collide=True):
    xyz = ' '.join(str(v) for v in pose)
    g = geometry('box', size)
    col = f'<collision name="{name}_c"><pose>{xyz}</pose><geometry>{g}</geometry></collision>' if collide else ''
    return f'{col}<visual name="{name}_v"><pose>{xyz}</pose><geometry>{g}</geometry>{material(color)}</visual>'


def fixture_sdf(f):
    x, y, z, yaw = f['pose']
    sx, sy, sz = f['size']
    c = f['color']
    if f['type'] == 'tray':
        parts = box_part('base', [0, 0, sz / 2, 0, 0, 0], [sx, sy, sz], c)
    else:  # open-top bin: floor + 4 walls
        parts = box_part('floor', [0, 0, WALL / 2, 0, 0, 0], [sx, sy, WALL], c)
        parts += box_part('wx0', [-sx / 2 + WALL / 2, 0, sz / 2, 0, 0, 0], [WALL, sy, sz], c)
        parts += box_part('wx1', [sx / 2 - WALL / 2, 0, sz / 2, 0, 0, 0], [WALL, sy, sz], c)
        parts += box_part('wy0', [0, -sy / 2 + WALL / 2, sz / 2, 0, 0, 0], [sx, WALL, sz], c)
        parts += box_part('wy1', [0, sy / 2 - WALL / 2, sz / 2, 0, 0, 0], [sx, WALL, sz], c)
    return (f'<model name="{f["name"]}"><static>true</static><pose>{x} {y} {z} 0 0 {yaw}</pose>'
            f'<link name="link">{parts}</link></model>')


def object_sdf(o, park_index=None):
    x, y, yaw = o['pose']
    shape, size, m = o['shape'], o['size'], o['mass']
    h = height(shape, size)
    z = h / 2 + 0.001
    if park_index is not None:
        x, y = parking_xy(park_index)
        z += FLOOR_Z
    ixx, iyy, izz = inertia(shape, size, m)
    g = geometry(shape, size)
    return f'''<model name="{o['name']}">
  <pose>{x} {y} {z} 0 0 {yaw}</pose>
  <link name="link">
    <inertial><mass>{m}</mass><inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>
      <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
    <collision name="c"><geometry>{g}</geometry>
      <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
    <visual name="v"><geometry>{g}</geometry>{material(o['color'])}</visual>
  </link>
</model>'''


def camera_sdf(cam):
    pose = ' '.join(str(v) for v in cam['pose'])
    return f'''<model name="overhead_camera">
  <static>true</static>
  <pose>{pose}</pose>
  <link name="camera_link">
    <visual name="body"><geometry><box><size>0.03 0.09 0.03</size></box></geometry>{material([0.1, 0.1, 0.1])}</visual>
    <sensor name="rgbd" type="rgbd_camera">
      <topic>camera</topic>
      <ignition_frame_id>camera_optical_frame</ignition_frame_id>
      <update_rate>{cam['rate']}</update_rate>
      <always_on>true</always_on>
      <camera>
        <horizontal_fov>{cam['hfov']}</horizontal_fov>
        <image><width>{cam['width']}</width><height>{cam['height']}</height><format>R8G8B8</format></image>
        <clip><near>0.1</near><far>3.0</far></clip>
        <depth_camera><clip><near>0.1</near><far>3.0</far></clip></depth_camera>
        <noise><type>gaussian</type><mean>0</mean><stddev>0.003</stddev></noise>
      </camera>
    </sensor>
  </link>
</model>'''


def human_sdf(h):
    """Simple co-worker mannequin. Kinematic (moved only via set_pose) and without collisions,
    so it can never knock things over; it exists to be seen and to feed the safety sensor."""
    x, y = h['park']
    skin, shirt = [0.85, 0.65, 0.5], [0.15, 0.35, 0.65]
    return f'''<model name="{h['name']}">
  <pose>{x} {y} {FLOOR_Z} 0 0 0</pose>
  <link name="link">
    <kinematic>true</kinematic><gravity>false</gravity>
    <inertial><mass>70</mass><inertia><ixx>10</ixx><iyy>10</iyy><izz>10</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
    <visual name="legs"><pose>0 0 0.45 0 0 0</pose><geometry><box><size>0.25 0.35 0.9</size></box></geometry>{material([0.2, 0.2, 0.25])}</visual>
    <visual name="torso"><pose>0 0 1.2 0 0 0</pose><geometry><box><size>0.28 0.45 0.6</size></box></geometry>{material(shirt)}</visual>
    <visual name="head"><pose>0 0 1.62 0 0 0</pose><geometry><sphere><radius>0.12</radius></sphere></geometry>{material(skin)}</visual>
    <visual name="arm"><pose>-0.25 0 1.3 0 1.2 0</pose><geometry><box><size>0.08 0.08 0.55</size></box></geometry>{material(shirt)}</visual>
  </link>
</model>'''


def world_sdf(scene):
    t = TABLE
    light = scene['light']
    d = light['direction']
    n = math.sqrt(sum(v * v for v in d))
    d = [v / n for v in d]
    parked = [o for o in scene.get('objects', []) if o.get('parked')]
    models = '\n'.join(
        [fixture_sdf(f) for f in scene.get('fixtures', [])]
        + [object_sdf(o, parked.index(o) if o.get('parked') else None) for o in scene.get('objects', [])]
        + [object_sdf(o) for o in scene.get('obstacles', [])]
        + ([human_sdf(scene['human'])] if 'human' in scene else []))
    return f'''<?xml version="1.0"?>
<!-- GENERATED by make_world.py from a scene YAML. Edit the YAML, not this file. -->
<sdf version="1.9">
<world name="workcell">
  <physics name="1ms" type="ignored"><max_step_size>0.002</max_step_size><real_time_factor>1.0</real_time_factor></physics>
  <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
  <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
  <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
  <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
  <scene><ambient>0.4 0.4 0.4 1</ambient><background>0.75 0.8 0.85 1</background><shadows>true</shadows></scene>

  <light type="directional" name="sun">
    <cast_shadows>true</cast_shadows>
    <pose>0 0 3 0 0 0</pose>
    <diffuse>{rgba(light['diffuse'])}</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <direction>{d[0]:.3f} {d[1]:.3f} {d[2]:.3f}</direction>
  </light>

  <model name="ground_plane"><static>true</static><pose>0 0 -{t['h']} 0 0 0</pose>
    <link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>{material([0.55, 0.55, 0.55])}</visual>
    </link>
  </model>

  <model name="table"><static>true</static><pose>{t['x']} {t['y']} {-t['h'] / 2} 0 0 0</pose>
    <link name="link">
      <collision name="c"><geometry><box><size>{t['sx']} {t['sy']} {t['h']}</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>{t['sx']} {t['sy']} {t['h']}</size></box></geometry>{material([0.72, 0.6, 0.45])}</visual>
    </link>
  </model>

  {camera_sdf(scene['camera'])}

{models}
</world>
</sdf>
'''


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    with open(sys.argv[1]) as f:
        scene = yaml.safe_load(f)
    with open(sys.argv[2], 'w') as f:
        f.write(world_sdf(scene))


if __name__ == '__main__':
    main()
