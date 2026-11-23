"""Generates the self-contained Gazebo Fortress world (SDF 1.8).

Everything is inlined (no model:// lookups -> no IGN_GAZEBO_RESOURCE_PATH
problems). Physics fixes vs. the original project:
  * drones have NO rendering sensors (5 software-rendered RGB-D cameras were
    what pushed the real-time factor down to ~10 % in VirtualBox); camera,
    depth, IMU, VIO and UWB measurements are simulated by sensors.py instead
  * 2 ms physics step instead of 1 ms
  * drone collision covers the rotor disc (r = 0.5 m) and the landing feet,
    so legs no longer sink into the ground; spawn height = resting height
  * tree crowns and pine tiers have collision shapes (crowns had none)
  * OdometryPublisher (3-D) publishes the true pose; it only feeds the
    simulated sensors and the scoring, the drones fly on their EKF estimate
No route is drawn into the world: every drone plans on board, and the world node draws each drone's
routes live through the Gazebo GUI's marker service (gz_lines.py) - its reference route (A*, planned
on the pad before take-off) in GREEN and the route it has flown (RRT* or A*, re-planned in flight) as a
trail in its own colour.
"""
import math

from . import config as C


def _f(v):
    return f'{v:.4f}'.rstrip('0').rstrip('.') if isinstance(v, float) else str(v)


def pose(x=0.0, y=0.0, z=0.0, r=0.0, p=0.0, yw=0.0):
    return f'<pose>{_f(x)} {_f(y)} {_f(z)} {_f(r)} {_f(p)} {_f(yw)}</pose>'


def material(rgb, emissive=None, spec=0.2):
    r, g, b = rgb[:3]
    a = rgb[3] if len(rgb) > 3 else 1.0
    s = (f'<material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse>'
         f'<specular>{spec} {spec} {spec} 1</specular>')
    if emissive:
        s += f'<emissive>{emissive[0]} {emissive[1]} {emissive[2]} 1</emissive>'
    return s + '</material>'


def geom(kind, *dims):
    if kind == 'box':
        return f'<geometry><box><size>{_f(dims[0])} {_f(dims[1])} {_f(dims[2])}</size></box></geometry>'
    if kind == 'cyl':
        return f'<geometry><cylinder><radius>{_f(dims[0])}</radius><length>{_f(dims[1])}</length></cylinder></geometry>'
    if kind == 'sph':
        return f'<geometry><sphere><radius>{_f(dims[0])}</radius></sphere></geometry>'
    raise ValueError(kind)


def visual(name, g, mat, p='', transparency=None, shadows=True):
    t = f'<transparency>{transparency}</transparency>' if transparency else ''
    cs = '' if shadows else '<cast_shadows>false</cast_shadows>'
    return f'<visual name="{name}">{p}{g}{mat}{t}{cs}</visual>'


def collision(name, g, p=''):
    return f'<collision name="{name}">{p}{g}</collision>'


# ---------------------------------------------------------------------------
GRASS_A = (0.22, 0.56, 0.20)
GRASS_B = (0.27, 0.62, 0.24)
BARK = (0.36, 0.22, 0.10)
LEAF = (0.12, 0.50, 0.14)
LEAF2 = (0.18, 0.58, 0.18)
PINE = (0.07, 0.33, 0.16)


def ground():
    v = []
    v.append(visual('outer', geom('box', 44.0, 44.0, 0.02), material((0.36, 0.42, 0.28)),
                    pose(0, 0, -0.012), shadows=False))
    v.append(visual('grass', geom('box', 20.0, 20.0, 0.02), material(GRASS_A), pose(0, 0, -0.009),
                    shadows=False))
    for k in range(5):  # mowed-lawn stripes
        x = -8.0 + 4.0 * k
        v.append(visual(f'stripe{k}', geom('box', 2.0, 20.0, 0.004), material(GRASS_B),
                        pose(x, 0, 0.003), shadows=False))
    w = 0.08
    for k, (x, y, sx, sy) in enumerate([(0, 10, 20 + w, w), (0, -10, 20 + w, w), (10, 0, w, 20 + w), (-10, 0, w, 20 + w)]):
        v.append(visual(f'edge{k}', geom('box', sx, sy, 0.006), material((0.95, 0.95, 0.92)), pose(x, y, 0.005),
                        shadows=False))
    for k, (x, y) in enumerate([(10, 10), (10, -10), (-10, 10), (-10, -10)]):
        v.append(visual(f'corner{k}', geom('cyl', 0.06, 1.2), material((1.0, 0.45, 0.05), spec=0.5), pose(x, y, 0.6)))
        v.append(visual(f'cornertop{k}', geom('sph', 0.1), material((1.0, 0.95, 0.9), emissive=(0.8, 0.8, 0.7)),
                        pose(x, y, 1.25)))
    col = ('<collision name="ground"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>'
           '<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>')
    return f'<model name="ground_plane"><static>true</static><link name="link">{col}{"".join(v)}</link></model>'


def landmark_tree(k, x, y):
    s = []
    s.append(visual('trunk', geom('cyl', C.TREE_TRUNK_R, C.TREE_TRUNK_H), material(BARK), pose(0, 0, C.TREE_TRUNK_H / 2)))
    s.append(collision('trunk', geom('cyl', C.TREE_TRUNK_R, C.TREE_TRUNK_H), pose(0, 0, C.TREE_TRUNK_H / 2)))
    s.append(visual('crown', geom('sph', C.TREE_CROWN_R), material(LEAF), pose(0, 0, C.TREE_CROWN_Z)))
    s.append(visual('crown2', geom('sph', 0.32), material(LEAF2), pose(0.12, -0.08, C.TREE_CROWN_Z + 0.36)))
    s.append(collision('crown', geom('sph', C.TREE_CROWN_R), pose(0, 0, C.TREE_CROWN_Z)))
    # ArUco marker cube: one 0.30 m board on each of the 4 sides of the trunk
    m = C.SENSORS.marker_size
    s.append(visual('marker_cube', geom('box', m, m, m), material((0.97, 0.97, 0.97)), pose(0, 0, C.ANCHOR_Z)))
    for f, (dx, dy, yw) in enumerate([(1, 0, 0.0), (-1, 0, math.pi), (0, 1, math.pi / 2), (0, -1, -math.pi / 2)]):
        # black code pattern: outer square + two white "bits"
        cx, cy = dx * (m / 2 + 0.002), dy * (m / 2 + 0.002)
        s.append(visual(f'code{f}', geom('box', 0.004, 0.62 * m, 0.62 * m), material((0.03, 0.03, 0.03)),
                        pose(cx, cy, C.ANCHOR_Z, 0, 0, yw), shadows=False))
        for b, (u, w) in enumerate([(-0.12, 0.10), (0.10, -0.08)]):
            ox, oy = -math.sin(yw) * u * m, math.cos(yw) * u * m
            s.append(visual(f'bit{f}_{b}', geom('box', 0.006, 0.16 * m, 0.16 * m), material((0.97, 0.97, 0.97)),
                            pose(cx + ox, cy + oy, C.ANCHOR_Z + w * m, 0, 0, yw), shadows=False))
    return (f'<model name="tree_{k}"><static>true</static>{pose(x, y, 0)}'
            f'<link name="link">{"".join(s)}</link></model>')


def pine(k, x, y):
    s = []
    for n, (z0, z1, r) in enumerate(C.PINE_TIERS):
        h = z1 - z0
        c = BARK if n == 0 else tuple(min(1.0, v + 0.03 * n) for v in PINE)
        s.append(visual(f'tier{n}', geom('cyl', r, h), material(c), pose(0, 0, z0 + h / 2)))
        s.append(collision(f'tier{n}', geom('cyl', r, h), pose(0, 0, z0 + h / 2)))
    return (f'<model name="unknown_pine_{k}"><static>true</static>{pose(x, y, 0)}'
            f'<link name="link">{"".join(s)}</link></model>')


def start_pad(name, x, y, ring):
    s = [visual('disc', geom('cyl', 0.85, 0.01), material((0.92, 0.92, 0.92)), pose(0, 0, 0.004), shadows=False),
         visual('ring', geom('cyl', 0.95, 0.008), material(ring, emissive=tuple(0.5 * c for c in ring)),
                pose(0, 0, 0.003), shadows=False),
         visual('h1', geom('box', 0.1, 0.7, 0.012), material((0.1, 0.1, 0.12)), pose(-0.22, 0, 0.006), shadows=False),
         visual('h2', geom('box', 0.1, 0.7, 0.012), material((0.1, 0.1, 0.12)), pose(0.22, 0, 0.006), shadows=False),
         visual('h3', geom('box', 0.44, 0.1, 0.012), material((0.1, 0.1, 0.12)), pose(0, 0, 0.006), shadows=False)]
    return f'<model name="{name}"><static>true</static>{pose(x, y, 0)}<link name="link">{"".join(s)}</link></model>'


def rgb(hex_color):
    """'#rrggbb' -> (r, g, b) in 0..1."""
    h = hex_color.lstrip('#')
    return tuple(int(h[k:k + 2], 16) / 255.0 for k in (0, 2, 4))


def goal_pad(name, x, y, color):
    """Goal pad in the drone's colour: rings and a flag (the same design for every drone)."""
    s = []
    white = (0.95, 0.95, 0.95)
    rings = [(0.95, color), (0.75, white), (0.55, color), (0.35, white), (0.15, color)]
    for n, (r, c) in enumerate(rings):
        s.append(visual(f'r{n}', geom('cyl', r, 0.006), material(c, emissive=tuple(0.25 * v for v in c)),
                        pose(0, 0, 0.004 + 0.003 * n), shadows=False))
    s.append(visual('pole', geom('cyl', 0.025, 2.2), material((0.85, 0.85, 0.85), spec=0.6), pose(1.2, 0, 1.1)))
    s.append(visual('flag', geom('box', 0.01, 0.5, 0.32), material(color, emissive=tuple(0.6 * v for v in color)),
                    pose(1.2, 0.26, 2.02)))
    return f'<model name="{name}"><static>true</static>{pose(x, y, 0)}<link name="link">{"".join(s)}</link></model>'


def drone(spec):
    body = rgb(spec.color)                         # every drone is the same model, in its own colour
    arm = tuple(0.55 * c for c in body)
    dark = (0.08, 0.08, 0.09)
    v = []
    v.append(visual('body', geom('cyl', 0.15, 0.09), material(body, spec=0.6)))
    v.append(visual('canopy', geom('sph', 0.105), material(tuple(min(1, c + 0.15) for c in body), spec=0.8),
                    pose(0, 0, 0.035)))
    for k, (dx, dy) in enumerate([(1, 0), (-1, 0), (0, 1), (0, -1)]):
        yaw = 0.0 if dy == 0 else math.pi / 2
        v.append(visual(f'arm{k}', geom('box', 0.375, 0.045, 0.022), material(arm), pose(0.225 * dx, 0.225 * dy, 0, 0, 0, yaw)))
        v.append(visual(f'motor{k}', geom('cyl', 0.032, 0.05), material(dark, spec=0.7), pose(0.39 * dx, 0.39 * dy, 0.02)))
        v.append(visual(f'rotor{k}', geom('cyl', 0.135, 0.006), material((0.2, 0.2, 0.22, 0.45)),
                        pose(0.39 * dx, 0.39 * dy, 0.05), transparency=0.55, shadows=False))
    for k, (dx, dy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        v.append(visual(f'leg{k}', geom('cyl', 0.008, 0.09), material(dark), pose(0.09 * dx, 0.09 * dy, -0.075)))
        v.append(visual(f'foot{k}', geom('sph', 0.012), material(dark), pose(0.09 * dx, 0.09 * dy, -0.115)))
    v.append(visual('camera', geom('box', 0.035, 0.05, 0.035), material(dark, spec=0.9), pose(0.16, 0, -0.01, 0, 0.2618, 0)))
    for k, a in enumerate((math.pi / 4, 3 * math.pi / 4, -3 * math.pi / 4, -math.pi / 4)):   # 360 deg nav camera ring
        v.append(visual(f'navcam{k}', geom('box', 0.025, 0.03, 0.025), material((0.05, 0.05, 0.06), spec=0.9),
                        pose(0.13 * math.cos(a), 0.13 * math.sin(a), 0.045, 0, 0, a)))
    v.append(visual('led_front', geom('sph', 0.022), material((0.1, 1.0, 0.2), emissive=(0.0, 0.9, 0.1)), pose(0.155, 0, 0.03)))
    v.append(visual('led_rear', geom('sph', 0.022), material((1.0, 0.1, 0.1), emissive=(0.9, 0.0, 0.0)), pose(-0.155, 0, 0.03)))
    c = [collision('rotor_disc', geom('cyl', C.DRONE_COLLISION_R, 0.1))]
    for k, (dx, dy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        c.append(collision(f'foot{k}', geom('sph', 0.012), pose(0.09 * dx, 0.09 * dy, -0.115)))
    x, y = spec.spawn
    return f'''<model name="{spec.name}">
  {pose(x, y, C.REST_Z + 0.003)}
  <link name="base_link">
    <inertial><mass>{C.ENERGY.mass}</mass>
      <inertia><ixx>0.03</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.03</iyy><iyz>0</iyz><izz>0.05</izz></inertia>
    </inertial>
    {"".join(v)}
    {"".join(c)}
  </link>
  <plugin filename="ignition-gazebo-velocity-control-system" name="ignition::gazebo::systems::VelocityControl"/>
  <plugin filename="ignition-gazebo-odometry-publisher-system" name="ignition::gazebo::systems::OdometryPublisher">
    <odom_frame>world</odom_frame>
    <robot_base_frame>{spec.name}</robot_base_frame>
    <odom_publish_frequency>50</odom_publish_frequency>
    <dimensions>3</dimensions>
  </plugin>
</model>'''


def gui_block(world_name):
    return f'''<gui fullscreen="0">
  <plugin filename="GzScene3D" name="3D View">
    <ignition-gui>
      <title>3D View</title>
      <property type="bool" key="showTitleBar">false</property>
      <property type="string" key="state">docked</property>
    </ignition-gui>
    <engine>ogre2</engine>
    <scene>scene</scene>
    <ambient_light>0.55 0.55 0.6</ambient_light>
    <background_color>0.62 0.78 0.95</background_color>
    <camera_pose>14 -14 14 0 0.62 2.356</camera_pose>
  </plugin>
  <plugin filename="WorldControl" name="World control">
    <ignition-gui>
      <title>World control</title>
      <property type="bool" key="showTitleBar">false</property>
      <property type="bool" key="resizable">false</property>
      <property type="double" key="height">72</property>
      <property type="double" key="width">121</property>
      <property type="double" key="z">1</property>
      <property type="string" key="state">floating</property>
      <anchors target="3D View">
        <line own="left" target="left"/>
        <line own="bottom" target="bottom"/>
      </anchors>
    </ignition-gui>
    <play_pause>true</play_pause>
    <step>true</step>
    <service>/world/{world_name}/control</service>
    <stats_topic>/world/{world_name}/stats</stats_topic>
  </plugin>
  <plugin filename="WorldStats" name="World stats">
    <ignition-gui>
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
    </ignition-gui>
    <sim_time>true</sim_time>
    <real_time>true</real_time>
    <real_time_factor>true</real_time_factor>
    <iterations>true</iterations>
    <topic>/world/{world_name}/stats</topic>
  </plugin>
  <plugin filename="EntityTree" name="Entity tree"/>
</gui>'''


def generate_world(nav_key=C.BASELINE, shadows=False, world_name='swarm_arena', show_goals=True):
    parts = []
    parts.append(f'''<physics name="2ms" type="ignored">
  <max_step_size>0.002</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
<plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
<plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
<plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
<gravity>0 0 -9.81</gravity>
<scene>
  <ambient>0.55 0.55 0.6 1</ambient>
  <background>0.62 0.78 0.95 1</background>
  <shadows>{"true" if shadows else "false"}</shadows>
  <grid>false</grid>
</scene>
<light type="directional" name="sun">
  <cast_shadows>{"true" if shadows else "false"}</cast_shadows>
  <pose>0 0 20 0 0 0</pose>
  <diffuse>0.95 0.93 0.85 1</diffuse>
  <specular>0.35 0.35 0.3 1</specular>
  <direction>-0.45 0.25 -0.85</direction>
</light>
<light type="directional" name="fill">
  <cast_shadows>false</cast_shadows>
  <pose>0 0 20 0 0 0</pose>
  <diffuse>0.35 0.38 0.45 1</diffuse>
  <specular>0 0 0 1</specular>
  <direction>0.5 -0.3 -0.6</direction>
</light>''')
    parts.append(ground())
    for k, (x, y) in enumerate(C.LANDMARK_TREES, start=1):
        parts.append(landmark_tree(k, x, y))
    for k, (x, y) in enumerate(C.UNKNOWN_PINES, start=1):
        parts.append(pine(k, x, y))
    for d in C.DRONES:                              # every drone: start pad and goal pad in its colour
        parts.append(start_pad(f'pad_start_{d.name}', d.spawn[0], d.spawn[1], rgb(d.color)))
        if show_goals:
            parts.append(goal_pad(f'pad_goal_{d.name}', d.goal[0], d.goal[1], rgb(d.color)))
    for d in C.DRONES:
        parts.append(drone(d))
    body = '\n'.join(parts)
    header = (f'<!-- AUTO-GENERATED by swarm_intel/world_gen.py  (nav={nav_key})\n'
              f'     No routes in the world: the drones plan on board. Edit config.py, not this file. -->')
    return f'''<?xml version="1.0" ?>
{header}
<sdf version="1.8">
<world name="{world_name}">
{gui_block(world_name)}
{body}
</world>
</sdf>
'''


def main():
    import argparse
    ap = argparse.ArgumentParser(description='Write the swarm arena world file')
    ap.add_argument('-o', '--output', default='swarm_arena_generated.world')
    ap.add_argument('--nav', default=C.BASELINE, choices=list(C.NAV))
    ap.add_argument('--shadows', action='store_true')
    a = ap.parse_args()
    with open(a.output, 'w') as f:
        f.write(generate_world(a.nav, a.shadows))
    print(f'wrote {a.output}')


if __name__ == '__main__':
    main()
