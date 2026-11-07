"""Generate the Gazebo (Harmonic) world from config/markers.yaml.

    python3 -m tello_sim.generate_world --markers config/markers.yaml --out /tmp/tello_world

Writes <out>/room.sdf plus one textured mesh per marker (<out>/markers/*). The room, the wall
markers, a kinematic Tello (velocity-commanded, forward camera), the fixed Logitech camera and a
tethered balloon all come from the same markers.yaml used by the real localizer.
"""
import argparse
import math
from pathlib import Path

import cv2
import numpy as np

from aruco_localization.marker_map import load_marker_map, marker_image, rpy_from_matrix

# Tello camera: 960x720, ~82.6 deg diagonal FOV -> ~70.2 deg horizontal
TELLO_HFOV = 2 * math.atan(480 / 683.0)
# Logitech C922: ~78 deg diagonal, 16:9 -> ~70.4 deg horizontal
LOGI_HFOV = math.radians(70.4)


def pose(xyz, rpy):
    return " ".join(f"{v:.5f}" for v in (*xyz, *rpy))


def write_marker_mesh(out_dir, dictionary, mk):
    img, bits = marker_image(dictionary, mk.id)
    # quad covers the black square (bits + 2 modules) plus one white module on each side
    half = mk.size / 2.0 * (bits + 4) / (bits + 2)
    name = f"marker_{mk.id}"
    cv2.imwrite(str(out_dir / f"{name}.png"), img)
    (out_dir / f"{name}.mtl").write_text(
        f"newmtl {name}\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nd 1\nillum 1\nmap_Kd {name}.png\n")
    # Quad in the marker frame (x right, y up, normal +z), UV origin bottom-left
    (out_dir / f"{name}.obj").write_text(
        f"mtllib {name}.mtl\no {name}\n"
        f"v {-half} {-half} 0\nv {half} {-half} 0\nv {half} {half} 0\nv {-half} {half} 0\n"
        "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 0 0 1\n"
        f"usemtl {name}\nf 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n")
    return name


def box_visual(name, size, color, collision=True):
    s = " ".join(f"{v:.4f}" for v in size)
    c = f"{color[0]} {color[1]} {color[2]} 1"
    col = f"<collision name='{name}_c'><geometry><box><size>{s}</size></box></geometry></collision>" if collision else ""
    return (f"{col}<visual name='{name}_v'><geometry><box><size>{s}</size></box></geometry>"
            f"<material><ambient>{c}</ambient><diffuse>{c}</diffuse></material></visual>")


def camera_sensor(name, topic, hfov, w, h, rate, pose_str, save_dir=None):
    save = f"<save enabled='true'><path>{save_dir}</path></save>" if save_dir else ""
    return f"""
        <sensor name='{name}' type='camera'>
          <pose>{pose_str}</pose>
          <always_on>1</always_on>
          <update_rate>{rate}</update_rate>
          <topic>{topic}</topic>
          <camera>
            <horizontal_fov>{hfov:.5f}</horizontal_fov>
            <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
            <clip><near>0.03</near><far>30</far></clip>
            {save}
          </camera>
        </sensor>"""


def build_world(cfg, markers, mesh_dir, args):
    room = cfg.get("room", {"size_x": 4.0, "size_y": 3.5, "height": 3.0})
    sx, sy, hz = float(room["size_x"]), float(room["size_y"]), float(room["height"])
    t = 0.05  # wall thickness

    walls = [
        ("wall_west", (-t / 2, sy / 2, hz / 2), (t, sy + 2 * t, hz)),
        ("wall_east", (sx + t / 2, sy / 2, hz / 2), (t, sy + 2 * t, hz)),
        ("wall_south", (sx / 2, -t / 2, hz / 2), (sx, t, hz)),
        ("wall_north", (sx / 2, sy + t / 2, hz / 2), (sx, t, hz)),
    ]
    wall_xml = "".join(
        f"<model name='{n}'><static>true</static><pose>{pose(p, (0, 0, 0))}</pose>"
        f"<link name='link'>{box_visual(n, s, (0.82, 0.80, 0.76))}</link></model>"
        for n, p, s in walls)
    floor_xml = (f"<model name='floor'><static>true</static><pose>{pose((sx / 2, sy / 2, -0.01), (0, 0, 0))}</pose>"
                 f"<link name='link'>{box_visual('floor', (sx, sy, 0.02), (0.45, 0.36, 0.28))}</link></model>")

    marker_xml = ""
    for mk in markers.values():
        name = f"marker_{mk.id}"
        marker_xml += (
            f"<model name='aruco_{mk.id}'><static>true</static><pose>{pose(mk.t, rpy_from_matrix(mk.R))}</pose>"
            f"<link name='link'><visual name='v'><cast_shadows>false</cast_shadows><geometry><mesh>"
            f"<uri>file://{mesh_dir}/{name}.obj</uri></mesh></geometry></visual></link></model>")

    # Fixed Logitech camera: high in the south-west corner, looking at the room center
    cam_p = np.array([0.15, 0.15, hz - 0.25])
    target = np.array([sx / 2, sy / 2, 1.0])
    d = target - cam_p
    logi_yaw = math.atan2(d[1], d[0])
    logi_pitch = math.atan2(-d[2], math.hypot(d[0], d[1]))  # positive = looking down
    logi_xml = (f"<model name='logitech'><static>true</static><pose>{pose(cam_p, (0, logi_pitch, logi_yaw))}</pose>"
                f"<link name='link'>{box_visual('logi', (0.03, 0.09, 0.03), (0.1, 0.1, 0.1), collision=False)}"
                f"{camera_sensor('logitech_camera', 'logitech_sim/camera/image', LOGI_HFOV, args.logi_width, args.logi_height, args.logi_rate, '0 0 0 0 0 0', args.save_dir and args.save_dir + '/logitech')}"
                f"</link></model>")

    # Tethered balloon
    bx, by, bz = args.balloon
    balloon_xml = (
        f"<model name='balloon'><static>true</static><pose>{pose((bx, by, bz), (0, 0, 0))}</pose><link name='link'>"
        f"<visual name='ball'><geometry><sphere><radius>0.14</radius></sphere></geometry>"
        f"<material><ambient>0.85 0.05 0.05 1</ambient><diffuse>0.9 0.1 0.1 1</diffuse><specular>0.4 0.4 0.4 1</specular></material></visual>"
        f"<visual name='string'><pose>0 0 {-(bz - 0.14) / 2 - 0.14} 0 0 0</pose><geometry><cylinder><radius>0.002</radius>"
        f"<length>{bz - 0.14}</length></cylinder></geometry><material><diffuse>0.9 0.9 0.9 1</diffuse></material></visual>"
        f"</link></model>")

    # Kinematic Tello: velocity-commanded body (the real Tello's inner loop is a black box)
    sxp, syp, syaw = args.start
    tello_xml = f"""
    <model name='tello'>
      <pose>{pose((sxp, syp, 0.03), (0, 0, syaw))}</pose>
      <link name='base_link'>
        <gravity>false</gravity>
        <inertial><mass>0.08</mass><inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>1.5e-4</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name='c'><geometry><box><size>0.10 0.10 0.04</size></box></geometry></collision>
        <visual name='body'><geometry><box><size>0.10 0.09 0.04</size></box></geometry>
          <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material></visual>
        <visual name='lens'><pose>0.051 0 0 0 1.5708 0</pose><geometry><cylinder><radius>0.008</radius><length>0.004</length></cylinder></geometry>
          <material><diffuse>0.05 0.05 0.05 1</diffuse></material></visual>
        {camera_sensor('tello_camera', 'tello_sim/camera/image', TELLO_HFOV, args.tello_width, args.tello_height, args.tello_rate, '0.05 0 0 0 0 0', args.save_dir and args.save_dir + '/tello')}
      </link>
      <plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'/>
      <plugin filename='gz-sim-odometry-publisher-system' name='gz::sim::systems::OdometryPublisher'>
        <odom_frame>map</odom_frame>
        <robot_base_frame>base_link_gt</robot_base_frame>
        <dimensions>3</dimensions>
        <odom_publish_frequency>50</odom_publish_frequency>
      </plugin>
    </model>"""

    return f"""<?xml version='1.0'?>
<!-- GENERATED by tello_sim/generate_world.py from config/markers.yaml. Do not edit by hand. -->
<sdf version='1.9'>
  <world name='room'>
    <physics name='1ms' type='ignored'><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>
    <plugin filename='gz-sim-user-commands-system' name='gz::sim::systems::UserCommands'/>
    <plugin filename='gz-sim-scene-broadcaster-system' name='gz::sim::systems::SceneBroadcaster'/>
    <plugin filename='gz-sim-sensors-system' name='gz::sim::systems::Sensors'><render_engine>{args.render_engine}</render_engine></plugin>
    <scene><ambient>0.55 0.55 0.55 1</ambient><background>0.7 0.8 0.9 1</background><shadows>false</shadows></scene>
    <light type='directional' name='sun'><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.6 0.6 0.6 1</diffuse><specular>0.1 0.1 0.1 1</specular><direction>-0.3 0.2 -1</direction></light>
    <light type='point' name='ceiling_lamp'><cast_shadows>false</cast_shadows><pose>{sx / 2} {sy / 2} {hz - 0.1} 0 0 0</pose>
      <diffuse>0.7 0.7 0.65 1</diffuse><specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>12</range><constant>0.6</constant><linear>0.05</linear><quadratic>0.01</quadratic></attenuation></light>
    {floor_xml}
    {wall_xml}
    {marker_xml}
    {logi_xml}
    {balloon_xml}
    {tello_xml}
  </world>
</sdf>
"""


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--markers", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--start", type=float, nargs=3, default=[2.0, 1.75, 0.0], metavar=("X", "Y", "YAW"))
    ap.add_argument("--balloon", type=float, nargs=3, default=[3.1, 2.6, 1.2], metavar=("X", "Y", "Z"))
    ap.add_argument("--tello-width", type=int, default=960)
    ap.add_argument("--tello-height", type=int, default=720)
    ap.add_argument("--tello-rate", type=float, default=30)
    ap.add_argument("--logi-width", type=int, default=960)
    ap.add_argument("--logi-height", type=int, default=540)
    ap.add_argument("--logi-rate", type=float, default=10)
    ap.add_argument("--render-engine", default="ogre2", choices=["ogre2", "ogre"],
                    help="ogre is faster with software rendering (no GPU)")
    ap.add_argument("--save-dir", default=None, help="also save camera frames to disk (debug)")
    args = ap.parse_args(argv)

    out = Path(args.out).resolve()
    mesh_dir = out / "markers"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    cfg, markers = load_marker_map(args.markers)
    for mk in markers.values():
        write_marker_mesh(mesh_dir, cfg.get("dictionary", "DICT_4X4_50"), mk)
    world = build_world(cfg, markers, mesh_dir, args)
    (out / "room.sdf").write_text(world)
    print(f"World with {len(markers)} markers written to {out / 'room.sdf'}")
    return out / "room.sdf"


if __name__ == "__main__":
    main()
