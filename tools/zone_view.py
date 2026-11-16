#!/usr/bin/env python3
"""3D view of the flight zone on the room scan (one HTML file, open it in any browser: drag = rotate,
right-drag = pan, wheel = zoom).

    python tools/zone_view.py <room_map.obj> [--zone config/flight_zone_3d.yaml] [--out zone_view.html]

Shows the scan (coloured by height), the markers with their ids, the proposed zone (green boxes) and the zone the
driver uses today (flight_zone_band.yaml, grey wireframe).
"""
import argparse
import base64
import json
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from free_zone import load_obj, surface_points  # noqa: E402

REPO = Path(__file__).resolve().parent.parent

PAGE = r"""<!doctype html>
<html lang="fr"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Zone de vol 3D</title>
<style>
:root{--bg:#f4f4f2;--fg:#1d1d1b;--panel:rgba(255,255,255,.92);--muted:#666}
@media (prefers-color-scheme: dark){:root{--bg:#141414;--fg:#eee;--panel:rgba(30,30,30,.92);--muted:#aaa}}
html,body{margin:0;height:100%;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif;overflow:hidden}
#c{position:fixed;inset:0}
#p{position:fixed;top:12px;left:12px;max-width:min(360px,calc(100vw - 48px));background:var(--panel);padding:12px 14px;border-radius:10px;box-shadow:0 2px 12px rgba(0,0,0,.2)}
#p h1{font-size:15px;margin:0 0 6px}#p p{margin:4px 0;color:var(--muted)}label{display:block;margin:3px 0;cursor:pointer}
.sw{display:inline-block;width:12px;height:12px;border-radius:3px;vertical-align:-1px;margin-right:6px}
</style></head><body><canvas id="c"></canvas>
<div id="p"><h1>Zone de vol proposée</h1>
<p>Glisser = tourner, clic droit = déplacer, molette = zoom.</p>
<div id="boxes"></div>
<label><input type="checkbox" id="old" checked><span class="sw" style="background:#888"></span>zone actuelle (driver)</label>
<label><span class="sw" style="background:#d9480f"></span>trajet de patrouille (tello_patrol)</label>
<label><input type="checkbox" id="ceil"><span class="sw" style="background:#bbb"></span>plafond</label>
<label><input type="checkbox" id="scan" checked><span class="sw" style="background:linear-gradient(90deg,#2b6cb0,#e53e3e)"></span>scan (couleur = hauteur)</label>
<p id="info"></p></div>
<script type="importmap">{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js","three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}</script>
<script type="module">
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
const D = __DATA__;
const dark = matchMedia('(prefers-color-scheme: dark)').matches;
const renderer = new THREE.WebGLRenderer({canvas: document.getElementById('c'), antialias: true});
renderer.setPixelRatio(devicePixelRatio);
const scene = new THREE.Scene(); scene.background = new THREE.Color(dark ? 0x141414 : 0xf4f4f2);
const cam = new THREE.PerspectiveCamera(50, 1, 0.05, 50); cam.up.set(0, 0, 1);
cam.position.set(-2.5, -3.0, 4.2);
const ctl = new OrbitControls(cam, renderer.domElement); ctl.target.set(0.2, 1.6, 1.6); ctl.update();
function resize(){renderer.setSize(innerWidth, innerHeight); cam.aspect = innerWidth/innerHeight; cam.updateProjectionMatrix();}
addEventListener('resize', resize); resize();
// scan
const raw = Uint8Array.from(atob(D.pts), c => c.charCodeAt(0));
const q = new Int16Array(raw.buffer); const n = q.length/3;
const pos = new Float32Array(n*3), col = new Float32Array(n*3);
const lo = new THREE.Color(0x2b6cb0), hi = new THREE.Color(0xe53e3e), tmp = new THREE.Color();
for (let i = 0; i < n; i++) { for (let k = 0; k < 3; k++) pos[3*i+k] = q[3*i+k]/100;
  tmp.copy(lo).lerp(hi, Math.min(1, Math.max(0, pos[3*i+2]/2.4))); col.set([tmp.r, tmp.g, tmp.b], 3*i); }
function cloud(filter){ const idx = []; for (let i = 0; i < n; i++) if (filter(pos[3*i+2])) idx.push(i);
  const g = new THREE.BufferGeometry(), P = new Float32Array(idx.length*3), C = new Float32Array(idx.length*3);
  idx.forEach((i, j) => { P.set(pos.subarray(3*i, 3*i+3), 3*j); C.set(col.subarray(3*i, 3*i+3), 3*j); });
  g.setAttribute('position', new THREE.BufferAttribute(P, 3)); g.setAttribute('color', new THREE.BufferAttribute(C, 3));
  return new THREE.Points(g, new THREE.PointsMaterial({size: 0.012, vertexColors: true})); }
const scan = cloud(z => z < D.ceil_cut), ceil = cloud(z => z >= D.ceil_cut); ceil.visible = false;
scene.add(scan, ceil);
// markers
function label(text, size, color){ const c = document.createElement('canvas'); c.width = 128; c.height = 64;
  const x = c.getContext('2d'); x.font = 'bold 44px system-ui'; x.fillStyle = color; x.textAlign = 'center'; x.textBaseline = 'middle';
  x.fillText(text, 64, 34); const s = new THREE.Sprite(new THREE.SpriteMaterial({map: new THREE.CanvasTexture(c), depthTest: false}));
  s.scale.set(size*2, size, 1); return s; }
for (const m of D.markers) {
  const g = new THREE.Mesh(new THREE.PlaneGeometry(D.msize, D.msize), new THREE.MeshBasicMaterial({color: 0x111111, side: THREE.DoubleSide}));
  g.quaternion.set(m.q[0], m.q[1], m.q[2], m.q[3]); g.position.set(m.x, m.y, m.z); scene.add(g);
  const l = label(String(m.id), 0.14, dark ? '#ffd166' : '#b7410e');
  const yaw = m.yaw*Math.PI/180; l.position.set(m.x + 0.12*Math.cos(yaw), m.y + 0.12*Math.sin(yaw), m.z + 0.14); scene.add(l); }
// boxes
function box(b, color, opacity, solid){ const sx = b.x[1]-b.x[0], sy = b.y[1]-b.y[0], sz = b.z[1]-b.z[0];
  const geo = new THREE.BoxGeometry(sx, sy, sz); const grp = new THREE.Group();
  if (solid) grp.add(new THREE.Mesh(geo, new THREE.MeshBasicMaterial({color, transparent: true, opacity, depthWrite: false})));
  grp.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo), new THREE.LineBasicMaterial({color})));
  grp.position.set((b.x[0]+b.x[1])/2, (b.y[0]+b.y[1])/2, (b.z[0]+b.z[1])/2); scene.add(grp); return grp; }
const greens = [0x2f9e44, 0x66d17a, 0x0b7a35, 0x94e2a4]; const div = document.getElementById('boxes');
D.boxes.forEach((b, i) => { const g = box(b, greens[i % 4], 0.28, true);
  const lab = document.createElement('label');
  lab.innerHTML = `<input type="checkbox" checked><span class="sw" style="background:#${greens[i%4].toString(16).padStart(6,'0')}"></span>${b.name} : x ${b.x[0]}…${b.x[1]}, y ${b.y[0]}…${b.y[1]}, z ${b.z[0]}…${b.z[1]} m`;
  lab.querySelector('input').onchange = e => g.visible = e.target.checked; div.appendChild(lab); });
const old = D.old ? box(D.old, 0x888888, 0, false) : null;
if (D.path) { const g = new THREE.BufferGeometry().setFromPoints(D.path.map(p => new THREE.Vector3(p[0], p[1], p[2])));
  scene.add(new THREE.Line(g, new THREE.LineBasicMaterial({color: 0xd9480f})));
  const l = label('patrouille', 0.12, '#d9480f'); l.position.set(D.path[0][0], D.path[0][1], D.path[0][2] + 0.1); scene.add(l); }
document.getElementById('old').onchange = e => { if (old) old.visible = e.target.checked; };
document.getElementById('ceil').onchange = e => ceil.visible = e.target.checked;
document.getElementById('scan').onchange = e => scan.visible = e.target.checked;
document.getElementById('info').textContent = D.info;
// floor grid + axes
const grid = new THREE.GridHelper(6, 12, 0x999999, dark ? 0x333333 : 0xdddddd); grid.rotation.x = Math.PI/2; grid.position.set(0.2, 1.6, 0); scene.add(grid);
scene.add(new THREE.AxesHelper(0.5));
(function loop(){ requestAnimationFrame(loop); renderer.render(scene, cam); })();
</script></body></html>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("obj")
    ap.add_argument("--zone", default=str(REPO / "config" / "flight_zone_3d.yaml"))
    ap.add_argument("--markers", default=str(REPO / "config" / "markers.yaml"))
    ap.add_argument("--old", default=str(REPO / "config" / "flight_zone_band.yaml"))
    ap.add_argument("--voxel", type=float, default=0.03)
    ap.add_argument("--ceil-cut", type=float, default=2.65, help="points above: ceiling (hidden by default)")
    ap.add_argument("--out", default="zone_view.html")
    a = ap.parse_args()

    V, F = load_obj(a.obj)
    P = surface_points(V, F)
    key = np.floor(P / a.voxel).astype(np.int64)
    _, keep = np.unique(key, axis=0, return_index=True)
    P = P[keep]
    q = np.round(P * 100).astype(np.int16)
    zone = yaml.safe_load(open(a.zone))
    mk = yaml.safe_load(open(a.markers))
    old = None
    if a.old and Path(a.old).exists():
        o = yaml.safe_load(open(a.old))
        old = {"x": [o["x_min"], o["x_max"]], "y": [o["y_min"], o["y_max"]], "z": [o["z_min"], o["z_max"]]}
    vol = sum((b["x"][1] - b["x"][0]) * (b["y"][1] - b["y"][0]) * (b["z"][1] - b["z"][0]) for b in zone["boxes"])
    info = f"{len(P)} points de scan, volume proposé {vol:.2f} m³"
    if old:
        info += f" (zone actuelle {(old['x'][1]-old['x'][0])*(old['y'][1]-old['y'][0])*(old['z'][1]-old['z'][0]):.2f} m³)"
    data = {"pts": base64.b64encode(q.tobytes()).decode(), "ceil_cut": a.ceil_cut,
            "markers": [{"id": m["id"], "x": m["x"], "y": m["y"], "z": m["z"], "yaw": m["yaw_deg"], "q": m["q"]}
                        for m in mk["markers"]],
            "msize": mk["marker_size"], "boxes": zone["boxes"], "old": old, "info": info, "path": None}
    if old:  # tello_patrol defaults: 0.35 m inside the zone the driver uses, 0.05 m under its top
        x0, x1, y0, y1, z = old["x"][0] + 0.35, old["x"][1] - 0.35, old["y"][0] + 0.35, old["y"][1] - 0.35, old["z"][1] - 0.05
        data["path"] = [[x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z], [x0, y0, z]]
    Path(a.out).write_text(PAGE.replace("__DATA__", json.dumps(data)), encoding="utf-8")
    print(f"{a.out}: {len(P)} points, {len(zone['boxes'])} boxes, {info}")


if __name__ == "__main__":
    main()
