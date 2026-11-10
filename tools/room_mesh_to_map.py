#!/usr/bin/env python3
"""Put the room scan (Scaniverse OBJ) in the `map` frame, for RViz.

    python tools/room_mesh_to_map.py <scan.obj> <alignment.npz> <out_dir> [--ceiling-cut 2.45] [--max-texture 4096]

alignment.npz holds the scan -> map similarity found from the markers: p_map = s * R @ p_scan + t.
Writes <out_dir>/room_map.obj (+ .mtl + texture). Faces above --ceiling-cut are dropped, so that RViz can look
into the room from above. Needs numpy and opencv (texture resize).
"""
import argparse
from pathlib import Path

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("obj")
    ap.add_argument("alignment")
    ap.add_argument("out_dir")
    ap.add_argument("--ceiling-cut", type=float, default=2.45, help="drop faces above this height (m)")
    ap.add_argument("--max-texture", type=int, default=4096)
    a = ap.parse_args()
    al = np.load(a.alignment)
    s, R, t = float(al["s"]), al["R"], al["t"]
    src = Path(a.obj)
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    V, VT, F = [], [], []
    tex = None
    for line in open(src, encoding="utf-8", errors="ignore"):
        if line.startswith("v "):
            V.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("vt "):
            VT.append(line)
        elif line.startswith("f "):
            F.append(line.split()[1:])
    for line in open(src.with_suffix(".mtl"), encoding="utf-8", errors="ignore"):
        if line.strip().startswith("map_Kd"):
            tex = src.parent / line.split(None, 1)[1].strip()
    M = s * np.asarray(V) @ R.T + t
    keep = [f for f in F if max(M[int(c.split("/")[0]) - 1, 2] for c in f) <= a.ceiling_cut]

    tex_name = None
    if tex and tex.exists():
        import cv2
        img = cv2.imread(str(tex))
        k = a.max_texture / max(img.shape[:2])
        if k < 1:
            img = cv2.resize(img, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
        tex_name = "room_texture.jpg"
        cv2.imwrite(str(out / tex_name), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
    with open(out / "room_map.mtl", "w") as f:
        f.write("newmtl main\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nd 1\nillum 1\n")
        if tex_name:
            f.write(f"map_Kd {tex_name}\n")
    with open(out / "room_map.obj", "w") as f:
        f.write("# room scan in the map frame (tools/room_mesh_to_map.py)\nmtllib room_map.mtl\nusemtl main\n")
        for p in M:
            f.write(f"v {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
        f.writelines(VT)
        for fc in keep:
            f.write("f " + " ".join(fc) + "\n")
    print(f"{len(M)} vertices, {len(keep)}/{len(F)} faces kept (ceiling cut at {a.ceiling_cut} m); "
          f"floor ~{np.percentile(M[:, 2], 1):.2f} m; written to {out}")


if __name__ == "__main__":
    main()
