#!/usr/bin/env python3
"""Fix the scale of the marker map once the printed marker size is measured.

    python tools/rescale_marker_map.py 0.194            # measured black square side, in metres
    python tools/rescale_marker_map.py 0.194 --dry-run  # only print what would change

Why: the whole map (room_mapper.py bundle adjustment) is scaled by the marker size written in markers.yaml.
If the markers printed at 19.4 cm instead of 20 cm, every position is 3 % too large.
What it does:
  - marker_size := measured value
  - every marker x, y, z and the room box multiplied by k = measured / old marker_size, about the map origin
    (the floor point below marker 6, so z stays the height above the floor)
  - scan alignments (scans/*/alignment.npz, P_map = s R P_scan + t) updated: s -> k s, t -> k t
  - backups: config/markers.yaml.bak_before_rescale, alignment_before_rescale.npz
Orientations (yaw_deg) do not change with a uniform scale.
"""
import argparse
import re
import shutil
from datetime import datetime
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
SCANS = REPO.parent / "scans"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("size", type=float, help="measured side of the black square, metres (e.g. 0.194)")
    ap.add_argument("--markers", default=str(REPO / "config" / "markers.yaml"))
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    if not 0.05 < a.size < 0.5:
        raise SystemExit(f"{a.size} m looks wrong: give the size in METRES (19.4 cm -> 0.194)")
    path = Path(a.markers)
    text = path.read_text(encoding="utf-8")
    old = float(re.search(r"^marker_size:\s*([0-9.]+)", text, re.M).group(1))
    k = a.size / old
    print(f"marker_size {old} -> {a.size}: scale factor k = {k:.4f} ({(k - 1) * 100:+.1f} %)")

    def scale_marker(m):
        vals = {key: float(m.group(key)) * k for key in ("x", "y", "z")}
        return f"{m.group('pre')}x: {vals['x']:.3f}, y: {vals['y']:.3f}, z: {vals['z']:.3f},"
    new, n = re.subn(r"(?P<pre>\{id:\s*\d+,\s*)x:\s*(?P<x>-?[0-9.]+),\s*y:\s*(?P<y>-?[0-9.]+),\s*z:\s*(?P<z>-?[0-9.]+),",
                     scale_marker, text)
    new = re.sub(r"^marker_size:\s*[0-9.]+", f"marker_size: {a.size}", new, flags=re.M)
    for key in ("size_x", "size_y", "height"):
        new = re.sub(rf"^(\s+{key}:\s*)([0-9.]+)", lambda m: f"{m.group(1)}{float(m.group(2)) * k:.2f}", new, flags=re.M)
    note = (f"# {datetime.now():%Y-%m-%d %H:%M}: marker_size measured {a.size * 100:.1f} cm (was {old * 100:.1f}): all positions "
            f"x{k:.4f} about the origin (tools/rescale_marker_map.py).\n")
    new = new.replace("\ndictionary:", f"\n{note.rstrip()}\n\ndictionary:", 1) if "\ndictionary:" in new else note + new
    print(f"{n} markers rescaled")
    for line in new.splitlines():
        if line.strip().startswith("- {id:") and ("id:  6" in line or "id:  1," in line):
            print("  ", line.strip())

    aligns = sorted(SCANS.glob("*/alignment.npz")) if SCANS.exists() else []
    for f in aligns:
        d = dict(np.load(f))
        print(f"{f.relative_to(SCANS.parent)}: s {float(d['s']):.4f} -> {float(d['s']) * k:.4f}, "
              f"t {np.round(d['t'], 3)} -> {np.round(d['t'] * k, 3)}")
    if a.dry_run:
        print("dry run: nothing written")
        return
    shutil.copy(path, path.with_name(path.name + ".bak_before_rescale"))
    path.write_text(new, encoding="utf-8", newline="\n")
    for f in aligns:
        d = dict(np.load(f))
        shutil.copy(f, f.with_name("alignment_before_rescale.npz"))
        d["s"], d["t"] = d["s"] * k, d["t"] * k
        np.savez(f, **d)
    print(f"Written {path} (backup .bak_before_rescale) and {len(aligns)} scan alignment(s). Then in WSL: git pull + colcon build.")


if __name__ == "__main__":
    main()
