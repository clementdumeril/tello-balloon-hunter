#!/usr/bin/env python3
"""Flight zone with nothing in it, from the room scan (map frame, made by tools/room_mesh_to_map.py).

    python tools/free_zone.py <room_map.obj> [--z 1.2 2.4] [--margin 0.5] [--out config/flight_zone.yaml]

1. Obstacles = every scan point between z_lo - 0.2 and z_hi + 0.2 (the drone flies in [z_lo, z_hi]), on a
   5 cm grid, plus everything outside the room (flood fill from the room centre).
2. Grown by --margin (drone half-size ~0.1 m + pose error + braking).
3. The largest obstacle-free axis-aligned rectangle -> config/flight_zone.yaml (+ a PNG to check it), used by
   the missions to plan; and the whole free space: distance to the nearest obstacle on the 5 cm grid ->
   config/flight_zone_grid.npz, used by the driver's guard (the drone may use all the free space).
Furniture lower than z_lo - 0.2 (bed, desk) is ignored: the drone flies above it (but takes off/lands on
the floor: start from a free spot of the floor inside the zone).
"""
import argparse
from pathlib import Path

import numpy as np

CELL = 0.05


def load_obj(path):
    V, F = [], []
    for line in open(path, encoding="utf-8", errors="ignore"):
        if line.startswith("v "):
            V.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            F.append([int(c.split("/")[0]) - 1 for c in line.split()[1:4]])
    return np.asarray(V), np.asarray(F)


def surface_points(V, F, step=0.03):
    """Vertices + points inside the big triangles, so that a large flat face (a wardrobe door) is not
    represented only by its 3 corners."""
    pts = [V]
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    size = np.maximum(np.linalg.norm(b - a, axis=1), np.linalg.norm(c - a, axis=1))
    big = size > step
    for k in range(1, 6):
        for j in range(0, 6 - k + 1):
            u, v = k / 6, j / 6
            if u + v <= 1:
                pts.append(a[big] + u * (b[big] - a[big]) + v * (c[big] - a[big]))
    return np.concatenate(pts)


def largest_rectangle(free):
    """Largest all-True axis-aligned rectangle in a boolean grid (histogram method). Returns (i0, i1, j0, j1)."""
    h = np.zeros(free.shape[1], int)
    best, box = 0, None
    for i in range(free.shape[0]):
        h = np.where(free[i], h + 1, 0)
        stack = []
        for j in range(len(h) + 1):
            cur = h[j] if j < len(h) else 0
            start = j
            while stack and stack[-1][1] >= cur:
                s, hh = stack.pop()
                area = hh * (j - s)
                if area > best:
                    best, box = area, (i - hh + 1, i + 1, s, j)
                start = s
            stack.append((start, cur))
    return box


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("obj")
    ap.add_argument("--z", type=float, nargs=2, default=[1.4, 2.4], help="flight heights (m)")
    ap.add_argument("--margin", type=float, default=0.35, help="clearance from any obstacle (m)")
    ap.add_argument("--corridor", type=float, default=0.25,
                    help="free passages narrower than ~2x this are cut (keeps the drone in the main region)")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent.parent / "config" / "flight_zone.yaml"))
    a = ap.parse_args()
    V, F = load_obj(a.obj)
    P = surface_points(V, F)
    z_lo, z_hi = a.z
    obs = P[(P[:, 2] > z_lo - 0.2) & (P[:, 2] < z_hi + 0.2)]
    x0, y0 = np.percentile(V[:, 0], 0.5) - 0.3, np.percentile(V[:, 1], 0.5) - 0.3
    x1, y1 = np.percentile(V[:, 0], 99.5) + 0.3, np.percentile(V[:, 1], 99.5) + 0.3
    nx, ny = int((x1 - x0) / CELL) + 1, int((y1 - y0) / CELL) + 1
    occ = np.zeros((ny, nx), bool)
    ix = ((obs[:, 0] - x0) / CELL).astype(int).clip(0, nx - 1)
    iy = ((obs[:, 1] - y0) / CELL).astype(int).clip(0, ny - 1)
    occ[iy, ix] = True
    # close small holes of the scan walls (1-2 cells) before the flood fill
    import cv2
    walls = cv2.dilate(occ.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
    # inside of the room = flood fill from the centre of the floor points
    floor = V[V[:, 2] < 0.1]
    cx, cy = np.median(floor[:, 0]), np.median(floor[:, 1])
    seed = (int((cx - x0) / CELL), int((cy - y0) / CELL))
    ff = (~walls).astype(np.uint8)
    mask = np.zeros((ny + 2, nx + 2), np.uint8)
    cv2.floodFill(ff, mask, seed, 2)
    inside = ff == 2
    blocked = occ | ~inside
    # distance (m) from every cell to the nearest obstacle or to the outside of the room
    clearance = cv2.distanceTransform((~blocked).astype(np.uint8), cv2.DIST_L2, 5).astype(np.float32) * CELL
    r = int(np.ceil(a.margin / CELL))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
    blocked = cv2.dilate(blocked.astype(np.uint8), kernel) > 0
    box = largest_rectangle(~blocked)
    if box is None:
        raise SystemExit("no free zone found")
    i0, i1, j0, j1 = box
    zx0, zx1 = x0 + j0 * CELL, x0 + j1 * CELL
    zy0, zy1 = y0 + i0 * CELL, y0 + i1 * CELL
    # Keep only the MAIN free region: remove passages narrower than ~2 x --corridor (opening), then the connected
    # part that contains the mission rectangle. Elsewhere (e.g. the alcove behind a narrow gap, where no marker is
    # in view) becomes no-fly: its clearance decreases below the margin towards the outside.
    free = ~blocked
    rc = int(np.ceil(a.corridor / CELL))
    k2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * rc + 1, 2 * rc + 1))
    opened = cv2.morphologyEx(free.astype(np.uint8), cv2.MORPH_OPEN, k2)
    n_lab, lab = cv2.connectedComponents(opened)
    main = lab == lab[(i0 + i1) // 2, (j0 + j1) // 2]
    dist_in = cv2.distanceTransform(main.astype(np.uint8), cv2.DIST_L2, 5) * CELL      # inside main: to its edge
    dist_out = cv2.distanceTransform((~main).astype(np.uint8), cv2.DIST_L2, 5) * CELL  # outside: to main
    clearance = np.where(main, np.minimum(clearance, a.margin + dist_in),
                         np.minimum(clearance, np.maximum(0.0, a.margin - dist_out))).astype(np.float32)
    blocked = ~main
    out = Path(a.out)
    grid = out.with_name(out.stem + "_grid.npz")
    np.savez_compressed(grid, clearance=clearance, x0=x0, y0=y0, cell=CELL, z_lo=z_lo, z_hi=z_hi)
    out.write_text(
        "# Flight zone with no obstacle (tools/free_zone.py, from the room scan): largest free rectangle at\n"
        f"# flight heights, {a.margin} m from any scanned surface between z {z_lo - 0.2:.1f} and {z_hi + 0.2:.1f} m.\n"
        f"x_min: {zx0:.2f}\nx_max: {zx1:.2f}\ny_min: {zy0:.2f}\ny_max: {zy1:.2f}\nz_min: {z_lo:.2f}\nz_max: {z_hi:.2f}\n"
        f"margin: {a.margin}\ngrid_file: {grid.name}\n", encoding="utf-8")
    print(f"free zone x [{zx0:.2f}, {zx1:.2f}] y [{zy0:.2f}, {zy1:.2f}] ({zx1 - zx0:.2f} x {zy1 - zy0:.2f} m), "
          f"z [{z_lo}, {z_hi}] -> {out}")
    # picture: top view, scan in grey, obstacles at flight height red, margin orange, zone green
    img = np.full((ny, nx, 3), 255, np.uint8)
    allp = V
    jx = ((allp[:, 0] - x0) / CELL).astype(int).clip(0, nx - 1)
    jy = ((allp[:, 1] - y0) / CELL).astype(int).clip(0, ny - 1)
    img[jy, jx] = (200, 200, 200)
    img[blocked & ~occ] = (120, 190, 255)
    img[occ] = (40, 40, 220)
    img[i0:i1, j0:j1][~blocked[i0:i1, j0:j1]] = (120, 220, 120)
    img = cv2.resize(img[::-1], None, fx=6, fy=6, interpolation=cv2.INTER_NEAREST)  # y up
    cv2.putText(img, f"free zone {zx1 - zx0:.2f} x {zy1 - zy0:.2f} m (green), obstacles at {z_lo - .2:.1f}-{z_hi + .2:.1f} m (red), "
                f"margin {a.margin} m (orange)", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    cv2.imwrite(str(out.with_suffix(".png")), img)


if __name__ == "__main__":
    main()
