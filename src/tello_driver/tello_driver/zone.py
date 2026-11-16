"""Obstacle-free flight zone from the room scan (tools/free_zone.py), without ROS.

load_zone(file)       config/flight_zone*.yaml -> dict: mission rectangle (x_min ... z_max), margin, and the
                      free-space grid (distance to the nearest obstacle at the flight height)
clearance(zone, x, y) -> (distance to the nearest obstacle in m, unit vector pointing away from it)
"""
import math
from pathlib import Path

import numpy as np
import yaml


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def make_grid(c, x0, y0, cell, margin):
    """Clearance grid (rows = y, columns = x) -> the dict clearance() reads: values + direction away from obstacles."""
    gy, gx = np.gradient(c)
    # inside an obstacle / outside the scan the clearance is a flat 0: no gradient, so the guard did not push
    # at all (offline stress test, seed 367: the drone stayed at x = 2.2 m). There: point to the nearest
    # cell with room (margin + 10 cm)
    from scipy.ndimage import distance_transform_edt
    bad = c < margin + 0.1
    if bad.any() and (~bad).any():
        _, (ii, jj) = distance_transform_edt(bad, return_indices=True)
        I, J = np.indices(c.shape)
        flat = bad & (np.hypot(gx, gy) < 1e-6) | (c < margin)
        gx = np.where(flat, jj - J, gx).astype(float)
        gy = np.where(flat, ii - I, gy).astype(float)
    return {"d": c, "gx": gx, "gy": gy, "x0": float(x0), "y0": float(y0), "cell": float(cell)}


def box_layers(boxes, margin, cell=0.02, pad=0.6):
    """3D zone = union of boxes (config/flight_zone_3d.yaml: each box = where the drone CENTRE may be, already
    `margin` away from every scanned surface). Cut into height layers between the box floors/tops; per layer a
    clearance grid like the scan one: margin + depth inside the union, falling to 0 at `margin` outside it.
    -> [(z_lo, z_hi, grid)], lowest first."""
    from scipy.ndimage import distance_transform_edt
    xs0, xs1 = min(b["x"][0] for b in boxes) - pad, max(b["x"][1] for b in boxes) + pad
    ys0, ys1 = min(b["y"][0] for b in boxes) - pad, max(b["y"][1] for b in boxes) + pad
    nx, ny = int(math.ceil((xs1 - xs0) / cell)) + 1, int(math.ceil((ys1 - ys0) / cell)) + 1
    X, Y = xs0 + cell * np.arange(nx)[None, :], ys0 + cell * np.arange(ny)[:, None]
    zs = sorted({float(b["z"][0]) for b in boxes} | {float(b["z"][1]) for b in boxes})
    layers = []
    for lo, hi in zip(zs[:-1], zs[1:]):
        inside = np.zeros((ny, nx), bool)
        for b in boxes:
            if b["z"][0] <= lo + 1e-6 and b["z"][1] >= hi - 1e-6:
                inside |= (X >= b["x"][0]) & (X <= b["x"][1]) & (Y >= b["y"][0]) & (Y <= b["y"][1])
        if not inside.any():
            continue
        c = np.where(inside, margin + distance_transform_edt(inside) * cell,
                     np.maximum(0.0, margin - distance_transform_edt(~inside) * cell))
        layers.append((lo, hi, make_grid(c, xs0, ys0, cell, margin)))
    return layers


def load_zone(path):
    """config/flight_zone*.yaml -> dict: the mission rectangle (x_min ... z_max), the margin, and when present the
    free-space grid (distance to the nearest obstacle, tools/free_zone.py). None if no file.
    A 3D zone (key 'boxes', config/flight_zone_3d.yaml) gives one grid per height layer ('layers'); 'grid' is
    then the smallest layer (the one valid at every height), used when the height is not given."""
    if not path:
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            z = yaml.safe_load(f)
        if "boxes" in z:
            B = z["boxes"]
            zone = {"x_min": min(b["x"][0] for b in B), "x_max": max(b["x"][1] for b in B),
                    "y_min": min(b["y"][0] for b in B), "y_max": max(b["y"][1] for b in B),
                    "z_min": min(b["z"][0] for b in B), "z_max": max(b["z"][1] for b in B)}
            zone = {k: float(v) for k, v in zone.items()}
            zone["margin"] = float(z.get("margin", 0.25))
            zone["boxes"] = [{k: [float(a) for a in b[k]] for k in ("x", "y", "z")} for b in B]
            zone["layers"] = box_layers(zone["boxes"], zone["margin"])
            zone["grid"] = min(zone["layers"], key=lambda L: int((L[2]["d"] >= zone["margin"]).sum()))[2]
            return zone
        zone = {k: float(z[k]) for k in ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max")}
        zone["margin"] = float(z.get("margin", 0.35))
        if z.get("grid_file"):
            g = np.load(Path(path).parent / z["grid_file"])
            zone["grid"] = make_grid(g["clearance"].astype(float), g["x0"], g["y0"], g["cell"], zone["margin"])
        return zone
    except Exception:  # noqa: BLE001
        return None


def grid_at(zone, z=None):
    """The clearance grid valid at height z (3D zone: its layer, 5 cm of slack under a layer floor so that the
    height noise at 2.08 m does not flip the drone in and out of the upper layer)."""
    if z is None or "layers" not in zone:
        return zone.get("grid")
    L = zone["layers"]
    zc = min(max(z, L[0][0]), L[-1][1])
    best = L[0]
    for lay in L:
        if zc >= lay[0] - 0.05:
            best = lay
    return best[2]


def height_limits(zone, x, y):
    """(lowest, highest) height allowed at (x, y): 3D zone = the boxes over that point; else the zone band."""
    if not zone or "boxes" not in zone:
        return (zone["z_min"], zone["z_max"]) if zone else None
    t = 0.05
    over = [b for b in zone["boxes"] if b["x"][0] - t <= x <= b["x"][1] + t and b["y"][0] - t <= y <= b["y"][1] + t]
    if not over:  # outside the zone horizontally (being pushed back in): the limits of the nearest box
        def dist(b):
            return math.hypot(max(b["x"][0] - x, 0, x - b["x"][1]), max(b["y"][0] - y, 0, y - b["y"][1]))
        dmin = min(dist(b) for b in zone["boxes"])
        over = [b for b in zone["boxes"] if dist(b) <= dmin + t]
    return min(b["z"][0] for b in over), max(b["z"][1] for b in over)


def clearance(zone, x, y, z=None):
    """(distance to the nearest obstacle in m, unit vector pointing away from it) at (x, y) (and height z for a
    3D zone: without it, the layer valid at every height); None without grid."""
    g = grid_at(zone, z) if zone else None
    if g is None:
        return None
    j = int(round((x - g["x0"]) / g["cell"]))
    i = int(round((y - g["y0"]) / g["cell"]))
    if not (0 <= i < g["d"].shape[0] and 0 <= j < g["d"].shape[1]):
        # outside the scanned area: back towards the middle of the flight zone
        ux, uy = (zone["x_min"] + zone["x_max"]) / 2 - x, (zone["y_min"] + zone["y_max"]) / 2 - y
        n = math.hypot(ux, uy)
        return 0.0, ((ux / n, uy / n) if n > 1e-6 else (0.0, 0.0))
    gx, gy = g["gx"][i, j], g["gy"][i, j]
    n = math.hypot(gx, gy)
    return float(g["d"][i, j]), ((gx / n, gy / n) if n > 1e-6 else (0.0, 0.0))
