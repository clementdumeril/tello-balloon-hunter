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


def load_zone(path):
    """config/flight_zone.yaml -> dict: the mission rectangle (x_min ... z_max), the margin, and when present the
    free-space grid (distance to the nearest obstacle, tools/free_zone.py). None if no file."""
    if not path:
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            z = yaml.safe_load(f)
        zone = {k: float(z[k]) for k in ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max")}
        zone["margin"] = float(z.get("margin", 0.35))
        if z.get("grid_file"):
            g = np.load(Path(path).parent / z["grid_file"])
            c = g["clearance"].astype(float)
            gy, gx = np.gradient(c)
            # inside an obstacle / outside the scan the clearance is a flat 0: no gradient, so the guard did not push
            # at all (offline stress test, seed 367: the drone stayed at x = 2.2 m). There: point to the nearest
            # cell with room (margin + 10 cm)
            from scipy.ndimage import distance_transform_edt
            margin = zone["margin"]
            bad = c < margin + 0.1
            if bad.any() and (~bad).any():
                _, (ii, jj) = distance_transform_edt(bad, return_indices=True)
                I, J = np.indices(c.shape)
                flat = bad & (np.hypot(gx, gy) < 1e-6) | (c < margin)
                gx = np.where(flat, jj - J, gx).astype(float)
                gy = np.where(flat, ii - I, gy).astype(float)
            zone["grid"] = {"d": c, "gx": gx, "gy": gy, "x0": float(g["x0"]), "y0": float(g["y0"]),
                            "cell": float(g["cell"])}
        return zone
    except Exception:  # noqa: BLE001
        return None


def clearance(zone, x, y):
    """(distance to the nearest obstacle in m, unit vector pointing away from it) at (x, y); None without grid."""
    g = zone.get("grid") if zone else None
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
