"""Takeoff pad ("home"): the drone always takes off from the same spot, marker 48 lying on the floor.

config/home.yaml holds where that pad is in the map and the heading the drone is put on it with. It is learnt
from real flights by tools/set_home.py (the drone camera looks forward: it never sees the pad marker itself, but
the wall markers right after takeoff tell where the drone lifted off). Uses:
  - the EKF starts on the pad, before any wall marker is seen (the drone is localised from the first second);
  - the missions fly back above the pad at the end and land on it.
"""
import os
from pathlib import Path

import yaml

REPO = os.environ.get("TELLO_REPO", str(Path.home() / "tello_ws/src/tello-balloon-hunter"))
HOME_FILE = os.path.join(REPO, "config", "home.yaml")


def load_home(path=HOME_FILE):
    """dict(x, y, yaw_deg, marker, samples) or None when the pad position is not known yet."""
    try:
        with open(path, encoding="utf-8") as f:
            h = yaml.safe_load(f) or {}
        if h.get("x") is None or h.get("y") is None:
            return None
        return {"x": float(h["x"]), "y": float(h["y"]), "yaw_deg": float(h.get("yaw_deg", 0.0)),
                "marker": int(h.get("pad_marker", 48)), "samples": int(h.get("samples", 1))}
    except (OSError, ValueError, TypeError, yaml.YAMLError):
        return None
