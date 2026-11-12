#!/usr/bin/env python3
"""Learn where the takeoff pad (marker 48 on the floor) is, from real flights that took off from it.

    python3 tools/set_home.py              # adds the latest session
    python3 tools/set_home.py <session>... # adds these sessions
    python3 tools/set_home.py --reset      # forgets everything (the pad was moved)

The drone camera looks forward and never sees the pad; but the drone climbs straight up after takeoff, so the
first wall-marker poses of the climb give the takeoff point and the heading the drone was put on the pad with.
Every flight adds one sample; config/home.yaml keeps them all and the pad is their median (a flight that did not
start from the pad shows up as an outlier and is ignored). config/home.yaml is local (not in git).
"""
import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
HOME = ROOT / "config" / "home.yaml"
SESSIONS = Path.home() / "tello_ws" / "sessions"


def rows(path):
    if not path.exists():
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def takeoff_point(ses):
    """(x, y, yaw_deg, n_poses) of the takeoff, from the first ArUco poses of the climb, or None."""
    tel = rows(ses / "telemetry.csv")
    fly = [(float(r["t"]), r.get("flying") == "True") for r in tel if r.get("t")]
    t_up = next((t for t, f in fly if f), None)
    if t_up is None:
        return None
    P = np.array([[float(r["t"]), float(r["x"]), float(r["y"]), float(r["yaw_deg"])] for r in rows(ses / "poses.csv")])
    if not len(P):
        return None
    climb = P[(P[:, 0] > t_up) & (P[:, 0] < t_up + 8.0)]  # takeoff + climb, before any horizontal move
    if len(climb) < 3:
        return None
    first = climb[climb[:, 0] < climb[0, 0] + 1.0]  # the first second with markers
    yaw = math.degrees(math.atan2(np.median(np.sin(np.radians(first[:, 3]))), np.median(np.cos(np.radians(first[:, 3])))))
    return float(np.median(first[:, 1])), float(np.median(first[:, 2])), yaw, len(first)


def landing_point(ses):
    """(x, y) of the last ArUco poses before the landing, or None."""
    tel = rows(ses / "telemetry.csv")
    fly = [(float(r["t"]), r.get("flying") == "True") for r in tel if r.get("t")]
    t_down = next((t for t, f in reversed(fly) if f), None)
    P = np.array([[float(r["t"]), float(r["x"]), float(r["y"])] for r in rows(ses / "poses.csv")])
    if t_down is None or not len(P):
        return None
    last = P[(P[:, 0] < t_down - 2.0) & (P[:, 0] > t_down - 4.0)]  # just before the descent
    return (float(np.median(last[:, 1])), float(np.median(last[:, 2]))) if len(last) else None


def load():
    try:
        return yaml.safe_load(open(HOME, encoding="utf-8")) or {}
    except OSError:
        return {}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="*")
    ap.add_argument("--marker", type=int, default=48)
    ap.add_argument("--reset", action="store_true")
    a = ap.parse_args()
    h = {} if a.reset else load()
    hist = h.get("history") or []
    if a.reset and not a.sessions:
        HOME.write_text(f"pad_marker: {a.marker}\n", encoding="utf-8")
        print(f"{HOME}: pad forgotten; fly once from the pad, then run this tool again")
        return
    sessions = [Path(s).expanduser() for s in a.sessions] or \
        sorted((p for p in SESSIONS.glob("2*") if p.is_dir() and (p / "telemetry.csv").exists()), key=lambda p: p.name)[-1:]
    done = {s[3] for s in hist}
    for ses in sessions:
        if ses.name in done:
            print(f"{ses.name}: already counted")
            continue
        tp = takeoff_point(ses)
        if tp is None:
            print(f"{ses.name}: no takeoff with markers in view during the climb: skipped")
            continue
        hist.append([round(tp[0], 3), round(tp[1], 3), round(tp[2], 1), ses.name])
        print(f"{ses.name}: took off at ({tp[0]:+.2f}, {tp[1]:+.2f}), heading {tp[2]:.0f} deg ({tp[3]} poses)")
    if not hist:
        sys.exit("no usable flight: the pad position is still unknown")
    H = np.array([s[:3] for s in hist], float)
    x, y = np.median(H[:, 0]), np.median(H[:, 1])
    d = np.hypot(H[:, 0] - x, H[:, 1] - y)
    keep = d < 0.25  # flights that did not start from the pad
    x, y = np.median(H[keep, 0]), np.median(H[keep, 1])
    yaw = math.degrees(math.atan2(np.median(np.sin(np.radians(H[keep, 2]))), np.median(np.cos(np.radians(H[keep, 2])))))
    spread = np.std(np.hypot(H[keep, 0] - x, H[keep, 1] - y)) if keep.sum() > 1 else float("nan")
    lines = [
        "# Takeoff pad: marker 48 lying on the floor, the drone is always put on it (tools/set_home.py).",
        "# Learnt from the first wall-marker poses of each climb; the pad = median of the flights below.",
        f"pad_marker: {h.get('pad_marker', a.marker)}",
        f"x: {x:.3f}",
        f"y: {y:.3f}",
        f"yaw_deg: {yaw:.1f}           # heading of the drone on the pad",
        f"samples: {int(keep.sum())}",
        "history:                 # [x, y, heading, session]",
    ] + [f"  - [{s[0]}, {s[1]}, {s[2]}, {s[3]}]" for s in hist]
    HOME.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"\npad at ({x:+.2f}, {y:+.2f}), heading {yaw:.0f} deg, from {keep.sum()}/{len(hist)} flights"
          + (f" (spread {spread * 100:.0f} cm)" if keep.sum() > 1 else "") + f" -> {HOME}")
    if (~keep).any():
        print("ignored (not from the pad?): " + ", ".join(s[3] for s, k in zip(hist, keep) if not k))


if __name__ == "__main__":
    main()
