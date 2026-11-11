#!/usr/bin/env python3
"""Perception latency of a recorded flight: ArUco heading vs IMU heading during turns.

    python tools/measure_latency.py ~/tello_ws/sessions/<time>      (a recorder folder: poses.csv + telemetry.csv)

The IMU heading reaches the PC in the telemetry almost at once; the ArUco heading comes through the whole video
chain (encoding, Wi-Fi, decoding, detection). Both are recorded with their arrival time on the PC. The delay that
best superposes the two heading curves is the perception latency (video + decoding + detection), minus the small
telemetry delay. Needs turns in the flight (a survey turn, the tour, or a few j/l in the teleop).
"""
import csv
import math
import sys
from pathlib import Path

import numpy as np


def load(folder):
    f = Path(folder)
    P = [(float(r["t"]), math.radians(float(r["yaw_deg"] if "yaw_deg" in r else r["yaw"])))
         for r in csv.DictReader(open(f / "poses.csv"))]
    if P and "yaw_deg" not in open(f / "poses.csv").readline():
        P = [(t, y) for t, y in P]  # mission files store radians already
    T = []
    for r in csv.DictReader(open(f / "telemetry.csv")):
        try:
            T.append((float(r["t"]), -math.radians(float(r["yaw"]))))  # Tello yaw: clockwise-positive degrees
        except (KeyError, ValueError):
            pass
    return np.array(P), np.array(T)


def measure(P, T, max_lag=1.0):
    if len(P) < 50 or len(T) < 50:
        return None
    ta, ya = P[:, 0], np.unwrap(P[:, 1])
    ti, yi = T[:, 0], np.unwrap(T[:, 1])
    # keep the moments when the drone is turning (the latency is invisible while the heading is constant)
    rate = np.abs(np.gradient(yi, ti))
    turning = np.interp(ta, ti, rate) > math.radians(15)
    if turning.sum() < 20:
        return None
    best = None
    lags = np.arange(-0.2, max_lag, 0.01)
    errs = []
    for L in lags:
        yi_at = np.interp(ta - L, ti, yi)       # IMU heading L seconds before the ArUco pose arrived
        d = ya - yi_at
        d -= np.median(d[turning])             # the two headings differ by a constant offset
        e = math.sqrt(np.mean(np.angle(np.exp(1j * d[turning])) ** 2))
        errs.append(e)
        if best is None or e < best[1]:
            best = (L, e)
    return best[0], math.degrees(best[1]), int(turning.sum()), lags, np.degrees(errs)


def main():
    folder = sys.argv[1]
    P, T = load(folder)
    r = measure(P, T)
    if r is None:
        print("not enough turns in this flight (turn the drone a few times: survey, tour, or j/l in the teleop)")
        return
    L, e, n, lags, errs = r
    msg = (f"perception latency (video + decoding + detection): {L * 1000:.0f} ms "
           f"(heading residual {e:.1f} deg over {n} poses while turning)")
    # a trustworthy result has a clear minimum inside the search range and a small residual; large residuals
    # come from heading jumps of the marker map (inconsistent markers), not from the latency
    if L <= lags[0] + 1e-6 or L >= lags[-1] - 0.011 or e > 3.0:
        msg += "  -> UNRELIABLE (no clear minimum or heading jumps): redo with slow back-and-forth turns facing the markers"
    print(msg)


if __name__ == "__main__":
    main()
