#!/usr/bin/env python3
"""Replay a recorded flight (tello_step_test / tello_marker_tour session) through the drone EKF.

    python tools/ekf_replay.py ~/tello_ws/sessions/step_<time> [--blackout 2.0 --every 6.0] [--plot out.png]

Every `every` seconds the ArUco poses are hidden for `blackout` seconds: the EKF predicts from the commands
(dead reckoning) and its estimate is compared with the hidden poses, against the naive "last known pose".
That measures how long the drone can fly without markers, with the identified model.
"""
import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src" / "tello_driver"))
from tello_driver.ekf_core import DroneEKF, IX, IY, IZ, IYAW  # noqa: E402


def load(folder):
    f = Path(folder)
    P = [(float(r["t"]), "pose", np.array([float(r["x"]), float(r["y"]), float(r["z"]), float(r["yaw"])]))
         for r in csv.DictReader(open(f / "poses.csv"))]
    C = [(float(r["t"]), "cmd", np.array([float(r["cx"]), float(r["cy"]), float(r["cz"]), float(r["cw"])]))
         for r in csv.DictReader(open(f / "commands.csv"))]
    T = []
    for r in csv.DictReader(open(f / "telemetry.csv")):
        try:
            T.append((float(r["t"]), "tel", (float(r["tof"]), float(r["yaw"]))))
        except (KeyError, ValueError):
            pass
    ev = sorted(P + C + T, key=lambda e: e[0])
    t0 = C[0][0] if C else P[0][0]
    return [e for e in ev if e[0] >= t0 - 1.0]


def replay(ev, blackout, every, ekf_kwargs=None):
    ekf = DroneEKF(**(ekf_kwargs or {}))
    t_start = ev[0][0]
    rows = []           # (time since blackout start, ekf error, last-pose error, sigma)
    track = []          # (t, est x, est y, aruco x, aruco y, hidden)
    last_pose = None
    for t, kind, v in ev:
        if kind == "cmd":
            ekf.predict(t)
            ekf.add_command(t, v)
        elif kind == "tel":
            tof, yaw_deg = v
            ekf.predict(t)
            if 15 <= tof <= 400:
                ekf.update_height(tof / 100.0)
            ekf.update_imu_yaw(-math.radians(yaw_deg))
        else:
            phase = (t - t_start) % every
            hidden = blackout > 0 and ekf.initialised and (every - blackout) <= phase
            if hidden:
                ekf.predict(t)
                # the pose describes the drone L_v ago: compare with the EKF state of that time
                t_meas = t - ekf.L_v
                st = next((h for h in reversed(ekf.hist) if h[0] <= t_meas), None)
                est = st[1] if st is not None else ekf.s
                e_ekf = math.hypot(est[IX] - v[0], est[IY] - v[1])
                e_last = math.hypot(last_pose[0] - v[0], last_pose[1] - v[1]) if last_pose is not None else float("nan")
                rows.append((phase - (every - blackout), e_ekf, e_last, ekf.sigma_xy()))
                track.append((t, est[IX], est[IY], v[0], v[1], True))
            else:
                ekf.update_pose(t, v, 0.02, math.radians(1.5))
                last_pose = v
                track.append((t, ekf.s[IX], ekf.s[IY], v[0], v[1], False))
    return np.array(rows), track


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--blackout", type=float, default=2.0)
    ap.add_argument("--every", type=float, default=6.0)
    ap.add_argument("--plot", default="")
    a = ap.parse_args()
    ev = load(a.folder)
    rows, track = replay(ev, a.blackout, a.every)
    if not len(rows):
        print("no hidden poses")
        return
    print(f"{len(rows)} hidden poses. Horizontal error vs time without markers (median, cm):")
    print(f"{'without markers':>16s} {'EKF':>6s} {'last pose':>10s} {'EKF sigma':>10s}")
    for lo in np.arange(0, a.blackout, 0.5):
        m = (rows[:, 0] >= lo) & (rows[:, 0] < lo + 0.5)
        if m.any():
            print(f"{lo:5.1f}-{lo + 0.5:3.1f} s {np.median(rows[m, 1]) * 100:13.1f} {np.median(rows[m, 2]) * 100:10.1f}"
                  f" {np.median(rows[m, 3]) * 100:10.1f}")
    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        T = np.array([(r[0], r[1], r[2], r[3], r[4], r[5]) for r in track], float)
        fig, axs = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
        for k, (ax, name) in enumerate(zip(axs, ("x", "y"))):
            ax.plot(T[:, 0] - T[0, 0], T[:, 3 + k], ".", ms=2, color="tab:blue", label="ArUco")
            ax.plot(T[:, 0] - T[0, 0], T[:, 1 + k], "-", lw=1, color="tab:orange", label="EKF")
            hid = T[:, 5] > 0
            ax.plot(T[hid, 0] - T[0, 0], T[hid, 3 + k], ".", ms=3, color="tab:red", label="ArUco hidden from the EKF")
            ax.set_ylabel(f"{name} [m]")
        axs[0].legend(loc="upper right", fontsize=8)
        axs[0].set_title(f"EKF replay of {Path(a.folder).name}: markers hidden {a.blackout:g} s every {a.every:g} s")
        axs[1].set_xlabel("time [s]")
        fig.tight_layout()
        fig.savefig(a.plot, dpi=110)


if __name__ == "__main__":
    main()
