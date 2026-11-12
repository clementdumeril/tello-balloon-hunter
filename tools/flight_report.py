#!/usr/bin/env python3
"""Report of one flight: is the localization good enough to fly fast and precisely?

    python3 tools/flight_report.py [session_dir] [--band 1.7 2.0]

session_dir: a folder of ~/tello_ws/sessions written by the session recorder (default: the latest one). Mission
folders (strike_*, tour_*, step_*) whose start falls inside the session are read too (tracking.csv).

Checks, while flying:
  - markers in view: share of the time with an ArUco pose less than 0.3 s old, longest blind period;
  - pose jumps: ArUco poses more than 15 cm away from the EKF prediction (a wrong marker pose, or a map error);
  - height: ArUco z minus the distance sensor (ToF). A correct marker map gives a constant offset of a few cm
    (the ToF looks at what is under the drone: over the bed or the desk it reads less, those samples are dropped);
  - band: share of the time inside the height band (band mode);
  - EKF uncertainty; mission tracking error (RMS and max, per phase) when a mission ran.
Writes report.txt (and report.png when matplotlib is there) in the session folder.
"""
import argparse
import csv
import json
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

SESSIONS = Path.home() / "tello_ws" / "sessions"


def read_csv(path):
    if not path.exists() or path.stat().st_size == 0:
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def col(rows, key, default=np.nan):
    out = []
    for r in rows:
        try:
            out.append(float(r[key]))
        except (KeyError, TypeError, ValueError):
            out.append(default)
    return np.array(out, float)


def nearest(t_src, v_src, t):
    """Value of v_src at the times t (nearest sample) and the time distance to it."""
    k = np.clip(np.searchsorted(t_src, t), 1, len(t_src) - 1)
    k = np.where(np.abs(t_src[k - 1] - t) < np.abs(t_src[k] - t), k - 1, k)
    return v_src[k], np.abs(t_src[k] - t)


def folder_time(p):
    try:
        return datetime.strptime("_".join(p.name.split("_")[-2:]), "%Y%m%d_%H%M%S").timestamp()
    except ValueError:
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", nargs="?", default="")
    ap.add_argument("--band", type=float, nargs=2, default=None, help="height band (m), e.g. 1.7 2.0")
    a = ap.parse_args()
    if a.session:
        ses = Path(a.session).expanduser()
    else:
        cands = sorted((p for p in SESSIONS.glob("2*") if p.is_dir()), key=lambda p: p.name)
        if not cands:
            sys.exit(f"no session in {SESSIONS}")
        ses = cands[-1]
    lines = []

    def say(s=""):
        print(s)
        lines.append(s)

    tel = read_csv(ses / "telemetry.csv")
    if not tel:
        sys.exit(f"{ses}: no telemetry.csv")
    t_rel, stamp = col(tel, "t"), col(tel, "stamp")
    t0 = float(np.nanmedian(stamp - t_rel))  # recorder start (epoch): poses.csv and telemetry use times from it
    tt = stamp
    flying = np.array([r.get("flying") == "True" for r in tel])
    tof = col(tel, "tof") / 100.0
    bat = col(tel, "bat")
    if not flying.any():
        say(f"{ses.name}: the drone never flew in this session.")
        (ses / "report.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
        return
    f0, f1 = tt[flying][0], tt[flying][-1]
    say(f"Flight report {ses.name}: {f1 - f0:.0f} s in the air, battery {bat[flying][0]:.0f} -> {bat[flying][-1]:.0f} %")
    say("")

    poses = read_csv(ses / "poses.csv")
    ekf = read_csv(ses / "ekf.csv")
    pt = col(poses, "t") + t0
    P = np.c_[col(poses, "x"), col(poses, "y"), col(poses, "z")] if poses else np.zeros((0, 3))
    air_p = (pt > f0 + 3) & (pt < f1 - 1)  # skip the takeoff and the landing
    verdict = []

    # ---- markers in view
    grid = np.arange(f0 + 3, f1 - 1, 0.05)
    if len(grid) and len(pt):
        age = nearest(pt, pt, grid)[1]
        seen = age < 0.3
        blind, longest, run = 0, 0.0, 0.0
        for s in seen:
            run = 0.0 if s else run + 0.05
            longest = max(longest, run)
        share = seen.mean() * 100
        rate = air_p.sum() / max(f1 - f0 - 4, 1)
        say(f"Markers in view   {share:5.1f} % of the time, {rate:.1f} poses/s, longest blind period {longest:.1f} s")
        verdict.append(("markers in view >= 85 %", share >= 85))
    # ---- pose jumps against the EKF
    if ekf and air_p.any():
        et = col(ekf, "t")
        E = np.c_[col(ekf, "x"), col(ekf, "y"), col(ekf, "z")]
        ex, dt = nearest(et, E, pt[air_p])
        d = np.linalg.norm(P[air_p] - ex, axis=1)
        ok = dt < 0.2
        d = d[ok]
        jumps = int((d > 0.15).sum())
        say(f"ArUco vs EKF      median {np.median(d) * 100:4.1f} cm, 95 % {np.percentile(d, 95) * 100:4.1f} cm, "
            f"jumps > 15 cm: {jumps} ({jumps / max(len(d), 1) * 100:.1f} % of the poses)")
        verdict.append(("pose jumps < 1 %", jumps / max(len(d), 1) < 0.01))
        sig = col(ekf, "sigma_xy")[(et > f0 + 3) & (et < f1 - 1)]
        if len(sig):
            say(f"EKF uncertainty   median {np.nanmedian(sig) * 100:4.1f} cm, max {np.nanmax(sig) * 100:4.1f} cm")
    # ---- height: ArUco z - ToF
    if air_p.any():
        tof_p, dt = nearest(tt, tof, pt[air_p])
        z = P[air_p, 2]
        ok = (dt < 0.6) & (tof_p > 0.3)
        diff = z[ok] - tof_p[ok]
        if len(diff) > 10:
            med = np.median(diff)
            floor_ok = np.abs(diff - med) < 0.15  # over the floor; the bed/desk samples read 0.5-0.9 m less
            off = diff[floor_ok]
            say(f"Height ArUco-ToF  {np.median(off) * 100:+5.1f} cm (spread {np.std(off) * 100:.1f} cm, "
                f"{(~floor_ok).mean() * 100:.0f} % of samples over furniture)")
            verdict.append(("ArUco height = ToF within 5 cm", abs(np.median(off)) < 0.05 and np.std(off) < 0.05))
    # ---- band
    if a.band and ekf:
        et = col(ekf, "t")
        ez = col(ekf, "z")[(et > f0 + 8) & (et < f1 - 1)]  # after the climb
        if len(ez):
            inside = ((ez >= a.band[0] - 0.05) & (ez <= a.band[1] + 0.05)).mean() * 100
            say(f"Height band       {inside:5.1f} % of the time in [{a.band[0]:.2f}, {a.band[1]:.2f}] m (+-5 cm), "
                f"z {np.min(ez):.2f}-{np.max(ez):.2f} m")
            verdict.append(("in the band >= 95 %", inside >= 95))
    # ---- takeoff pad (config/home.yaml): did the flight start and end on it?
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from set_home import HOME, landing_point, takeoff_point  # noqa: E402
    try:
        import yaml
        h = yaml.safe_load(open(HOME, encoding="utf-8")) or {}
    except OSError:
        h = {}
    tp, lp = takeoff_point(ses), landing_point(ses)
    if h.get("x") is not None:
        hx, hy = float(h["x"]), float(h["y"])
        if tp:
            say(f"Takeoff           {np.hypot(tp[0] - hx, tp[1] - hy) * 100:4.0f} cm from the pad, heading "
                f"{tp[2]:.0f} deg (pad {float(h.get('yaw_deg', 0)):.0f})")
        if lp:
            d = np.hypot(lp[0] - hx, lp[1] - hy)
            say(f"Landing           {d * 100:4.0f} cm from the pad")
            verdict.append(("landed on the pad (< 15 cm)", d < 0.15))
    elif tp:
        say(f"Takeoff point     ({tp[0]:+.2f}, {tp[1]:+.2f}), heading {tp[2]:.0f} deg: pad not learnt yet "
            "(python3 tools/set_home.py)")
    # ---- missions in this session: tracking error
    for m in sorted(p for p in SESSIONS.glob("*_2*") if p.is_dir()):
        ft = folder_time(m)
        if ft is None or not (t0 - 5 <= ft <= f1):
            continue
        tr = read_csv(m / "tracking.csv")
        if not tr:  # identification flights (step_*) have no reference trajectory
            continue
        e = np.hypot(col(tr, "xd") - col(tr, "x"), col(tr, "yd") - col(tr, "y"))
        ez = np.abs(col(tr, "zd") - col(tr, "z"))
        ph = np.array([r.get("phase", "") for r in tr])
        say(f"\nMission {m.name}: horizontal tracking error RMS {np.sqrt(np.nanmean(e ** 2)) * 100:.1f} cm, "
            f"max {np.nanmax(e) * 100:.1f} cm; height error RMS {np.sqrt(np.nanmean(ez ** 2)) * 100:.1f} cm")
        for name in dict.fromkeys(ph):
            k = ph == name
            say(f"   {name:<22s} RMS {np.sqrt(np.nanmean(e[k] ** 2)) * 100:5.1f} cm   max {np.nanmax(e[k]) * 100:5.1f} cm")
        verdict.append((f"{m.name}: tracking RMS < 8 cm", np.sqrt(np.nanmean(e ** 2)) < 0.08))
        res = m / "results.csv"
        if res.exists():
            say(f"   results: {res}")
    say("")
    for name, ok in verdict:
        say(f"  [{'OK   ' if ok else 'CHECK'}] {name}")
    (ses / "report.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
        if ekf:
            et = col(ekf, "t") - f0
            for k, c in zip("xyz", "rgb"):
                ax[0].plot(et, col(ekf, k), c, lw=1, label=f"EKF {k}")
        for k, c in zip(range(3), "rgb"):
            ax[0].plot(pt - f0, P[:, k], c + ".", ms=2)
        ax[0].set_ylabel("position (m), dots = ArUco")
        ax[0].legend(loc="upper right", fontsize=8)
        ax[1].plot(tt - f0, tof, "k", lw=1, label="ToF")
        ax[1].plot(pt - f0, P[:, 2], "b.", ms=2, label="ArUco z")
        if a.band:
            for zb in a.band:
                ax[1].axhline(zb, color="g", ls="--", lw=0.8)
        ax[1].set_ylabel("height (m)")
        ax[1].set_xlabel("time since takeoff (s)")
        ax[1].legend(loc="upper right", fontsize=8)
        ax[1].set_xlim(-2, f1 - f0 + 2)
        fig.tight_layout()
        fig.savefig(ses / "report.png", dpi=110)
        print(f"\nwrote {ses / 'report.txt'} and report.png")
    except ImportError:
        print(f"\nwrote {ses / 'report.txt'} (no matplotlib: no plot)")


if __name__ == "__main__":
    main()
