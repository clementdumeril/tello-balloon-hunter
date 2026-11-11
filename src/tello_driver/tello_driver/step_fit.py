"""Fit the Tello's response to the identification doublets (tello_step_test) with a first-order + delay model.

    ros2 run tello_driver tello_step_fit ~/tello_ws/sessions/step_<time>

Per axis (forward x, left y, up z, yaw), velocity v answers the commanded velocity u as
    tau * dv/dt = K * u(t - L) - v          (position = integral of v)
K = real speed / commanded speed (1 = the driver gains are right), tau = time constant, L = pure delay
(command -> motion -> camera -> pose). Fitted on the ArUco POSITION (no noisy differentiation): K is linear
(least squares), tau and L by grid search; each doublet gets its own offset + slow drift.
Writes model.yaml and step_fit.png (if matplotlib is installed) in the session folder.
"""
import csv
import math
import sys
from pathlib import Path

import numpy as np

try:
    from scipy.signal import lfilter
except ImportError:  # slower pure-Python fallback
    lfilter = None

AXES = ("x", "y", "z", "yaw")
DT = 0.01


def load(folder):
    folder = Path(folder)
    cmds = list(csv.DictReader(open(folder / "commands.csv")))
    poses = np.array([[float(r["t"]), float(r["x"]), float(r["y"]), float(r["z"]), float(r["yaw"])]
                      for r in csv.DictReader(open(folder / "poses.csv"))])
    return cmds, poses


def segments(cmds, skip=()):
    """{segment id: dict(axis, t (n,), u (n,))} with u the commanded velocity along the tested axis."""
    out = {}
    for r in cmds:
        sid = int(r["segment"])
        if sid < 0 or sid in skip:
            continue
        s = out.setdefault(sid, {"axis": r["axis"], "t": [], "u": []})
        s["t"].append(float(r["t"]))
        s["u"].append(float(r["u"]))
    for s in out.values():
        s["t"], s["u"] = np.array(s["t"]), np.array(s["u"])
    return out


def measured(seg, poses):
    """Pose samples inside the segment, expressed along the tested axis (body axis at the segment start)."""
    t0, t1 = seg["t"][0], seg["t"][-1]
    m = (poses[:, 0] >= t0) & (poses[:, 0] <= t1)
    P = poses[m]
    if len(P) < 10:
        return None, None
    pre = P[P[:, 0] < t0 + 1.0]
    yaw0 = float(np.median(pre[:, 4])) if len(pre) else float(P[0, 4])
    ax = seg["axis"]
    if ax == "x":
        y = P[:, 1] * math.cos(yaw0) + P[:, 2] * math.sin(yaw0)
    elif ax == "y":
        y = -P[:, 1] * math.sin(yaw0) + P[:, 2] * math.cos(yaw0)
    elif ax == "z":
        y = P[:, 3]
    else:
        y = np.unwrap(P[:, 4])
    return P[:, 0], y


def response(t_cmd, u, t_eval, tau, L):
    """Position response for K = 1 (first order + delay), sampled at t_eval."""
    t0 = t_cmd[0]
    grid = np.arange(t0, max(t_cmd[-1], t_eval[-1]) + DT, DT)
    idx = np.searchsorted(t_cmd, grid - L, side="right") - 1
    ug = np.where(idx >= 0, u[np.clip(idx, 0, len(u) - 1)], 0.0)
    a = DT / tau
    if lfilter is not None:  # v[k] = (1 - a) v[k-1] + a u[k-1]
        v = lfilter([0.0, a], [1.0, -(1.0 - a)], ug)
    else:
        v = np.zeros_like(grid)
        for k in range(1, len(grid)):
            v[k] = v[k - 1] + a * (ug[k - 1] - v[k - 1])
    p = np.concatenate([[0.0], np.cumsum(v[:-1] * DT)])
    return np.interp(t_eval, grid, p)


def fit_axis(segs, poses):
    data = []
    for s in segs:
        t, y = measured(s, poses)
        if t is not None:
            data.append((s, t, y))
    if not data:
        return None
    n = sum(len(t) for _, t, _ in data)
    best = None
    for tau in np.arange(0.1, 3.01, 0.05):
        for L in np.arange(0.0, 0.81, 0.02):
            A = np.zeros((n, 1 + 2 * len(data)))
            b = np.zeros(n)
            r = 0
            for j, (s, t, y) in enumerate(data):
                k = len(t)
                A[r:r + k, 0] = response(s["t"], s["u"], t, tau, L)
                A[r:r + k, 1 + 2 * j] = 1.0
                A[r:r + k, 2 + 2 * j] = t - t[0]
                b[r:r + k] = y
                r += k
            coef, *_ = np.linalg.lstsq(A, b, rcond=None)
            rms = float(np.sqrt(np.mean((A @ coef - b) ** 2)))
            if best is None or rms < best[0]:
                best = (rms, tau, L, coef, A, b)
    rms, tau, L, coef, A, b = best
    return {"K": float(coef[0]), "tau": float(tau), "L": float(L), "rms": rms, "n_points": int(n),
            "n_doublets": len(data), "_fit": (A @ coef, b, data)}


def fit(folder, verbose=True):
    cmds, poses = load(folder)
    failed = Path(folder) / "failed_segments.txt"
    skip = {int(v) for v in failed.read_text().split()} if failed.exists() else set()
    segs = segments(cmds, skip)
    res = {}
    for ax in AXES:
        r = fit_axis([s for s in segs.values() if s["axis"] == ax], poses)
        if r is not None:
            res[ax] = r
    if verbose:
        unit = {"x": "m", "y": "m", "z": "m", "yaw": "rad"}
        print(f"\n{'axis':6s}{'K':>7s}{'tau [s]':>9s}{'L [s]':>8s}{'rms':>11s}  doublets")
        for ax, r in res.items():
            print(f"{ax:6s}{r['K']:7.2f}{r['tau']:9.2f}{r['L']:8.2f}{r['rms']:8.3f} {unit[ax]:3s}  {r['n_doublets']}")
        print("K: real/commanded speed (1 = driver gains right). tau: time constant. L: delay command -> pose.")
    save(folder, res)
    return res


def save(folder, res):
    folder = Path(folder)
    lines = ["# Tello response, first order + delay per axis: tau dv/dt = K u(t - L) - v  (tello_step_fit)"]
    for ax, r in res.items():
        lines.append(f"{ax}: {{K: {r['K']:.3f}, tau: {r['tau']:.2f}, L: {r['L']:.2f}, rms: {r['rms']:.4f}, "
                     f"doublets: {r['n_doublets']}}}")
    (folder / "model.yaml").write_text("\n".join(lines) + "\n")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    fig, axs = plt.subplots(len(res), 1, figsize=(10, 2.6 * max(len(res), 1)), squeeze=False)
    for a, (ax, r) in zip(axs[:, 0], res.items()):
        model, meas, data = r["_fit"]
        k = 0
        for j, (s, t, y) in enumerate(data):
            n = len(t)
            lbl = (j == 0)
            a.plot(t - data[0][1][0], meas[k:k + n], ".", ms=3, color="tab:blue", label="ArUco" if lbl else None)
            a.plot(t - data[0][1][0], model[k:k + n], "-", color="tab:orange", label="model" if lbl else None)
            a.step(s["t"] - data[0][1][0], meas[k] + s["u"] * 0.5, where="post", color="0.6", lw=0.8,
                   label="command (scaled)" if lbl else None)
            k += n
        a.set_title(f"{ax}: K={r['K']:.2f}  tau={r['tau']:.2f} s  L={r['L']:.2f} s  rms={r['rms']:.3f}")
        a.legend(loc="upper right", fontsize=8)
    axs[-1, 0].set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(folder / "step_fit.png", dpi=110)


def main():
    if len(sys.argv) < 2:
        root = Path.home() / "tello_ws/sessions"
        cands = sorted(root.glob("step_*"))
        if not cands:
            print("usage: tello_step_fit <session folder>")
            return
        folder = cands[-1]
    else:
        folder = Path(sys.argv[1])
    print(f"Fitting {folder}")
    fit(folder)
    print(f"Saved {folder / 'model.yaml'}" + (f" and {folder / 'step_fit.png'}" if (folder / 'step_fit.png').exists() else ""))


if __name__ == "__main__":
    main()
