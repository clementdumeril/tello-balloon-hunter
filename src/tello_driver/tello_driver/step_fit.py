"""Fit, check and validate the Tello's response to the identification doublets (tello_step_test).

    ros2 run tello_driver tello_step_fit [step_<time>]                           identification -> model.yaml
    ros2 run tello_driver tello_step_fit --validate step_<time> stepval_<time>    model simulated on another flight

What is identified: the Tello WITH its own (hidden) flight controller, from the velocity command we send to the
velocity it really reaches, per axis (forward x, left y, up z, yaw), as a first order + pure delay:

    tau dv/dt = K u(t - L) - v          position = integral of v

K = real speed / commanded speed, tau = time constant, L = delay command -> marker pose (= L_u command side
+ L_v video side). The Tello gives no access to its motors, so a motor-level identification is not possible.

Method
  * input u = the command REALLY sent (sent.csv: after the driver's guards; Nov 13 the band guard cut an up pulse
    from 0.30 to 0.08 m/s and the fit on the requested command found K_z = 0.43);
  * output = the raw ArUco position along the axis (body axis at the start of the doublet), not the EKF (it uses
    the model being identified), not the velocity (differentiating the pose amplifies its noise);
  * per doublet j:  y(t) = K g_{tau,L}(t) + c_j + d_j (t - t_j): g = simulated response for K = 1; c_j, d_j absorb
    the offset and the Tello's slow drift. For fixed (tau, L) this is linear: least squares; (tau, L) on a grid
    (tau 0.05-1.5 s step 0.025, L 0-0.9 s step 0.01) minimising J(tau, L) = residual sum of squares;
  * uncertainty: the fit again without each doublet in turn (spread of K, tau, L, L + tau);
  * identifiability: the map of J(tau, L) (cost_map.png): a long valley = tau and L trade off, only L + tau is
    well determined;
  * fit % = 100 (1 - ||y - y_model|| / ||y - mean_j(y)||);
  * video delay L_v: ArUco yaw vs IMU yaw (no video in the IMU path) during the yaw doublets; L_u = L - L_v;
  * validation: the identified model simulated on ANOTHER flight (only c_j, d_j fitted), compared with the model
    the EKF / driver use now.
Writes model.yaml, step_fit.png, cost_map.png (identification); validation.txt, validation.png (validation).
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
COL = {"x": "cx", "y": "cy", "z": "cz", "yaw": "cw"}
UNIT = {"x": "m", "y": "m", "z": "m", "yaw": "rad"}
DT = 0.01
TAUS_FINE, LS_FINE = np.arange(0.05, 1.5001, 0.025), np.arange(0.0, 0.9001, 0.01)
TAUS_COARSE, LS_COARSE = np.arange(0.05, 1.5001, 0.05), np.arange(0.0, 0.9001, 0.02)
# the model the EKF / driver use before this identification (Nov 11, map then 6 % too large)
MODEL_IN_USE = {"x": (1.15, 0.55, 0.50), "y": (1.15, 0.55, 0.50), "z": (0.9, 0.45, 0.50), "yaw": (1.0, 0.05, 0.50)}
SESSIONS = Path.home() / "tello_ws" / "sessions"


# ------------------------------------------------------------------ data
def rows(path):
    return list(csv.DictReader(open(path))) if Path(path).exists() else []


def load(folder):
    """(segments, poses array [t, x, y, z, yaw]) with u = the command really sent when sent.csv exists."""
    folder = Path(folder)
    poses = np.array([[float(r["t"]), float(r["x"]), float(r["y"]), float(r["z"]), float(r["yaw"])]
                      for r in rows(folder / "poses.csv")])
    failed = folder / "failed_segments.txt"
    skip = {int(v) for v in failed.read_text().split()} if failed.exists() else set()
    segs = {}
    for r in rows(folder / "commands.csv"):
        sid = int(r["segment"])
        if sid < 0 or sid in skip:
            continue
        s = segs.setdefault(sid, {"axis": r["axis"], "t": [], "u": []})
        s["t"].append(float(r["t"]))
        s["u"].append(float(r["u"]))
    sent = rows(folder / "sent.csv")
    st = np.array([float(r["t"]) for r in sent]) if sent else None
    for s in segs.values():
        s["t"], s["u"] = np.array(s["t"]), np.array(s["u"])
        s["u_asked"] = s["u"].copy()
        if st is not None and len(st):
            v = np.array([float(r[COL[s["axis"]]]) for r in sent])
            i = np.clip(np.searchsorted(st, s["t"], side="right") - 1, 0, len(st) - 1)
            s["u"] = v[i]  # zero-order hold of the command really sent
    return segs, poses, st is not None


def measured(seg, poses):
    """Pose samples inside the segment, along the tested axis (body axis at the segment start)."""
    t0, t1 = seg["t"][0], seg["t"][-1]
    P = poses[(poses[:, 0] >= t0) & (poses[:, 0] <= t1)]
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


# ------------------------------------------------------------------ fit
def solve(data, g, K=None):
    """Least squares of y = K g + c_j + d_j (t - t_j) over the doublets (K fixed if given).
    -> (K, residual sum of squares, model values, measured values)."""
    n = sum(len(t) for _, t, _ in data)
    A = np.zeros((n, (0 if K is not None else 1) + 2 * len(data)))
    b = np.zeros(n)
    r = 0
    off = 0 if K is not None else 1
    for j, ((s, t, y), gj) in enumerate(zip(data, g)):
        k = len(t)
        if K is None:
            A[r:r + k, 0] = gj
            b[r:r + k] = y
        else:
            b[r:r + k] = y - K * gj
        A[r:r + k, off + 2 * j] = 1.0
        A[r:r + k, off + 2 * j + 1] = t - t[0]
        r += k
    coef, *_ = np.linalg.lstsq(A, b, rcond=None)
    model = A @ coef
    if K is None:
        Kf = float(coef[0])
        meas = b
    else:
        Kf = K
        gall = np.concatenate(g)
        model = model + K * gall
        meas = b + K * gall
    return Kf, float(np.sum((model - meas) ** 2)), model, meas


def fit_percent(data, model, meas):
    """100 (1 - ||y - model|| / ||y - mean of each doublet||)."""
    r, dev = 0, []
    for _, t, _ in data:
        k = len(t)
        dev.append(meas[r:r + k] - np.mean(meas[r:r + k]))
        r += k
    return 100.0 * (1.0 - np.linalg.norm(meas - model) / max(np.linalg.norm(np.concatenate(dev)), 1e-9))


def grid_fit(data, taus, Ls, cache=None):
    """Best (tau, L) on the grid. -> (K, tau, L, J map[taus x Ls])."""
    J = np.full((len(taus), len(Ls)), np.inf)
    best = None
    for i, tau in enumerate(taus):
        for k, L in enumerate(Ls):
            g = []
            for j, (s, t, y) in enumerate(data):
                key = (id(s), i, k, len(taus))
                if cache is not None and key in cache:
                    g.append(cache[key])
                else:
                    gj = response(s["t"], s["u"], t, tau, L)
                    if cache is not None:
                        cache[key] = gj
                    g.append(gj)
            K, sse, _, _ = solve(data, g)
            J[i, k] = sse
            if best is None or sse < best[0]:
                best = (sse, K, tau, L)
    return best[1], best[2], best[3], J


def fit_axis(segs, poses):
    data = [(s, *measured(s, poses)) for s in segs]
    data = [d for d in data if d[1] is not None]
    if len(data) < 2:
        return None
    K, tau, L, J = grid_fit(data, TAUS_FINE, LS_FINE)
    g = [response(s["t"], s["u"], t, tau, L) for s, t, _ in data]
    _, sse, model, meas = solve(data, g)
    # uncertainty: leave one doublet out (coarse grid, responses cached across the subsets)
    cache, loo = {}, []
    for j in range(len(data)):
        Kj, tj, Lj, _ = grid_fit(data[:j] + data[j + 1:], TAUS_COARSE, LS_COARSE, cache)
        loo.append((Kj, tj, Lj))
    loo = np.array(loo)
    return {"K": K, "tau": float(tau), "L": float(L), "rms": float(np.sqrt(sse / len(meas))),
            "fit": fit_percent(data, model, meas), "n_doublets": len(data),
            "sd": {"K": float(np.std(loo[:, 0])), "tau": float(np.std(loo[:, 1])), "L": float(np.std(loo[:, 2])),
                   "L+tau": float(np.std(loo[:, 1] + loo[:, 2]))},
            "loo": loo, "J": J, "_fit": (model, meas, data)}


def video_delay(folder):
    """ArUco yaw lags the IMU yaw (no video in its path) by ~L_v: best time shift over the yaw doublets."""
    folder = Path(folder)
    P = np.array([[float(r["t"]), float(r["yaw"])] for r in rows(folder / "poses.csv")])
    T = [(float(r["t"]), -math.radians(float(r["yaw"]))) for r in rows(folder / "telemetry.csv")
         if r.get("yaw") not in (None, "")]
    if len(P) < 50 or len(T) < 50:
        return None
    T = np.array(T)
    segs, _, _ = load(folder)
    win = [(s["t"][0], s["t"][-1]) for s in segs.values() if s["axis"] == "yaw"]
    if not win:
        return None
    ia, ii = np.unwrap(P[:, 1]), np.unwrap(T[:, 1])
    best = None
    for lag in np.arange(-0.2, 1.0, 0.01):
        err, n = 0.0, 0
        for a, b in win:
            tg = np.arange(a + 0.5, b - 0.5, 0.02)
            if len(tg) < 10:
                continue
            ya = np.interp(tg + lag, P[:, 0], ia)
            yi = np.interp(tg, T[:, 0], ii)
            d = (ya - ya.mean()) - (yi - yi.mean())
            err += float(np.sum(d ** 2))
            n += len(tg)
        if n and (best is None or err / n < best[1]):
            best = (lag, err / n)
    return None if best is None else float(best[0])


def fit(folder, verbose=True):
    folder = Path(folder)
    segs, poses, has_sent = load(folder)
    res = {}
    for ax in AXES:
        r = fit_axis([s for s in segs.values() if s["axis"] == ax], poses)
        if r is not None:
            res[ax] = r
    Lv = video_delay(folder)
    if verbose:
        print(f"\nInput: {'commands REALLY sent (sent.csv)' if has_sent else 'commands ASKED (no sent.csv: older flight)'}")
        print(f"{'axis':5s}{'K':>12s}{'tau [s]':>14s}{'L [s]':>14s}{'L+tau [s]':>14s}{'fit %':>8s}{'rms':>9s}  doublets")
        for ax, r in res.items():
            sd = r["sd"]
            print(f"{ax:5s}{r['K']:7.2f}±{sd['K']:<4.2f}{r['tau']:9.2f}±{sd['tau']:<4.2f}{r['L']:9.2f}±{sd['L']:<4.2f}"
                  f"{r['L'] + r['tau']:9.2f}±{sd['L+tau']:<4.2f}{r['fit']:8.1f}{r['rms']:8.3f} {UNIT[ax]:3s} {r['n_doublets']}")
        print("± = spread when each doublet is left out in turn. A large spread of tau and L with a small one of L+tau:")
        print("    they trade off (see cost_map.png); only their sum is well determined.")
        if Lv is not None:
            print(f"Video delay L_v (ArUco yaw vs IMU yaw): {Lv:.2f} s  -> command delay L_u = L - L_v")
    save(folder, res, Lv, has_sent)
    return res


def save(folder, res, Lv, has_sent):
    folder = Path(folder)
    lines = ["# Tello response per axis, first order + delay: tau dv/dt = K u(t - L) - v   (tello_step_fit)",
             f"# input: {'commands really sent' if has_sent else 'commands asked'}; sd = leave-one-doublet-out spread"]
    for ax, r in res.items():
        sd = r["sd"]
        lines.append(f"{ax}: {{K: {r['K']:.3f}, tau: {r['tau']:.3f}, L: {r['L']:.2f}, L_plus_tau: {r['L'] + r['tau']:.3f}, "
                     f"sd_K: {sd['K']:.3f}, sd_tau: {sd['tau']:.3f}, sd_L: {sd['L']:.3f}, sd_L_plus_tau: {sd['L+tau']:.3f}, "
                     f"fit_percent: {r['fit']:.1f}, rms: {r['rms']:.4f}, doublets: {r['n_doublets']}}}")
    if Lv is not None:
        lines.append(f"video_delay_L_v: {Lv:.2f}")
    (folder / "model.yaml").write_text("\n".join(lines) + "\n")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    fig, axs = plt.subplots(len(res), 1, figsize=(11, 2.7 * max(len(res), 1)), squeeze=False)
    for a, (ax, r) in zip(axs[:, 0], res.items()):
        model, meas, data = r["_fit"]
        k, t0 = 0, data[0][1][0]
        for j, (s, t, y) in enumerate(data):
            n = len(t)
            lbl = j == 0
            a.plot(t - t0, meas[k:k + n], ".", ms=2, color="tab:blue", label="ArUco" if lbl else None)
            a.plot(t - t0, model[k:k + n], "-", color="tab:orange", label="model" if lbl else None)
            a.step(s["t"] - t0, meas[k] + s["u"] * 0.5, where="post", color="0.55", lw=0.8,
                   label="command sent (scaled)" if lbl else None)
            k += n
        a.set_title(f"{ax}: K={r['K']:.2f}  tau={r['tau']:.2f} s  L={r['L']:.2f} s  fit {r['fit']:.0f} %")
        a.legend(loc="upper right", fontsize=7)
    axs[-1, 0].set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(folder / "step_fit.png", dpi=105)
    plt.close(fig)
    fig, axs = plt.subplots(1, len(res), figsize=(4.2 * max(len(res), 1), 3.8), squeeze=False)
    for a, (ax, r) in zip(axs[0], res.items()):
        J = r["J"]
        rel = np.log10(J / np.nanmin(J))
        cs = a.contourf(LS_FINE, TAUS_FINE, np.minimum(rel, 1.0), levels=20, cmap="viridis_r")
        a.contour(LS_FINE, TAUS_FINE, rel, levels=[np.log10(1.1)], colors="w", linewidths=1)
        a.plot(r["loo"][:, 2], r["loo"][:, 1], "r.", ms=5, label="leave one out")
        a.plot(r["L"], r["tau"], "w*", ms=12, label="best")
        a.set_xlabel("L [s]")
        a.set_ylabel("tau [s]")
        a.set_title(f"{ax}: log10 J/Jmin (white: +10 %)", fontsize=9)
        a.legend(fontsize=7, loc="upper right")
    fig.colorbar(cs, ax=axs[0].tolist(), shrink=0.8)
    fig.savefig(folder / "cost_map.png", dpi=95)
    plt.close(fig)


# ------------------------------------------------------------------ validation
def read_model(folder):
    import yaml
    m = yaml.safe_load(open(Path(folder) / "model.yaml"))
    return {ax: (float(v["K"]), float(v["tau"]), float(v["L"])) for ax, v in m.items() if ax in AXES}


def latest_identification(before=None):
    """The latest identification folder (step_*, not stepval_*) with a model.yaml, older than `before` and in
    the same parent folder."""
    root = Path(before).parent if before is not None else SESSIONS
    c = [p for p in root.glob("step_*") if (p / "model.yaml").exists()
         and (before is None or p.name < Path(before).name.replace("stepval", "step"))]
    return sorted(c, key=lambda p: p.name)[-1] if c else None


def validate(ident_folder, val_folder, verbose=True):
    """Simulate the identified model (and the model in use) on another flight: only the offset and drift of each
    doublet are fitted, K, tau, L are not."""
    new = read_model(ident_folder)
    segs, poses, has_sent = load(val_folder)
    out, plots = [], []
    out.append(f"Validation of {Path(ident_folder).name} on {Path(val_folder).name} "
               f"(input: {'sent' if has_sent else 'asked'} commands)")
    out.append(f"{'axis':5s}{'identified model':>24s}{'model in use':>24s}")
    for ax in AXES:
        data = [(s, *measured(s, poses)) for s in segs.values() if s["axis"] == ax]
        data = [d for d in data if d[1] is not None]
        if not data or ax not in new:
            continue
        line = f"{ax:5s}"
        for name, (K, tau, L) in (("identified", new[ax]), ("in use", MODEL_IN_USE[ax])):
            g = [response(s["t"], s["u"], t, tau, L) for s, t, _ in data]
            _, sse, model, meas = solve(data, g, K=K)
            fp = fit_percent(data, model, meas)
            line += f"{fp:14.1f} % {math.sqrt(sse / len(meas)):6.3f} {UNIT[ax]}"
            plots.append((ax, name, model, meas, data))
        out.append(line)
    out.append("fit % on a flight NOT used for the fit (100 = perfect, 0 = no better than a constant).")
    text = "\n".join(out)
    if verbose:
        print("\n" + text)
    (Path(val_folder) / "validation.txt").write_text(text + "\n")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        axes_done = sorted({p[0] for p in plots}, key=AXES.index)
        fig, axs = plt.subplots(len(axes_done), 1, figsize=(11, 2.7 * max(len(axes_done), 1)), squeeze=False)
        for a, ax in zip(axs[:, 0], axes_done):
            for (axn, name, model, meas, data) in plots:
                if axn != ax:
                    continue
                t = np.concatenate([d[1] for d in data]) - data[0][1][0]
                if name == "identified":
                    a.plot(t, meas, ".", ms=2, color="tab:blue", label="ArUco")
                a.plot(t, model, ".", ms=1.5, color="tab:orange" if name == "identified" else "tab:gray", label=name)
            a.set_title(f"validation {ax}")
            a.legend(loc="upper right", fontsize=7)
        fig.tight_layout()
        fig.savefig(Path(val_folder) / "validation.png", dpi=105)
        plt.close(fig)
    except ImportError:
        pass
    return text


def main():
    args = sys.argv[1:]
    if args and args[0] == "--validate":
        if len(args) < 3:
            print("usage: tello_step_fit --validate <identification folder> <validation folder>")
            return
        validate(Path(args[1]).expanduser(), Path(args[2]).expanduser())
        return
    if args:
        folder = Path(args[0]).expanduser()
    else:
        cands = sorted(SESSIONS.glob("step_2*"))  # the latest identification flight (not stepval_*)
        if not cands:
            print("usage: tello_step_fit <session folder>")
            return
        folder = cands[-1]
    print(f"Fitting {folder}")
    fit(folder)
    print(f"Saved {folder / 'model.yaml'}, step_fit.png, cost_map.png")


if __name__ == "__main__":
    main()
