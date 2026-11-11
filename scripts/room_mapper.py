#!/usr/bin/env python3
"""Map the room's ArUco markers with the Tello camera: no tape measure, no reference corner.

    python scripts/room_mapper.py calibrate      # 1. camera calibration with the printed checkerboard
    python scripts/room_mapper.py capture        # 2. film the markers (drone held by hand, motors off)
    python scripts/room_mapper.py solve          # 3. compute config/markers.yaml from the captured frames

How it works: the printed size of every marker is known (15 cm), so each image that shows two markers
gives their relative pose. Chaining these pairs gives a first map; a bundle adjustment then refines all
marker and camera poses together to minimise the reprojection error of every detected corner.
The map frame is attached to an ORIGIN marker: origin on the floor right below it, y axis pointing out
of its wall into the room, z up. The floor height comes from the first frames, taken with the drone
sitting on the floor (camera ~3.5 cm above it).

Start this script first after switching the drone on (the Tello answers only its first client).
"""
import argparse
import glob
import json
import math
import shutil
import sys
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
CALIB = ROOT / "config" / "tello_camera.yaml"
MARKERS = ROOT / "config" / "markers.yaml"
CAPTURES = ROOT / "logs" / "room_captures"
def quat_xyzw(R):
    """Rotation matrix -> quaternion [x, y, z, w]."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s]
    q = np.array(q)
    return (q / np.linalg.norm(q)).tolist()


def map_settings():
    """Marker family and printed size, read from config/markers.yaml (the ones actually on the walls)."""
    try:
        cfg = yaml.safe_load(open(MARKERS, encoding="utf-8")) or {}
    except OSError:
        cfg = {}
    return cfg.get("dictionary", "DICT_6X6_50"), float(cfg.get("marker_size", 0.15))


DICT, MARKER_SIZE = map_settings()
FLOOR_CAM_HEIGHT = 0.035  # camera centre above the floor when the Tello sits on the ground [m]


# ================================================================== common
def aruco_detector():
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICT))
    if hasattr(cv2.aruco, "ArucoDetector"):
        p = cv2.aruco.DetectorParameters()
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        det = cv2.aruco.ArucoDetector(d, p)
        return lambda g: det.detectMarkers(g)[:2]
    p = cv2.aruco.DetectorParameters_create()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return lambda g: cv2.aruco.detectMarkers(g, d, parameters=p)[:2]


def detect(detector, img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    corners, ids = detector(gray)
    if ids is None:
        return {}
    return {int(i): c.reshape(4, 2).astype(np.float64) for c, i in zip(corners, ids.ravel())}


def load_calib(path):
    c = yaml.safe_load(open(path, encoding="utf-8"))
    K = np.array(c["camera_matrix"]["data"], float).reshape(3, 3)
    D = np.array(c["distortion_coefficients"]["data"], float)
    return K, D, (c["image_width"], c["image_height"])


def save_calib(path, K, D, size, rms):
    def mat(rows, cols, data):
        return {"rows": rows, "cols": cols, "data": [float(v) for v in np.ravel(data)]}
    P = np.hstack([K, np.zeros((3, 1))])
    data = {"image_width": int(size[0]), "image_height": int(size[1]), "camera_name": "tello",
            "camera_matrix": mat(3, 3, K), "distortion_model": "plumb_bob",
            "distortion_coefficients": mat(1, len(np.ravel(D)), D),
            "rectification_matrix": mat(3, 3, np.eye(3)), "projection_matrix": mat(3, 4, P),
            "reprojection_error_px": float(rms), "date": datetime.now().isoformat(timespec="seconds")}
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def tello_stream():
    """Generator of BGR frames from the drone (djitellopy), with a clean shutdown."""
    from first_flight import check_tello_network  # fails fast with a clear message if not on the Tello Wi-Fi
    check_tello_network()
    from djitellopy import Tello
    t = Tello()
    try:
        t.connect()
    except Exception as e:  # noqa: BLE001
        sys.exit(f"No answer from the Tello ({e}). Switch it OFF and on, then start this program FIRST.")
    print(f"Connected, battery {t.get_battery()} %")
    t.streamon()
    reader = t.get_frame_read()
    last = None
    try:
        while True:
            f = reader.frame
            # djitellopy starts with a black 400x300 placeholder before the first decoded frame: skip it
            if f is not None and f.shape[:2] == (300, 400) and not f.any():
                time.sleep(0.01)
                continue
            if f is not None and f is not last:
                last = f
                yield cv2.cvtColor(f, cv2.COLOR_RGB2BGR)
            else:
                time.sleep(0.005)
    finally:
        try:
            t.streamoff()
            t.end()
        except Exception:  # noqa: BLE001
            pass


def put(img, txt, y, color=(255, 255, 255), scale=0.6):
    cv2.putText(img, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4)
    cv2.putText(img, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1)


# ================================================================== 1. calibrate
def cmd_calibrate(a):
    pattern = (a.cols, a.rows)
    objp = np.zeros((a.cols * a.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:a.cols, 0:a.rows].T.reshape(-1, 2) * a.square
    obj, img_pts, size, last_t = [], [], None, 0.0
    print("Show the checkerboard to the camera: near, far, tilted, in every corner of the image.\n"
          "Views are taken automatically (1 per second when the board is found). Press C to compute, Esc to quit.")
    for frame in tello_stream():
        size = (frame.shape[1], frame.shape[0])
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK)
        view = frame.copy()
        if found:
            cv2.drawChessboardCorners(view, pattern, corners, found)
            if time.time() - last_t > 1.0:
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                           (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                obj.append(objp)
                img_pts.append(corners)
                last_t = time.time()
        put(view, f"views: {len(obj)} (aim for 25+)   C = compute   Esc = quit", 30, (0, 255, 255))
        cv2.imshow("calibration", view)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            return
        if k in (ord("c"), ord("C")) and len(obj) >= 10:
            break
    rms, K, D, _, _ = cv2.calibrateCamera(obj, img_pts, size, None, None)
    save_calib(CALIB, K, D, size, rms)
    cv2.destroyAllWindows()
    print(f"Calibration from {len(obj)} views: reprojection error {rms:.3f} px (good if < 0.5)\n"
          f"fx {K[0, 0]:.1f}  fy {K[1, 1]:.1f}  cx {K[0, 2]:.1f}  cy {K[1, 2]:.1f}\nSaved to {CALIB}")


# ================================================================== 2. capture
def cmd_capture(a):
    det = aruco_detector()
    out = CAPTURES / datetime.now().strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    seen, pairs = defaultdict(int), set()
    n, last_t, floor_saved = 0, 0.0, 0
    print(f"Saving to {out}\n"
          "1) Put the drone ON THE FLOOR facing the origin marker, press F (floor reference).\n"
          "2) Pick it up and walk slowly along the walls, 1-2 m away, at different heights,\n"
          "   so that every marker is seen together with its neighbours. Frames with 2+ markers are saved\n"
          "   automatically. Esc when every marker is green and linked.")
    for frame in tello_stream():
        dets = detect(det, frame)
        view = frame.copy()
        if dets:
            cv2.aruco.drawDetectedMarkers(view, [c.reshape(1, 4, 2).astype(np.float32) for c in dets.values()],
                                          np.array(list(dets.keys())).reshape(-1, 1))
        now = time.time()
        if len(dets) >= 2 and now - last_t > a.period:
            cv2.imwrite(str(out / f"f_{n:05d}.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            n, last_t = n + 1, now
            for i in dets:
                seen[i] += 1
            ids = sorted(dets)
            pairs.update((p, q) for k, p in enumerate(ids) for q in ids[k + 1:])
        groups = connected_groups(seen.keys(), pairs)
        put(view, f"saved {n} frames   markers seen: {len(seen)}   linked groups: {len(groups)} (want 1)", 30, (0, 255, 255))
        put(view, "seen: " + " ".join(f"{i}x{seen[i]}" for i in sorted(seen)), 55, (180, 255, 180), 0.5)
        put(view, f"F = floor reference ({floor_saved} taken)   Esc = finish", 80, (255, 255, 255), 0.5)
        cv2.imshow("room capture", view)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord("f"), ord("F")) and dets:
            cv2.imwrite(str(out / f"floor_{floor_saved:02d}.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            floor_saved += 1
        if k == 27:
            break
    cv2.destroyAllWindows()
    print(f"{n} frames + {floor_saved} floor frames in {out}. Markers seen: {dict(sorted(seen.items()))}")
    if len(connected_groups(seen.keys(), pairs)) > 1:
        print("WARNING: some markers are never seen together with the others: film the gaps again.")
    print(f"Next: python scripts/room_mapper.py solve --dir \"{out}\"")


def connected_groups(nodes, pairs):
    parent = {n: n for n in nodes}

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    for p, q in pairs:
        if p in parent and q in parent:
            parent[find(p)] = find(q)
    groups = defaultdict(list)
    for n in parent:
        groups[find(n)].append(n)
    return list(groups.values())


# ================================================================== 3. solve
def rt_to_T(rvec, tvec):
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(rvec, float))[0]
    T[:3, 3] = np.ravel(tvec)
    return T


def T_to_rt(T):
    return cv2.Rodrigues(T[:3, :3])[0].ravel(), T[:3, 3].copy()


def inv(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def mean_rotation(Rs):
    U, _, Vt = np.linalg.svd(sum(Rs))
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def marker_local(size):
    h = size / 2
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], float)


def single_pose(corners, size, K, D):
    """Camera <- marker pose from ONE marker (IPPE square, both solutions).

    Returns (T_cam_marker, reprojection error, ambiguity ratio). A small or far marker has two nearly
    equally good poses (planar flip): ratio = err(second) / err(best) close to 1 means "do not trust".
    """
    n, rs, ts, errs = cv2.solvePnPGeneric(marker_local(size), corners, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not n:
        return None, 1e9, 1.0
    errs = np.ravel(errs) if errs is not None and len(np.ravel(errs)) == n else np.array([
        float(np.sqrt(np.mean(np.sum((cv2.projectPoints(marker_local(size), r, tt, K, D)[0].reshape(4, 2) - corners) ** 2, 1))))
        for r, tt in zip(rs, ts)])
    order = np.argsort(errs)
    best = order[0]
    ratio = errs[order[1]] / max(errs[best], 1e-6) if n > 1 else 1e9
    return rt_to_T(rs[best], ts[best]), float(errs[best]), float(ratio)


def robust_average(Ts, max_angle_deg=8.0):
    """Average of poses after dropping rotation outliers (flipped single-marker solutions)."""
    R0 = mean_rotation([T[:3, :3] for T in Ts])
    keep = [T for T in Ts if np.degrees(np.arccos(np.clip((np.trace(R0.T @ T[:3, :3]) - 1) / 2, -1, 1))) < max_angle_deg]
    if len(keep) < max(1, len(Ts) // 3):
        keep = Ts
    T = np.eye(4)
    T[:3, :3] = mean_rotation([k[:3, :3] for k in keep])
    T[:3, 3] = np.median(np.array([k[:3, 3] for k in keep]), axis=0)
    return T, len(keep)


def camera_from_known(dets, M, size, K, D, single):
    """T_origin_cam from all already-mapped markers visible in the frame (joint PnP), or None."""
    known = [i for i in dets if i in M]
    if not known:
        return None
    if len(known) == 1:
        i = known[0]
        if i not in single:
            return None
        return M[i] @ inv(single[i])
    local = marker_local(size)
    obj = np.concatenate([(M[i][:3, :3] @ local.T).T + M[i][:3, 3] for i in known])
    img = np.concatenate([dets[i] for i in known])
    ok, r, tv = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_SQPNP)
    if not ok:
        return None
    r, tv = cv2.solvePnPRefineLM(obj, img, K, D, r, tv)
    err = np.sqrt(np.mean(np.sum((cv2.projectPoints(obj, r, tv, K, D)[0].reshape(-1, 2) - img) ** 2, 1)))
    return inv(rt_to_T(r, tv)) if err < 4.0 else None


def cmd_solve(a):
    from scipy.optimize import least_squares
    from scipy.sparse import lil_matrix

    caps = sorted(p for p in CAPTURES.glob("*") if p.is_dir()) if CAPTURES.exists() else []
    if not a.dir and not caps:
        sys.exit("No capture yet: run 'capture' first (menu choice 7).")
    cap = Path(a.dir) if a.dir else caps[-1]
    if a.sim_K:
        f, cx, cy = a.sim_K
        K, D = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1.0]]), np.zeros(5)
    else:
        if not CALIB.exists():
            sys.exit(f"No calibration file {CALIB}: run 'calibrate' first.")
        K, D, _ = load_calib(CALIB)
    size = a.size
    det = aruco_detector()
    files = sorted(glob.glob(str(cap / "f_*.jpg")))[:: a.stride]
    floor_files = sorted(glob.glob(str(cap / "floor_*.jpg")))
    if a.floor_from_first:
        floor_files = files[: a.floor_from_first]
    frames = []
    for path in floor_files + files:
        dets = detect(det, cv2.imread(path))
        if dets:
            frames.append({"file": Path(path).name, "dets": dets, "floor": path in floor_files})
    print(f"{len(frames)} frames with markers ({sum(f['floor'] for f in frames)} floor frames) from {cap}")

    # ---- trustworthy single-marker poses (unambiguous, low reprojection error)
    counts, covis = defaultdict(int), defaultdict(int)
    for fr in frames:
        fr["single"] = {}
        for i, c in fr["dets"].items():
            counts[i] += 1
            T, e, ratio = single_pose(c, size, K, D)
            if T is not None and e < 1.5 and ratio > a.ambiguity:
                fr["single"][i] = T
        for i in fr["dets"]:
            covis[i] += len(fr["dets"]) - 1
    origin = a.origin if a.origin is not None else max(counts, key=lambda i: (covis[i], counts[i]))
    print(f"origin marker: {origin}")

    # ---- incremental mapping: place the camera with every known marker, then the new markers it sees
    M = {origin: np.eye(4)}  # pose of each marker in the origin-marker frame
    for _ in range(50):
        cand = defaultdict(list)
        for fr in frames:
            T_oc = camera_from_known(fr["dets"], M, size, K, D, fr["single"])
            if T_oc is None:
                continue
            for j, T_cj in fr["single"].items():
                if j not in M:
                    cand[j].append(T_oc @ T_cj)
        added = 0
        for j, Ts in cand.items():
            if len(Ts) >= a.min_pairs:
                M[j], _ = robust_average(Ts)
                added += 1
        if not added:
            break
    missing = sorted(set(counts) - set(M))
    if missing:
        print(f"WARNING: markers {missing} could not be linked to the origin (never seen clearly together "
              f"with already-mapped markers): skipped. Film them again, closer, with a neighbour in view.")
    ids = sorted(M)

    # ---- initial camera poses for the bundle adjustment
    cams = []
    for fr in frames:
        T_oc = camera_from_known(fr["dets"], M, size, K, D, fr["single"])
        if T_oc is not None:
            fr["T"] = T_oc
            cams.append(fr)
    print(f"bundle adjustment: {len(ids)} markers, {len(cams)} camera poses")

    # ---- bundle adjustment (origin marker fixed)
    free = [i for i in ids if i != origin]
    mi = {i: k for k, i in enumerate(free)}
    nm = 6 * len(free)
    x0 = []
    for i in free:
        r, t = T_to_rt(M[i])
        x0 += [*r, *t]
    for fr in cams:
        r, t = T_to_rt(inv(fr["T"]))  # store camera as T_cam_origin
        x0 += [*r, *t]
    x0 = np.array(x0)
    obs = [(ci, i, fr["dets"][i]) for ci, fr in enumerate(cams) for i in fr["dets"] if i in M]
    local = marker_local(size)

    def residuals(x):
        res = []
        for ci, i, c in obs:
            cr, ct = x[nm + 6 * ci: nm + 6 * ci + 3], x[nm + 6 * ci + 3: nm + 6 * ci + 6]
            if i == origin:
                pts = local
            else:
                k = mi[i]
                Rm = cv2.Rodrigues(x[6 * k: 6 * k + 3])[0]
                pts = local @ Rm.T + x[6 * k + 3: 6 * k + 6]
            proj, _ = cv2.projectPoints(pts, cr, ct, K, D)
            res.append((proj.reshape(4, 2) - c).ravel())
        return np.concatenate(res)

    J = lil_matrix((8 * len(obs), len(x0)), dtype=int)
    for k, (ci, i, _) in enumerate(obs):
        J[8 * k: 8 * k + 8, nm + 6 * ci: nm + 6 * ci + 6] = 1
        if i != origin:
            J[8 * k: 8 * k + 8, 6 * mi[i]: 6 * mi[i] + 6] = 1
    r0 = residuals(x0)
    sol = least_squares(residuals, x0, jac_sparsity=J, loss="huber", f_scale=1.0, x_scale="jac", max_nfev=200, verbose=0)
    r1 = sol.fun.reshape(-1, 2)
    print(f"reprojection RMS: {np.sqrt(np.mean(r0 ** 2)) * math.sqrt(2):.2f} px before, "
          f"{np.sqrt(np.mean(np.sum(r1 ** 2, 1))):.2f} px after")
    for i in free:
        k = mi[i]
        M[i] = rt_to_T(sol.x[6 * k: 6 * k + 3], sol.x[6 * k + 3: 6 * k + 6])
    for ci, fr in enumerate(cams):
        fr["T"] = inv(rt_to_T(sol.x[nm + 6 * ci: nm + 6 * ci + 3], sol.x[nm + 6 * ci + 3: nm + 6 * ci + 6]))

    # ---- map frame: gravity from the markers' "up" axes, origin under the origin marker
    up = sum(M[i][:3, 1] for i in ids)
    up /= np.linalg.norm(up)
    n0 = M[origin][:3, 2]
    y = n0 - up * (n0 @ up)
    y /= np.linalg.norm(y)
    x_ax = np.cross(y, up)
    R_map = np.vstack([x_ax, y, up])  # rows: map axes expressed in the origin-marker frame
    p0 = M[origin][:3, 3]
    floor_cams = [fr for fr in cams if fr["floor"]]
    hand_h = [up @ (fr["T"][:3, 3] - p0) for fr in cams if not fr["floor"]]
    if a.origin_height is not None:  # an explicit height wins (floor frames taken off the floor are misleading)
        z_offset = a.origin_height
        floor_note = f"origin marker {origin} set at {a.origin_height} m above the floor (--origin-height)"
    elif floor_cams:
        cam_h = np.median([up @ (fr["T"][:3, 3] - p0) for fr in floor_cams])
        z_offset = -(cam_h - FLOOR_CAM_HEIGHT)  # height of the origin marker above the floor
        floor_note = f"floor from {len(floor_cams)} floor frames"
    else:
        z_offset = 1.0
        floor_note = "NO floor frame and no --origin-height: origin marker assumed at 1.0 m"
    if hand_h:
        floor_note += f"; hand-held camera at {np.median(hand_h) + z_offset:.2f} m median (sanity check)"
    to_map = lambda p: R_map @ (p - p0) + np.array([0, 0, z_offset])  # noqa: E731

    out = []
    for i in ids:
        R = R_map @ M[i][:3, :3]
        n = R[:, 2]
        yaw = math.degrees(math.atan2(n[1], n[0]))
        tilt = math.degrees(math.acos(max(-1, min(1, R[2, 1]))))
        pos = to_map(M[i][:3, 3])
        res_i = np.array([np.linalg.norm(sol.fun[8 * k: 8 * k + 8].reshape(4, 2), axis=1).mean()
                          for k, o in enumerate(obs) if o[1] == i])
        q = quat_xyzw(R)
        out.append({"id": i, "x": pos[0], "y": pos[1], "z": pos[2], "yaw": yaw, "tilt": tilt, "q": q,
                    "views": counts[i], "reproj": float(np.median(res_i))})

    # ---- report + yaml
    print(f"\n{floor_note}")
    print(" id      x       y       z    yaw   tilt  views  reproj")
    for m in out:
        flag = "  <- tilted: re-stick it flat and upright" if m["tilt"] > 4 else ""
        print(f"{m['id']:3d} {m['x']:7.3f} {m['y']:7.3f} {m['z']:7.3f} {m['yaw']:6.1f} {m['tilt']:5.1f} {m['views']:6d} {m['reproj']:6.2f}{flag}")
    if a.dry_run:
        return out
    if MARKERS.exists():
        shutil.copy(MARKERS, MARKERS.with_suffix(f".yaml.bak_{datetime.now():%Y%m%d_%H%M%S}"))
    xs, ys, zs = [m["x"] for m in out], [m["y"] for m in out], [m["z"] for m in out]
    lines = [
        "# Marker map: the SINGLE source of truth for the marker layout (sim + real localizer).",
        f"# Measured by scripts/room_mapper.py on {datetime.now():%Y-%m-%d %H:%M} from {cap.name}",
        f"# ({len(cams)} images, bundle adjustment RMS {np.sqrt(np.mean(np.sum(r1 ** 2, 1))):.2f} px; {floor_note}).",
        f"# Frame map: origin on the floor below marker {origin}, y out of its wall into the room, z up.",
        "# Each marker: centre (m), yaw_deg = direction the marker faces, q = full orientation (quaternion x y z w,",
        "# used by the localizer: a marker stuck slightly rotated or leaning is not an error any more).",
        "",
        f"dictionary: {DICT}",
        f"marker_size: {size}",
        "",
        "room:                      # bounding box of the markers + margin (simulator only)",
        f"  size_x: {max(xs) - min(xs) + 1.0:.2f}",
        f"  size_y: {max(ys) - min(ys) + 1.0:.2f}",
        f"  height: {max(2.5, max(zs) + 0.8):.2f}",
        "",
        "markers:",
    ]
    lines += [f"  - {{id: {m['id']:2d}, x: {m['x']:.3f}, y: {m['y']:.3f}, z: {m['z']:.3f}, yaw_deg: {m['yaw']:6.1f}, "
              f"q: [{m['q'][0]:.4f}, {m['q'][1]:.4f}, {m['q'][2]:.4f}, {m['q'][3]:.4f}]}}"
              f"   # views {m['views']}, tilt {m['tilt']:.1f} deg" for m in out]
    MARKERS.write_text("\n".join(lines) + "\n", encoding="utf-8")
    (cap / "solve_report.json").write_text(json.dumps(out, indent=1), encoding="utf-8")
    print(f"\nWrote {MARKERS} (previous version backed up).")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("calibrate")
    c.add_argument("--cols", type=int, default=9, help="inner corners per row")
    c.add_argument("--rows", type=int, default=6, help="inner corners per column")
    c.add_argument("--square", type=float, default=0.025, help="square size [m]")
    c = sub.add_parser("capture")
    c.add_argument("--period", type=float, default=0.4, help="seconds between saved frames")
    c = sub.add_parser("solve")
    c.add_argument("--dir", help="capture folder (default: latest)")
    c.add_argument("--origin", type=int, help="origin marker id (default: the best connected)")
    c.add_argument("--size", type=float, default=MARKER_SIZE, help="side of the black square [m] (default: marker_size in markers.yaml)")
    c.add_argument("--origin-height", type=float, default=None, help="height of the origin marker centre above the floor [m]; overrides the floor frames")
    c.add_argument("--stride", type=int, default=1)
    c.add_argument("--min-pairs", type=int, default=2, help="views needed to place a new marker")
    c.add_argument("--ambiguity", type=float, default=3.0, help="min error ratio between the two single-marker poses")
    c.add_argument("--dry-run", action="store_true", help="print the map, do not write markers.yaml")
    c.add_argument("--sim-K", type=float, nargs=3, metavar=("F", "CX", "CY"), help="test on simulator images")
    c.add_argument("--floor-from-first", type=int, default=0, help="treat the first N frames as floor frames")
    a = ap.parse_args()
    {"calibrate": cmd_calibrate, "capture": cmd_capture, "solve": cmd_solve}[a.cmd](a)


if __name__ == "__main__":
    main()
