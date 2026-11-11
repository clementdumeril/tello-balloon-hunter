#!/usr/bin/env python3
"""Align a phone room scan (Scaniverse OBJ + texture) on the marker map, automatically.

    python tools/align_scan.py <scan.obj> [--markers config/markers.yaml] [--out <scan folder>/alignment.npz]

1. The ArUco markers are detected directly in the scan's TEXTURE image (they are printed on the walls, so they
   appear in the texture atlas).
2. Each detected corner (a pixel of the texture) is turned back into a 3D point of the scan: the mesh triangle
   whose texture coordinates contain it, barycentric interpolation of its 3 vertices.
3. The same corners are known in the map (config/markers.yaml). A similarity (scale s, rotation R, translation t:
   p_map = s R p_scan + t) is fitted with RANSAC over the markers (Umeyama), then on all the inliers.
Prints the residual of every marker and a leave-one-out check; writes alignment.npz (s, R, t, ids), used by
tools/room_mesh_to_map.py.
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src" / "aruco_localization"))
from aruco_localization.marker_map import load_marker_map  # noqa: E402


def load_obj(path):
    V, VT, F = [], [], []
    for line in open(path, encoding="utf-8", errors="ignore"):
        if line.startswith("v "):
            V.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("vt "):
            VT.append([float(x) for x in line.split()[1:3]])
        elif line.startswith("f "):
            idx = [c.split("/") for c in line.split()[1:4]]
            F.append([(int(a[0]) - 1, int(a[1]) - 1) for a in idx])
    F = np.array(F)
    return np.array(V), np.array(VT), F[:, :, 0], F[:, :, 1]


def texture_of(obj):
    mtl = Path(obj).with_suffix(".mtl")
    for line in open(mtl, encoding="utf-8", errors="ignore"):
        if line.strip().startswith("map_Kd"):
            return Path(obj).parent / line.split(None, 1)[1].strip()
    raise SystemExit("no texture in the .mtl")


def uv_to_3d(uv, V, VT, FV, FT):
    """3D point of the scan at texture coordinate uv (None if no triangle contains it)."""
    a, b, c = VT[FT[:, 0]], VT[FT[:, 1]], VT[FT[:, 2]]
    v0, v1, v2 = b - a, c - a, uv - a
    d00, d01, d11 = (v0 * v0).sum(1), (v0 * v1).sum(1), (v1 * v1).sum(1)
    d20, d21 = (v2 * v0).sum(1), (v2 * v1).sum(1)
    den = d00 * d11 - d01 * d01
    ok = np.abs(den) > 1e-18
    w1 = np.where(ok, (d11 * d20 - d01 * d21) / np.where(ok, den, 1), -1)
    w2 = np.where(ok, (d00 * d21 - d01 * d20) / np.where(ok, den, 1), -1)
    w0 = 1 - w1 - w2
    inside = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6)
    k = np.flatnonzero(inside)
    if not len(k):
        return None
    k = k[0]
    return w0[k] * V[FV[k, 0]] + w1[k] * V[FV[k, 1]] + w2[k] * V[FV[k, 2]]


def umeyama(src, dst):
    """Similarity dst ~ s R src + t (least squares)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    U, S, Vt = np.linalg.svd(xd.T @ xs / len(src))
    D = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        D[2, 2] = -1
    R = U @ D @ Vt
    s = np.trace(np.diag(S) @ D) / (xs ** 2).sum(1).mean()
    return s, R, mu_d - s * R @ mu_s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("obj")
    ap.add_argument("--markers", default=str(ROOT / "config" / "markers.yaml"))
    ap.add_argument("--out", default="")
    ap.add_argument("--inlier", type=float, default=0.06, help="RANSAC inlier distance (m)")
    a = ap.parse_args()
    cfg, markers = load_marker_map(a.markers)
    V, VT, FV, FT = load_obj(a.obj)
    tex = cv2.imread(str(texture_of(a.obj)), cv2.IMREAD_GRAYSCALE)
    H, W = tex.shape
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg.get("dictionary", "DICT_6X6_50")))
    prm = cv2.aruco.DetectorParameters()
    prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    det = cv2.aruco.ArucoDetector(d, prm)
    corners, ids, _ = det.detectMarkers(tex)
    if ids is None:
        raise SystemExit("no marker found in the texture")
    found = {}
    for c, i in zip(corners, ids.ravel()):
        i = int(i)
        if i not in markers:
            continue
        pts = []
        for (px, py) in c.reshape(4, 2):
            p = uv_to_3d(np.array([px / W, 1.0 - py / H]), V, VT, FV, FT)
            if p is None:
                break
            pts.append(p)
        if len(pts) == 4:
            side = np.mean(np.linalg.norm(np.array(pts) - np.roll(np.array(pts), 1, 0), axis=1))
            found.setdefault(i, []).append((np.array(pts), side))
    print(f"markers detected in the texture: {sorted(int(i) for i in ids.ravel())}; usable (in the map, 3D ok): {sorted(found)}")
    if len(found) < 3:
        raise SystemExit("need at least 3 markers found in the scan texture and present in the map")
    # one 3D instance per marker (the texture can hold the same wall twice: take the median corners)
    scan_c = {i: np.median(np.array([p for p, _ in v]), axis=0) for i, v in found.items()}
    sides = {i: np.median([s for _, s in v]) for i, v in found.items()}
    ids_ok = sorted(scan_c)
    best = None
    rng = np.random.default_rng(0)
    for _ in range(300):
        pick = rng.choice(ids_ok, 3, replace=False)
        src = np.concatenate([scan_c[i] for i in pick])
        dst = np.concatenate([markers[i].corners_map() for i in pick])
        s, R, t = umeyama(src, dst)
        err = {i: np.linalg.norm((s * (R @ scan_c[i].T)).T + t - markers[i].corners_map(), axis=1).mean() for i in ids_ok}
        inl = [i for i in ids_ok if err[i] < a.inlier]
        if best is None or len(inl) > len(best[0]) or (len(inl) == len(best[0]) and sum(err[i] for i in inl) < best[1]):
            best = (inl, sum(err[i] for i in inl))
    inl = best[0]
    src = np.concatenate([scan_c[i] for i in inl])
    dst = np.concatenate([markers[i].corners_map() for i in inl])
    s, R, t = umeyama(src, dst)
    print(f"\nscale s = {s:.4f} (1 = scan and map agree on distances), inliers {len(inl)}/{len(ids_ok)}")
    print(" id   residual   leave-one-out   side in scan (scaled)")
    loo_all = []
    for i in ids_ok:
        res = np.linalg.norm((s * (R @ scan_c[i].T)).T + t - markers[i].corners_map(), axis=1).mean()
        others = [j for j in inl if j != i]
        if len(others) >= 3:
            s2, R2, t2 = umeyama(np.concatenate([scan_c[j] for j in others]),
                                 np.concatenate([markers[j].corners_map() for j in others]))
            loo = np.linalg.norm((s2 * (R2 @ scan_c[i].T)).T + t2 - markers[i].corners_map(), axis=1).mean()
            loo_all.append(loo)
        else:
            loo = float("nan")
        flag = "" if i in inl else "   <- outlier (moved since the scan, or wrong in the map?)"
        print(f"{i:3d} {res * 100:8.1f} cm {loo * 100:12.1f} cm {sides[i] * s * 100:12.1f} cm{flag}")
    print(f"\nmean residual {np.mean([np.linalg.norm((s * (R @ scan_c[i].T)).T + t - markers[i].corners_map(), axis=1).mean() for i in inl]) * 100:.1f} cm, "
          f"leave-one-out median {np.median(loo_all) * 100 if loo_all else float('nan'):.1f} cm")
    out = Path(a.out) if a.out else Path(a.obj).parent / "alignment.npz"
    np.savez(out, s=s, R=R, t=t, ids=np.array(inl))
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
