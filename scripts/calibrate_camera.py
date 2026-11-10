#!/usr/bin/env python3
"""Camera calibration for the Tello with the printed checkerboard (docs/print/checkerboard_A4.pdf).

    python scripts/calibrate_camera.py            (or double-click calibrer_camera.bat)

No target to aim at: move the board freely in front of the camera. The image is split into 9 cells
that turn green as the board passes over them, and a checklist shows what is still missing (near, far,
tilted left/right/up/down, rotated). A photo is taken automatically whenever the board is sharp, still
for a moment and brings something new. When everything is green, the calibration is computed, rated,
and saved to config/tello_camera.yaml.

Keys: C = compute now (10+ photos) · Esc = quit without saving.
Start it first after switching the drone on (the Tello answers only its first client).
"""
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parent))
from room_mapper import CALIB, save_calib, tello_stream  # noqa: E402

COLS, ROWS, SQUARE = 9, 6, 0.025  # inner corners, square size [m] (docs/print/checkerboard_A4.pdf)
GRID = 3                          # image split in GRID x GRID cells
TILT = 20.0                       # degrees for a "tilted" view
STILL_PX = 3.0                    # mean corner motion between frames still counted as "still"
STILL_TIME = 0.3                  # seconds still before the photo
SHARP_MIN = 30.0                  # Laplacian variance on the board (below = blurry)
MIN_VIEWS, MAX_VIEWS = 15, 40

CHECKS = [  # key, label, test on the measurement
    ("near", "Près (damier grand dans l'image)", lambda m: m["w"] > 0.45),
    ("far", "Loin (damier petit, ~1 m)", lambda m: m["w"] < 0.28),
    ("left", "Penché : tourné vers un côté", lambda m: m["th"] < -TILT),
    ("right", "Penché : tourné vers l'autre côté", lambda m: m["th"] > TILT),
    ("up", "Basculé : haut vers l'arrière", lambda m: m["tv"] < -TILT),
    ("down", "Basculé : haut vers l'avant", lambda m: m["tv"] > TILT),
    ("roll", "Tourné en losange (~45°)", lambda m: 25 <= abs(m["roll"]) <= 65),
]

try:
    F_BIG = ImageFont.truetype("C:/Windows/Fonts/segoeuib.ttf", 24)
    F_TXT = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", 17)
    F_ST = ImageFont.truetype("C:/Windows/Fonts/segoeuib.ttf", 20)
except OSError:
    F_BIG = F_TXT = F_ST = ImageFont.load_default()


def board_points():
    objp = np.zeros((COLS * ROWS, 3), np.float32)
    objp[:, :2] = np.mgrid[0:COLS, 0:ROWS].T.reshape(-1, 2) * SQUARE
    return objp


def measure(pts, size, K0):
    """Size, tilts (deg), in-plane rotation and covered cells of a detected board."""
    W, H = size
    width = (pts[:, 0].max() - pts[:, 0].min()) / W
    ok, r, _ = cv2.solvePnP(board_points(), pts, K0, None)
    n = cv2.Rodrigues(r)[0][:, 2] if ok else np.array([0, 0, 1.0])
    row = pts[COLS - 1] - pts[0]
    roll = ((math.degrees(math.atan2(row[1], row[0])) + 90) % 180) - 90
    cells = set()
    for x, y in pts:
        cells.add((min(GRID - 1, int(y / H * GRID)), min(GRID - 1, int(x / W * GRID))))
    return dict(w=width, th=math.degrees(math.atan2(n[0], abs(n[2]))), tv=math.degrees(math.atan2(n[1], abs(n[2]))),
                roll=roll, cells=cells, c=pts.mean(0) / np.array([W, H]))


def novelty(m, cov_cells, cov_checks, views):
    """What this view would add (new cells / new checks), and whether it differs from every photo taken."""
    new_cells = m["cells"] - cov_cells
    new_checks = {k for k, _, f in CHECKS if f(m) and k not in cov_checks}
    different = all(np.linalg.norm(m["c"] - v["c"]) > 0.12 or abs(m["w"] - v["w"]) > 0.1
                    or abs(m["th"] - v["th"]) > 12 or abs(m["tv"] - v["tv"]) > 12 for v in views)
    return new_cells, new_checks, different


def draw(frame, cov_cells, cov_checks, n_views, status, color, corners, flash, hint=""):
    H, W = frame.shape[:2]
    view = frame.copy()
    overlay = view.copy()
    for r in range(GRID):
        for c in range(GRID):
            if (r, c) in cov_cells:
                cv2.rectangle(overlay, (c * W // GRID, r * H // GRID), ((c + 1) * W // GRID, (r + 1) * H // GRID), (80, 200, 90), -1)
    view = cv2.addWeighted(overlay, 0.22, view, 0.78, 0)
    for k in range(1, GRID):
        cv2.line(view, (k * W // GRID, 0), (k * W // GRID, H), (220, 220, 220), 1)
        cv2.line(view, (0, k * H // GRID), (W, k * H // GRID), (220, 220, 220), 1)
    if corners is not None:
        cv2.drawChessboardCorners(view, (COLS, ROWS), corners, True)
    if flash:
        view = cv2.addWeighted(view, 0.55, np.full_like(view, 255), 0.45, 0)
    img = Image.fromarray(cv2.cvtColor(view, cv2.COLOR_BGR2RGB))
    d = ImageDraw.Draw(img, "RGBA")
    d.rectangle((0, 0, W, 40), fill=(15, 20, 25, 215))
    n_cells = len(cov_cells)
    d.text((12, 6), f"Calibration · photos {n_views} · cases vertes {n_cells}/{GRID * GRID}", font=F_BIG, fill=(240, 245, 245))
    # checklist
    x0, y0 = W - 330, 52
    d.rectangle((x0 - 10, y0 - 8, W - 8, y0 + 26 * len(CHECKS) + 4), fill=(15, 20, 25, 190))
    for i, (k, label, _) in enumerate(CHECKS):
        done = k in cov_checks
        d.text((x0, y0 + 26 * i), ("● " if done else "○ ") + label, font=F_TXT, fill=(130, 230, 140) if done else (235, 235, 235))
    d.rectangle((0, H - 62, W, H), fill=(15, 20, 25, 215))
    d.text((12, H - 58), status, font=F_ST, fill=color)
    d.text((12, H - 28), hint or "Bouge le damier partout dans l'image, puis penche-le · C = calculer · Échap = quitter",
           font=F_TXT, fill=(255, 205, 120) if hint else (170, 180, 185))
    return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)


def verdict(rms):
    if rms < 0.35:
        return "EXCELLENTE", (120, 230, 140)
    if rms < 0.6:
        return "BONNE", (120, 230, 140)
    if rms < 1.0:
        return "MOYENNE : à refaire si possible", (255, 205, 120)
    return "MAUVAISE : recommence (damier pas plat ? flou ?)", (255, 120, 120)


def missing_hint(cov_cells, cov_checks):
    names = {(0, 0): "en haut à gauche", (0, 1): "en haut", (0, 2): "en haut à droite", (1, 0): "à gauche",
             (1, 1): "au centre", (1, 2): "à droite", (2, 0): "en bas à gauche", (2, 1): "en bas", (2, 2): "en bas à droite"}
    cells = [names[c] for c in sorted(set(names) - cov_cells)]
    checks = [label.split(" (")[0].lower() for k, label, _ in CHECKS if k not in cov_checks]
    parts = []
    if cells:
        parts.append("il manque l'image " + ", ".join(cells[:3]) + ("…" if len(cells) > 3 else ""))
    if checks:
        parts.append("à faire : " + checks[0])
    return " · ".join(parts)


def run(frames_iter, save=True, show=True):
    obj, img_pts, views = [], [], []
    cov_cells, cov_checks = set(), set()
    size = K0 = None
    still_since, last_pts, flash_until, frame = None, None, 0.0, None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    for frame in frames_iter:
        if size is None:
            size = (frame.shape[1], frame.shape[0])
            f0 = math.hypot(*size) / 2 / math.tan(math.radians(82.6) / 2)
            K0 = np.array([[f0, 0, size[0] / 2], [0, f0, size[1] / 2], [0, 0, 1]])
        complete = len(cov_cells) == GRID * GRID and len(cov_checks) == len(CHECKS) and len(obj) >= MIN_VIEWS
        if complete or len(obj) >= MAX_VIEWS:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (COLS, ROWS),
                                                   cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
        take = False
        if not found:
            status, color = "Damier non détecté : montre-le EN ENTIER à la caméra", (255, 150, 120)
            still_since, last_pts, corners = None, None, None
        else:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
            pts = corners.reshape(-1, 2)
            m = measure(pts, size, K0)
            x0, y0 = pts.min(0).astype(int)
            x1, y1 = pts.max(0).astype(int)
            sharp = cv2.Laplacian(gray[max(0, y0):y1, max(0, x0):x1], cv2.CV_64F).var() if x1 > x0 and y1 > y0 else 0
            moving = last_pts is not None and np.abs(pts - last_pts).mean() > STILL_PX
            last_pts = pts
            new_cells, new_checks, different = novelty(m, cov_cells, cov_checks, views)
            useful = bool(new_cells or new_checks) or (different and len(obj) < MIN_VIEWS)
            if sharp < SHARP_MIN:
                status, color, still_since = "Image floue : bouge plus doucement, éclaire la pièce", (255, 150, 120), None
            elif not useful:
                status, color, still_since = "Déjà en photo : change de position", (255, 205, 120), None
            elif moving:
                status, color, still_since = "Nouveau : tiens-le immobile un instant…", (255, 205, 120), None
            else:
                still_since = still_since or time.time()
                status, color = "Ne bouge plus…", (120, 230, 140)
                take = time.time() - still_since >= STILL_TIME
        k = 255
        if show:
            cv2.imshow("Calibration de la camera", draw(frame, cov_cells, cov_checks, len(obj), status, color,
                                                        corners if found else None, time.time() < flash_until,
                                                        missing_hint(cov_cells, cov_checks)))
            k = cv2.waitKey(1) & 0xFF
        if k == 27:
            print("Abandon : rien n'est enregistré.")
            return None
        if k in (ord("c"), ord("C")) and len(obj) >= 10:
            break
        if take:
            obj.append(board_points())
            img_pts.append(corners)
            views.append(m)
            cov_cells |= m["cells"]
            cov_checks |= {key for key, _, f in CHECKS if f(m)}
            still_since, flash_until = None, time.time() + 0.25
            print(f"photo {len(obj)} : cases {len(cov_cells)}/{GRID * GRID}, coché {sorted(cov_checks)}")
    if len(obj) < 10:
        print(f"Seulement {len(obj)} photos : il en faut au moins 10. Relance le script.")
        return None
    rms, K, D, rv, tv = cv2.calibrateCamera(obj, img_pts, size, None, None)
    errs = [float(np.sqrt(np.mean(np.sum((cv2.projectPoints(o, r, t, K, D)[0].reshape(-1, 2) - p.reshape(-1, 2)) ** 2, 1))))
            for o, r, t, p in zip(obj, rv, tv, img_pts)]
    keep = [i for i, e in enumerate(errs) if e < 2.5 * np.median(errs)]
    if len(keep) < len(obj) and len(keep) >= 10:
        rms, K, D, _, _ = cv2.calibrateCamera([obj[i] for i in keep], [img_pts[i] for i in keep], size, None, None)
    word, color = verdict(rms)
    f_exp = math.hypot(*size) / 2 / math.tan(math.radians(82.6) / 2)
    f_note = "" if abs(K[0, 0] / f_exp - 1) < 0.2 else f"  ATTENTION : focale {K[0, 0]:.0f} px loin des ~{f_exp:.0f} attendus (damier mal imprimé ?)"
    missing = [label for key, label, _ in CHECKS if key not in cov_checks]
    print(f"Calibration {word} : erreur {rms:.2f} px ({len(keep)} photos utilisées sur {len(obj)})\n"
          f"fx {K[0, 0]:.1f}  fy {K[1, 1]:.1f}  cx {K[0, 2]:.1f}  cy {K[1, 2]:.1f}  distorsion {np.round(np.ravel(D)[:5], 3).tolist()}{f_note}"
          + (f"\nNon couvert : {', '.join(missing)}" if missing else ""))
    if save:
        save_calib(CALIB, K, D, size, rms)
        print(f"Enregistrée dans {CALIB}")
    if show and frame is not None:
        end = draw(cv2.undistort(frame, K, D), cov_cells, cov_checks, len(obj), f"Calibration {word} : erreur {rms:.2f} px",
                   color, None, False, "Image corrigée : les lignes droites doivent être droites · une touche pour fermer")
        cv2.imshow("Calibration de la camera", end)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return rms, K, D


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:  # noqa: BLE001
        pass
    print(__doc__)
    run(tello_stream())


if __name__ == "__main__":
    main()
