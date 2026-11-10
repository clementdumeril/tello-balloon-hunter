#!/usr/bin/env python3
"""Guided camera calibration for the Tello with the printed checkerboard (docs/print/checkerboard_A4.pdf).

    python scripts/calibrate_camera.py            (or double-click calibrer_camera.bat)

The screen shows, step by step, WHERE to hold the board (blue zone + arrow) and HOW to tilt it.
A photo is taken automatically when the board is in place, sharp and still. At the end the script
computes the calibration, says whether it is good, and saves config/tello_camera.yaml.

Keys: Space = take the photo now · S = skip this step · Esc = quit without saving.
Start it first after switching the drone on (the Tello answers only its first client).
"""
import argparse
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

# where to hold the board: centre (fraction of the image), width of the board (fraction of the image width)
STEPS = [
    dict(title="Au centre, PRÈS", hint="Damier face à la caméra, à ~40 cm, bien au centre", cx=.5, cy=.5, w=(.45, .95)),
    dict(title="Au centre, LOIN", hint="Recule à ~1 m, toujours au centre", cx=.5, cy=.5, w=(.15, .38)),
    dict(title="Coin haut-gauche de l'image", hint="Mets le damier dans la zone bleue", cx=.27, cy=.28, w=(.2, .5)),
    dict(title="Coin haut-droit de l'image", hint="Mets le damier dans la zone bleue", cx=.73, cy=.28, w=(.2, .5)),
    dict(title="Coin bas-gauche de l'image", hint="Mets le damier dans la zone bleue", cx=.27, cy=.72, w=(.2, .5)),
    dict(title="Coin bas-droit de l'image", hint="Mets le damier dans la zone bleue", cx=.73, cy=.72, w=(.2, .5)),
    dict(title="Bord gauche de l'image", hint="Damier collé au bord gauche de l'image", cx=.2, cy=.5, w=(.2, .45)),
    dict(title="Bord droit de l'image", hint="Damier collé au bord droit de l'image", cx=.8, cy=.5, w=(.2, .45)),
    dict(title="Bord haut de l'image", hint="Damier collé au bord haut de l'image", cx=.5, cy=.22, w=(.2, .45)),
    dict(title="Bord bas de l'image", hint="Damier collé au bord bas de l'image", cx=.5, cy=.78, w=(.2, .45)),
    dict(title="Penché sur le côté (1)", hint="Au centre, tourne le damier d'environ 35° autour d'un axe VERTICAL", cx=.5, cy=.5, w=(.2, .7), tilt="h"),
    dict(title="Penché sur le côté (2)", hint="Même chose, mais de l'AUTRE côté", cx=.5, cy=.5, w=(.2, .7), tilt="h2"),
    dict(title="Penché en avant/arrière (1)", hint="Au centre, bascule le haut du damier d'environ 35° (axe HORIZONTAL)", cx=.5, cy=.5, w=(.2, .7), tilt="v"),
    dict(title="Penché en avant/arrière (2)", hint="Même chose, dans l'AUTRE sens", cx=.5, cy=.5, w=(.2, .7), tilt="v2"),
    dict(title="Tourné de 45°", hint="Au centre, tourne le damier dans son plan, comme un losange", cx=.5, cy=.5, w=(.25, .75), roll=True),
]
TOL = 0.13          # position tolerance (fraction of the image)
TILT_MIN = 22.0     # degrees
STILL_PX = 2.0      # corner motion allowed between frames to count as "still"
STILL_TIME = 0.5    # seconds still before the photo
SHARP_MIN = 40.0    # Laplacian variance on the board (below = blurry)

FONT = "C:/Windows/Fonts/segoeui.ttf"
FONT_B = "C:/Windows/Fonts/segoeuib.ttf"


def fonts():
    try:
        return ImageFont.truetype(FONT_B, 26), ImageFont.truetype(FONT, 19), ImageFont.truetype(FONT_B, 21)
    except OSError:
        d = ImageFont.load_default()
        return d, d, d


F_BIG, F_TXT, F_STATUS = fonts()


def board_points():
    objp = np.zeros((COLS * ROWS, 3), np.float32)
    objp[:, :2] = np.mgrid[0:COLS, 0:ROWS].T.reshape(-1, 2) * SQUARE
    return objp


def measure(corners, size, K0):
    """Position, size, tilts (deg) and in-plane rotation of the board in the image."""
    W, H = size
    pts = corners.reshape(-1, 2)
    cx, cy = pts[:, 0].mean() / W, pts[:, 1].mean() / H
    width = (pts[:, 0].max() - pts[:, 0].min()) / W
    ok, r, t = cv2.solvePnP(board_points(), pts, K0, None)
    R = cv2.Rodrigues(r)[0] if ok else np.eye(3)
    n = R[:, 2]  # board normal in the camera frame
    tilt_h = math.degrees(math.atan2(n[0], abs(n[2])))
    tilt_v = math.degrees(math.atan2(n[1], abs(n[2])))
    row = pts[COLS - 1] - pts[0]
    roll = math.degrees(math.atan2(row[1], row[0]))
    roll = ((roll + 90) % 180) - 90  # a board turned by 180 deg looks the same
    return dict(cx=cx, cy=cy, w=width, th=tilt_h, tv=tilt_v, roll=roll)


def check(step, m, done):
    """(ok, advice) for the current measurement against the step's target."""
    adv = []
    if abs(m["cx"] - step["cx"]) > TOL or abs(m["cy"] - step["cy"]) > TOL:
        adv.append("déplace le damier vers la zone bleue (suis la flèche)")
    lo, hi = step["w"]
    if m["w"] < lo:
        adv.append("rapproche-le de la caméra")
    elif m["w"] > hi:
        adv.append("éloigne-le de la caméra")
    tilt = step.get("tilt")
    if tilt in ("h", "h2"):
        sign_ok = True
        if tilt == "h2" and "h" in done:
            sign_ok = np.sign(m["th"]) != np.sign(done["h"])
        if abs(m["th"]) < TILT_MIN:
            adv.append(f"tourne-le davantage autour de l'axe vertical ({abs(m['th']):.0f}°, il faut {TILT_MIN:.0f}°+)")
        elif not sign_ok:
            adv.append("tourne-le de l'AUTRE côté que la photo précédente")
    if tilt in ("v", "v2"):
        sign_ok = True
        if tilt == "v2" and "v" in done:
            sign_ok = np.sign(m["tv"]) != np.sign(done["v"])
        if abs(m["tv"]) < TILT_MIN:
            adv.append(f"bascule-le davantage autour de l'axe horizontal ({abs(m['tv']):.0f}°, il faut {TILT_MIN:.0f}°+)")
        elif not sign_ok:
            adv.append("bascule-le dans l'AUTRE sens que la photo précédente")
    if step.get("roll") and not 25 <= abs(m["roll"]) <= 65:
        adv.append(f"tourne-le dans son plan, vers 45° (là : {abs(m['roll']):.0f}°)")
    if not step.get("tilt") and not step.get("roll") and max(abs(m["th"]), abs(m["tv"])) > 25:
        adv.append("tiens-le bien face à la caméra (pas penché)")
    return not adv, adv


def draw_ui(frame, step_i, steps, status, color, advice, zone, arrow, corners, found, n_ok, flash):
    H, W = frame.shape[:2]
    view = frame.copy()
    st = steps[step_i] if step_i < len(steps) else None
    if zone is not None:
        overlay = view.copy()
        cv2.rectangle(overlay, zone[0], zone[1], (255, 170, 60), -1)
        view = cv2.addWeighted(overlay, 0.18, view, 0.82, 0)
        cv2.rectangle(view, zone[0], zone[1], (255, 170, 60), 2)
    if found:
        cv2.drawChessboardCorners(view, (COLS, ROWS), corners, True)
    if arrow is not None:
        cv2.arrowedLine(view, arrow[0], arrow[1], (255, 170, 60), 4, cv2.LINE_AA, tipLength=0.15)
    if flash:
        view = cv2.addWeighted(view, 0.6, np.full_like(view, 255), 0.4, 0)
    img = Image.fromarray(cv2.cvtColor(view, cv2.COLOR_BGR2RGB))
    d = ImageDraw.Draw(img, "RGBA")
    d.rectangle((0, 0, W, 92), fill=(15, 20, 25, 210))
    if st:
        d.text((14, 8), f"Étape {step_i + 1}/{len(steps)} : {st['title']}", font=F_BIG, fill=(240, 245, 245))
        d.text((14, 44), st["hint"], font=F_TXT, fill=(200, 215, 220))
    d.text((W - 230, 12), f"Photos : {n_ok}/{len(steps)}", font=F_TXT, fill=(150, 220, 160))
    d.rectangle((0, H - 66, W, H), fill=(15, 20, 25, 210))
    d.text((14, H - 62), status, font=F_STATUS, fill=color)
    if advice:
        d.text((14, H - 32), " · ".join(advice)[:110], font=F_TXT, fill=(255, 205, 120))
    d.text((W - 410, H - 30), "Espace = photo · S = passer · Échap = quitter", font=F_TXT, fill=(160, 170, 175))
    return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)


def verdict(rms):
    if rms < 0.35:
        return "EXCELLENTE", (120, 230, 140)
    if rms < 0.6:
        return "BONNE", (120, 230, 140)
    if rms < 1.0:
        return "MOYENNE : à refaire si possible", (255, 205, 120)
    return "MAUVAISE : recommence (damier pas plat ? flou ? mal imprimé ?)", (255, 120, 120)


def run(frames_iter, save=True, show=True):
    obj, img_pts, done, still_since, last_pts = [], [], {}, None, None
    step_i, size, flash_until, K0 = 0, None, 0.0, None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    for frame in frames_iter:
        if size is None:
            size = (frame.shape[1], frame.shape[0])
            f0 = math.hypot(*size) / 2 / math.tan(math.radians(82.6) / 2)
            K0 = np.array([[f0, 0, size[0] / 2], [0, f0, size[1] / 2], [0, 0, 1]])
        if step_i >= len(STEPS):
            break
        st = STEPS[step_i]
        W, H = size
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (COLS, ROWS),
                                                   cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
        zw = (st["w"][0] + st["w"][1]) / 2
        zone = ((int((st["cx"] - zw / 2) * W), int((st["cy"] - zw / 2 * W / H * 0.67) * H)),
                (int((st["cx"] + zw / 2) * W), int((st["cy"] + zw / 2 * W / H * 0.67) * H)))
        arrow, advice, take = None, [], False
        if not found:
            status, color = "Damier non détecté : montre-le en entier à la caméra", (255, 150, 120)
            still_since, last_pts = None, None
        else:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
            m = measure(corners, size, K0)
            ok, advice = check(st, m, done)
            pts = corners.reshape(-1, 2)
            if abs(m["cx"] - st["cx"]) > TOL or abs(m["cy"] - st["cy"]) > TOL:
                arrow = ((int(m["cx"] * W), int(m["cy"] * H)), (int(st["cx"] * W), int(st["cy"] * H)))
            x0, y0 = pts.min(0).astype(int)
            x1, y1 = pts.max(0).astype(int)
            sharp = cv2.Laplacian(gray[max(0, y0):y1, max(0, x0):x1], cv2.CV_64F).var() if x1 > x0 and y1 > y0 else 0
            moving = last_pts is not None and np.abs(pts - last_pts).mean() > STILL_PX
            last_pts = pts
            if ok and sharp < SHARP_MIN:
                ok, advice = False, ["image floue : bouge moins vite, éclaire mieux la pièce"]
            if ok and not moving:
                still_since = still_since or time.time()
                left = STILL_TIME - (time.time() - still_since)
                status, color = (f"Parfait, ne bouge plus… {max(left, 0):.1f} s", (120, 230, 140))
                take = left <= 0
            else:
                still_since = None
                status, color = ("Bien placé, mais tiens-le immobile", (255, 205, 120)) if ok else ("Pas encore", (255, 205, 120))
        if show:
            view = draw_ui(frame, step_i, STEPS, status, color, advice, zone, arrow, corners, found, len(obj), time.time() < flash_until)
            cv2.imshow("Calibration de la camera", view)
            k = cv2.waitKey(1) & 0xFF
        else:
            k = 255
        if k == 27:
            print("Abandon : rien n'est enregistré.")
            return None
        if k in (ord("s"), ord("S")):
            step_i, still_since = step_i + 1, None
            continue
        if found and (take or k == 32):
            obj.append(board_points())
            img_pts.append(corners)
            if st.get("tilt"):
                done[st["tilt"]] = m["th"] if st["tilt"].startswith("h") else m["tv"]
            step_i, still_since, flash_until = step_i + 1, None, time.time() + 0.25
            print(f"photo {len(obj)} : {st['title']}")
    if len(obj) < 8:
        print(f"Seulement {len(obj)} photos : il en faut au moins 8. Relance le script.")
        return None
    # calibration, then drop the worst views if they are clear outliers
    rms, K, D, rv, tv = cv2.calibrateCamera(obj, img_pts, size, None, None)
    errs = [float(np.sqrt(np.mean(np.sum((cv2.projectPoints(o, r, t, K, D)[0].reshape(-1, 2) - p.reshape(-1, 2)) ** 2, 1))))
            for o, r, t, p in zip(obj, rv, tv, img_pts)]
    keep = [i for i, e in enumerate(errs) if e < 2.5 * np.median(errs)]
    if len(keep) < len(obj) and len(keep) >= 8:
        rms, K, D, _, _ = cv2.calibrateCamera([obj[i] for i in keep], [img_pts[i] for i in keep], size, None, None)
    word, color = verdict(rms)
    f_expected = math.hypot(*size) / 2 / math.tan(math.radians(82.6) / 2)
    f_note = "" if abs(K[0, 0] / f_expected - 1) < 0.2 else f"  ATTENTION : focale {K[0, 0]:.0f} px loin des ~{f_expected:.0f} attendus (damier mal imprimé ?)"
    msg = (f"Calibration {word} : erreur {rms:.2f} px ({len(keep)} photos utilisées sur {len(obj)})\n"
           f"fx {K[0, 0]:.1f}  fy {K[1, 1]:.1f}  cx {K[0, 2]:.1f}  cy {K[1, 2]:.1f}  distorsion {np.round(np.ravel(D)[:5], 3).tolist()}{f_note}")
    print(msg)
    if save:
        save_calib(CALIB, K, D, size, rms)
        print(f"Enregistrée dans {CALIB}")
    if show:
        end = draw_ui(cv2.undistort(frame, K, D), len(STEPS), STEPS, f"Calibration {word} : erreur {rms:.2f} px", color,
                      ["image corrigée de la distorsion affichée : les lignes droites doivent être droites",
                       "appuie sur une touche pour fermer"], None, None, None, False, len(obj), False)
        cv2.imshow("Calibration de la camera", end)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return rms, K, D


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.parse_args()
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:  # noqa: BLE001
        pass
    print(__doc__)
    run(tello_stream())


if __name__ == "__main__":
    main()
