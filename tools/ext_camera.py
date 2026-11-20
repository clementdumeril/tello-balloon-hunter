#!/usr/bin/env python3
"""External (fixed) camera: find where it is in the room, then see the balloons in the `map` frame.

Runs on Windows (the USB camera is a Windows device; WSL has no /dev/video):
    python tools/ext_camera.py list                       which camera index is which
    python tools/ext_camera.py locate --cam 1             intrinsics AND pose in `map` from the wall markers
    python tools/ext_camera.py detect --cam 1             balloons in `map` -> UDP -> WSL (tello_balloons node)
    python tools/ext_camera.py detect --cam 1 --record    same + raw video in logs/balloons/ (offline tuning)
    python tools/ext_camera.py detect --source logs/balloons/rec_<time>.mp4    replay a recording (or an image)

locate: no checkerboard and no tape measure. The camera sees markers on several walls whose corners are known in
`map` (config/markers.yaml). Those 3D<->pixel pairs are NOT on one plane, so a single (averaged) view is enough
to solve the focal length, the image centre, the radial distortion AND the camera pose in one least-squares
problem (cv2.calibrateCamera with an initial guess). Markers that disagree (moved since the mapping) are
dropped one by one. Result: config/ext_camera.yaml.

detect: a balloon is a ray from the camera through its image centre; its distance comes from its apparent size
(d = f D / width_px, D = balloon diameter, measured once). Detection: YOLO-World (open vocabulary, no training)
or plain colour segmentation; both are compared by `detect --compare`.
"""
import argparse
import json
import math
import socket
import struct
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src" / "aruco_localization"))
from aruco_localization.marker_map import load_marker_map  # noqa: E402

CALIB_OUT = REPO / "config" / "ext_camera.yaml"
UDP = ("127.0.0.1", 9870)   # WSL mirrored networking: Windows localhost is WSL localhost
UDP_IMG = ("127.0.0.1", 9871)  # the annotated image (JPEG) for RViz


def open_cam(idx, w=1280, h=720):
    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    if not cap.isOpened():
        raise SystemExit(f"camera {idx} does not open")
    for _ in range(10):  # auto exposure settles
        cap.read()
    return cap


def cmd_list(_a):
    for i in range(6):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        ok, img = cap.read() if cap.isOpened() else (False, None)
        print(f"camera {i}: " + (f"{img.shape[1]}x{img.shape[0]}" if ok else "-"))
        if ok:
            cv2.imwrite(str(REPO / "logs" / f"cam{i}.jpg"), img)
        cap.release()
    print("snapshots in logs/cam<i>.jpg")


# ---------------------------------------------------------------- locate
def detect_markers(cap, frames, dict_name):
    """Average each marker's corners over `frames` images (fixed camera: less pixel noise)."""
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name)),
                                  cv2.aruco.DetectorParameters())
    acc, img = {}, None
    for _ in range(frames):
        ok, img = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = det.detectMarkers(gray)
        if ids is None:
            continue
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
        for c, i in zip(corners, ids.ravel()):
            c = cv2.cornerSubPix(gray, c.reshape(4, 1, 2).astype(np.float32), (5, 5), (-1, -1), crit)
            acc.setdefault(int(i), []).append(c.reshape(4, 2))
    seen = {i: np.median(np.array(v), axis=0) for i, v in acc.items() if len(v) >= frames // 3}
    return seen, img


def solve(seen, markers, size, drop_max=4):
    """Intrinsics + pose from the non-planar marker corners; drop the worst marker while it is an outlier."""
    ids = [i for i in seen if i in markers]
    w, h = size
    for _ in range(drop_max + 1):
        if len(ids) < 4:
            raise SystemExit(f"only {len(ids)} known markers in view: need 4+ on 2+ walls")
        obj = np.vstack([markers[i].corners_map() for i in ids]).astype(np.float32)
        img = np.vstack([seen[i] for i in ids]).astype(np.float32)
        K0 = np.array([[w, 0, w / 2], [0, w, h / 2], [0, 0, 1]], np.float64)  # ~53 deg horizontal FOV guess
        flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_ZERO_TANGENT_DIST
                 | cv2.CALIB_FIX_K3)
        rms, K, D, rv, tv = cv2.calibrateCamera([obj], [img], (w, h), K0, np.zeros(5), flags=flags)
        proj, _ = cv2.projectPoints(obj, rv[0], tv[0], K, D)
        err = np.linalg.norm(proj.reshape(-1, 2) - img, axis=1).reshape(-1, 4).mean(axis=1)
        worst = int(np.argmax(err))
        if err[worst] < max(3.0, 3 * np.median(err)):
            break
        print(f"  marker {ids[worst]}: {err[worst]:.1f} px off (moved?): dropped")
        ids.pop(worst)
    normals = {tuple(np.round(markers[i].R[:, 2])) for i in ids}
    R, _ = cv2.Rodrigues(rv[0])
    C = (-R.T @ tv[0]).ravel()  # camera centre in map
    return dict(K=K, D=D.ravel(), R=R, C=C, rms=rms, ids=ids, walls=len(normals),
                err={i: float(e) for i, e in zip(ids, err)})


def cmd_locate(a):
    _, markers = load_marker_map(REPO / "config" / "markers.yaml")
    dict_name = yaml.safe_load(open(REPO / "config" / "markers.yaml"))["dictionary"]
    cap = open_cam(a.cam)
    seen, img = detect_markers(cap, a.frames, dict_name)
    cap.release()
    h, w = img.shape[:2]
    print(f"{len(seen)} markers in view: {sorted(seen)}")
    s = solve(seen, markers, (w, h))
    fov = math.degrees(2 * math.atan(w / 2 / s['K'][0, 0]))
    look = s["R"].T @ np.array([0, 0, 1.0])  # optical axis in map
    print(f"RMS {s['rms']:.2f} px over {len(s['ids'])} markers on {s['walls']} walls; f = {s['K'][0, 0]:.0f} px "
          f"(HFOV {fov:.0f} deg), k1 {s['D'][0]:+.3f}")
    print(f"camera at ({s['C'][0]:+.2f}, {s['C'][1]:+.2f}, {s['C'][2]:.2f}) m, looking "
          f"{math.degrees(math.atan2(look[1], look[0])):+.0f} deg, pitch {math.degrees(math.asin(look[2])):+.0f} deg")
    if s["walls"] < 2:
        print("WARNING: markers on one wall only: the focal length is poorly determined (plane). Turn the camera.")
    out = {"image_size": [w, h], "K": s["K"].round(3).tolist(), "D": s["D"].round(5).tolist(),
           "R_cam_to_map": s["R"].T.round(6).tolist(), "C_map": s["C"].round(4).tolist(),
           "rms_px": round(float(s["rms"]), 3), "markers": s["ids"],
           "stamp": time.strftime("%Y-%m-%d %H:%M")}
    CALIB_OUT.write_text("# external camera, solved by tools/ext_camera.py locate (do not edit by hand)\n"
                         + yaml.safe_dump(out, sort_keys=False))
    print(f"saved {CALIB_OUT}")
    if not a.no_show:
        proj = {i: cv2.projectPoints(markers[i].corners_map(), cv2.Rodrigues(s['R'])[0],
                                     -s['R'] @ s['C'], s['K'], s['D'])[0].reshape(4, 2) for i in s['ids']}
        for i, c in proj.items():
            cv2.polylines(img, [c.astype(np.int32)], True, (0, 255, 0), 2)
            cv2.putText(img, str(i), tuple(c[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imwrite(str(REPO / "logs" / "ext_camera_locate.jpg"), img)
        print("check: logs/ext_camera_locate.jpg (green = map markers reprojected with the solved camera)")


# ---------------------------------------------------------------- detect
HSV = {"red": [((0, 110, 60), (10, 255, 255)), ((170, 110, 60), (180, 255, 255))],
       "blue": [((95, 120, 50), (130, 255, 255))]}


def colour_of(img, box):
    x0, y0, x1, y1 = map(int, box)
    roi = cv2.cvtColor(img[max(y0, 0):y1, max(x0, 0):x1], cv2.COLOR_BGR2HSV)
    if roi.size == 0:
        return None
    score = {c: sum(int(cv2.inRange(roi, lo, hi).sum() // 255) for lo, hi in rs) for c, rs in HSV.items()}
    c = max(score, key=score.get)
    return c if score[c] > 0.25 * roi.shape[0] * roi.shape[1] else None


def detect_colour(img, min_px=12):
    hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV)
    out = []
    for c, rs in HSV.items():
        m = sum(cv2.inRange(hsv, lo, hi) for lo, hi in rs).clip(0, 255).astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        n, _, st, _ = cv2.connectedComponentsWithStats(m)
        for k in range(1, n):
            x, y, w, h, area = st[k]
            if min(w, h) >= min_px and 0.6 < w / h < 1.6 and area > 0.5 * w * h:  # round-ish blob
                out.append((c, (x, y, x + w, y + h), 1.0))
    return out


class YoloWorld:
    def __init__(self, weights):
        from ultralytics import YOLO
        if not Path(weights).exists() and Path(weights).name == "yolo_world_balloon.pt":
            # first run: the open-vocabulary model, frozen on the word "balloon" (needs CLIP once, ~340 MB)
            m = YOLO(str(Path(weights).with_name("yolov8s-worldv2.pt")))
            m.set_classes(["balloon"])
            m.save(str(weights))
        self.m = YOLO(str(weights))
        if list(self.m.names.values()) != ["balloon"]:
            self.m.set_classes(["balloon"])

    def __call__(self, img, conf=0.15):
        r = self.m.predict(img, conf=conf, verbose=False)[0]
        out = []
        for b, s in zip(r.boxes.xyxy.cpu().numpy(), r.boxes.conf.cpu().numpy()):
            c = colour_of(img, b)
            if c:
                out.append((c, tuple(b), float(s)))
        return out


def to_map(cal, box, diameter):
    """Balloon centre in map: ray through the box centre, distance from the apparent width."""
    K, D = np.array(cal["K"]), np.array(cal["D"])
    u, v = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
    w_px = 0.5 * ((box[2] - box[0]) + (box[3] - box[1]))
    xn = cv2.undistortPoints(np.array([[[u, v]]], np.float64), K, D).ravel()
    ray = np.array([xn[0], xn[1], 1.0])
    ray /= np.linalg.norm(ray)
    d = K[0, 0] * diameter / max(w_px, 1.0)
    return np.array(cal["C_map"]) + np.array(cal["R_cam_to_map"]) @ (ray * d), d


class ImageSource:
    """A still image as an endless video (offline tests)."""
    def __init__(self, path):
        self.img = cv2.imread(str(path))
        if self.img is None:
            raise SystemExit(f"cannot read {path}")

    def read(self):
        time.sleep(0.03)
        return True, self.img.copy()

    def release(self):
        pass


def open_source(a):
    if a.source is None:
        return open_cam(a.cam), True
    if Path(a.source).suffix.lower() in (".jpg", ".jpeg", ".png", ".bmp"):
        return ImageSource(a.source), False
    cap = cv2.VideoCapture(a.source)
    if not cap.isOpened():
        raise SystemExit(f"cannot open {a.source}")
    return cap, False


def cmd_detect(a):
    calib = Path(a.calib)
    cal = yaml.safe_load(calib.read_text()) if calib.exists() else None
    if cal is None:
        print("no config/ext_camera.yaml: run 'locate' first (balloons shown in pixels only)")
    # the room = the box of the wall markers (+0.2 m), floor to ceiling: anything found outside is dropped
    mm = yaml.safe_load(open(REPO / "config" / "markers.yaml"))
    P = np.array([[e["x"], e["y"], e["z"]] for e in mm["markers"]])
    lo = np.array([*(P[:, :2].min(0) - 0.2), 0.1])
    hi = np.array([*(P[:, :2].max(0) + 0.2), mm["room"]["height"]])
    yolo = YoloWorld(a.weights) if a.method in ("yolo", "compare") else None
    cap, live = open_source(a)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rec_dir = REPO / "logs" / "balloons"
    rec_dir.mkdir(parents=True, exist_ok=True)
    rec = None
    t_last, fps, t_img, n_img = time.time(), 0.0, 0.0, 0
    stats = {"frames": 0, "colour": {"red": 0, "blue": 0}, "yolo": {"red": 0, "blue": 0}}
    while True:
        ok, img = cap.read()
        if not ok:
            if live:
                continue
            break  # end of the recording
        raw = img.copy()
        if a.record and live:
            if rec is None:
                path = rec_dir / time.strftime("rec_%Y%m%d_%H%M%S.mp4")
                rec = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 15, (img.shape[1], img.shape[0]))
                print(f"recording the raw video to {path}")
            rec.write(raw)
        t = time.time()
        dets = {"colour": detect_colour(img) if a.method in ("colour", "compare") else [],
                "yolo": yolo(img) if yolo else []}
        fps = 0.9 * fps + 0.1 / max(time.time() - t_last, 1e-3)
        t_last = time.time()
        stats["frames"] += 1
        for name in ("colour", "yolo"):
            for c in {d[0] for d in dets[name]}:
                stats[name][c] += 1
        main = dets["yolo"] if a.method == "yolo" else dets["colour"]
        balloons = []
        for c, box, s in main:
            if cal:
                p, d = to_map(cal, box, a.diameter)
                if not (lo <= p).all() or not (p <= hi).all():
                    continue  # outside the room (a blue sheet, a red poster): not a balloon
                balloons.append({"colour": c, "x": round(float(p[0]), 3), "y": round(float(p[1]), 3),
                                 "z": round(float(p[2]), 3), "dist": round(float(d), 2), "score": round(s, 2)})
        sock.sendto(json.dumps({"t": t, "balloons": balloons}).encode(), UDP)
        for name, col in (("colour", (0, 255, 255)), ("yolo", (255, 0, 255))):
            for c, box, s in dets[name]:
                x0, y0, x1, y1 = map(int, box)
                cv2.rectangle(img, (x0, y0), (x1, y1), col, 2)
                cv2.putText(img, f"{name[0]}:{c} {s:.2f}", (x0, y0 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        for i, b in enumerate(balloons):
            cv2.putText(img, f"{b['colour']} ({b['x']:+.2f},{b['y']:+.2f},{b['z']:.2f}) m", (10, 25 + 22 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(img, f"{fps:.1f} fps", (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        # the annotated image, small, ~5 Hz, to RViz (balloons node -> /balloons/image). In 1200-byte pieces: the
        # WSL mirrored loopback drops UDP packets above ~1.5 kB (measured Nov 19). Piece: frame, index, count, data
        if time.time() - t_img > 0.2:
            t_img = time.time()
            ok, jpg = cv2.imencode(".jpg", cv2.resize(img, (640, 360)), [cv2.IMWRITE_JPEG_QUALITY, 60])
            if ok:
                data = jpg.tobytes()
                parts = [data[i:i + 1200] for i in range(0, len(data), 1200)]
                n_img = (n_img + 1) % 65536
                for k, part in enumerate(parts):
                    sock.sendto(struct.pack("!HHH", n_img, k, len(parts)) + part, UDP_IMG)
        if not a.no_show:
            cv2.imshow("balloons (Esc: quit, S: save image)", img)
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break
            if k in (ord("s"), ord("S")):
                stamp = f"{time.time():.0f}"
                cv2.imwrite(str(rec_dir / f"{stamp}_raw.jpg"), raw)
                cv2.imwrite(str(rec_dir / f"{stamp}_annotated.jpg"), img)
                print(f"saved {rec_dir / stamp}_raw.jpg / _annotated.jpg")
        if a.max_frames and stats["frames"] >= a.max_frames:
            break
    cap.release()
    if rec is not None:
        rec.release()
    n = max(stats["frames"], 1)
    print(f"{stats['frames']} frames, {fps:.1f} fps. Share of frames where each balloon was found:")
    for name in ("colour", "yolo"):
        print(f"  {name:6s}  red {100 * stats[name]['red'] / n:5.1f} %   blue {100 * stats[name]['blue'] / n:5.1f} %")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    p = sub.add_parser("locate")
    p.add_argument("--cam", type=int, default=1)
    p.add_argument("--frames", type=int, default=30)
    p.add_argument("--no-show", action="store_true")
    p = sub.add_parser("detect")
    p.add_argument("--cam", type=int, default=1)
    p.add_argument("--method", choices=["colour", "yolo", "compare"], default="compare")
    p.add_argument("--calib", default=str(CALIB_OUT), help="camera intrinsics + pose (written by 'locate')")
    p.add_argument("--source", help="video or image file instead of the camera (replay a recording)")
    p.add_argument("--record", action="store_true", help="save the raw camera video in logs/balloons/")
    p.add_argument("--max-frames", type=int, default=0, help="stop after N frames (0: Esc)")
    p.add_argument("--weights", default=str(REPO / "models" / "yolo_world_balloon.pt"),
                   help="downloaded automatically by ultralytics the first time")
    p.add_argument("--diameter", type=float, default=0.28, help="balloon diameter (m), measure it")
    p.add_argument("--no-show", action="store_true")
    a = ap.parse_args()
    (REPO / "logs").mkdir(exist_ok=True)
    {"list": cmd_list, "locate": cmd_locate, "detect": cmd_detect}[a.cmd](a)


if __name__ == "__main__":
    main()
