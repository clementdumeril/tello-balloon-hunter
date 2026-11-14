"""Fake drone for tests without hardware: renders the REAL marker map (config/markers.yaml) seen by a virtual
Tello camera, and moves that camera with the rc commands (same gains as the real drone).

So `real.launch.py mock:=true` exercises the whole chain (image -> ArUco pose -> TF -> RViz), and the teleop
flies the virtual drone in RViz exactly like the real one. Simple dynamics: first-order velocity response.
"""
import math
import os
import time

import cv2
import numpy as np

from aruco_localization.marker_map import aruco_dictionary, load_marker_map

LIN_GAIN = 0.38 / 30              # m/s per rc point (measured on the real Tello, Nov 7)
YAW_GAIN = math.radians(18) / 30  # rad/s per rc point
# The real Tello's identified response (model.py, identification of Nov 13): the fake drone behaves like it.
# The command delay L_U is applied here, the video delay L_V in the image pipeline (driver's FakeFrameRead).
from tello_driver import model as M  # noqa: E402
TAU = M.TAU_XY                    # s, velocity response time constant
K_X, K_Y = M.K_X, M.K_Y           # real speed / commanded speed, forward and left
DELAY = M.L_U                     # s, command -> motion delay (radio + flight controller)
VIDEO_DELAY = M.L_V               # s, image taken -> image received
CAM_OFFSET = np.array([0.05, 0.0, 0.0])
R_BASE_CAM = np.column_stack([[0, -1, 0], [0, 0, -1], [1, 0, 0]]).astype(float)  # optical frame in body frame


def marker_image(dictionary, mid, px=240):
    """Printed marker: black ArUco square (px) centred on a white sheet 1.5x larger."""
    if hasattr(cv2.aruco, "generateImageMarker"):
        m = cv2.aruco.generateImageMarker(dictionary, mid, px)
    else:
        m = cv2.aruco.drawMarker(dictionary, mid, px)
    sheet = np.full((px * 3 // 2, px * 3 // 2), 245, np.uint8)
    o = px // 4
    sheet[o:o + px, o:o + px] = m
    return cv2.cvtColor(sheet, cv2.COLOR_GRAY2RGB)


class FakeWorld:
    def __init__(self, markers_file, width=960, height=720):
        self.cfg, self.markers = load_marker_map(markers_file)
        d = aruco_dictionary(self.cfg.get("dictionary", "DICT_4X4_50"))
        self.images = {i: marker_image(d, i) for i in self.markers}
        self.w, self.h = width, height
        f = width / 2 / math.tan(math.radians(55.6) / 2)
        self.K = np.array([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1]], float)
        self.D = np.zeros(5)
        self.p, self.yaw = self.start_pose()
        self.v = np.zeros(3)   # body-frame velocity (x fwd, y left, z up)
        self.yaw_rate = 0.0
        self.rc = (0, 0, 0, 0)
        self.rc_queue = []  # (time, rc): applied DELAY s after it was sent
        self.flying = False
        self.z_target = None
        self.t = time.time()

    def set_camera(self, K, D):
        self.K, self.D = np.asarray(K, float).reshape(3, 3), np.asarray(D, float).ravel()

    def start_pose(self):
        """On the floor, 2 m in front of the wall that holds the most markers, facing it (like a real start).
        FAKE_START=x,y,yaw_deg: start there instead (e.g. on the takeoff pad)."""
        st = os.environ.get("FAKE_START")
        if st:
            x, y, yaw = (float(v) for v in st.split(","))
            return np.array([x, y, 0.0]), math.radians(yaw)
        groups = {}
        for m in self.markers.values():
            groups.setdefault(round(math.degrees(math.atan2(m.R[1, 2], m.R[0, 2])) / 45), []).append(m)
        wall = max(groups.values(), key=len)
        c = np.mean([m.t for m in wall], axis=0)
        n = np.mean([m.R[:, 2] for m in wall], axis=0)
        n[2] = 0.0
        n /= np.linalg.norm(n)
        p = c + 2.0 * n
        p[2] = 0.0
        return p, math.atan2(-n[1], -n[0])

    # ---------------------------------------------------------------- dynamics
    def step(self):
        now = time.time()
        dt = min(now - self.t, 0.1)
        self.t = now
        self.rc_queue.append((now, self.rc))
        while len(self.rc_queue) > 1 and self.rc_queue[1][0] <= now - DELAY:
            self.rc_queue.pop(0)
        rc = self.rc_queue[0][1] if self.rc_queue[0][0] <= now - DELAY else (0, 0, 0, 0)
        lr, fb, ud, yw = rc if self.flying else (0, 0, 0, 0)
        v_target = np.array([fb * K_X, -lr * K_Y, ud], float) * LIN_GAIN
        a = min(1.0, dt / TAU)
        self.v += (v_target - self.v) * a
        if self.z_target is not None:
            # takeoff / 'up' / land: the Tello's own position loop, it stops on the target (no lag, no overshoot)
            dz = self.z_target - self.p[2]
            self.v[2] = max(-0.6, min(0.6, 2.0 * dz))
            if abs(dz) < 0.02:
                self.z_target = None
                self.v[2] = 0.0
        self.yaw_rate += (-yw * YAW_GAIN - self.yaw_rate) * a
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.p += np.array([c * self.v[0] - s * self.v[1], s * self.v[0] + c * self.v[1], self.v[2]]) * dt
        drift = os.environ.get("FAKE_DRIFT")  # tests: "vx,vy" m/s in the map, like the real Tello without its VPS
        if drift and self.flying:
            self.p[:2] += np.array([float(v) for v in drift.split(",")]) * dt
        self.p[2] = max(0.05, self.p[2])
        self.yaw = math.atan2(math.sin(self.yaw + self.yaw_rate * dt), math.cos(self.yaw + self.yaw_rate * dt))

    # ---------------------------------------------------------------- rendering
    def render(self):
        self.step()
        img = np.empty((self.h, self.w, 3), np.uint8)
        img[:] = np.linspace(150, 110, self.h, dtype=np.uint8)[:, None, None]  # plain wall, a bit darker lower down
        # tests: FAKE_BLACKOUT="a:b" -> no marker in the picture from a to b s after the takeoff (search behaviour)
        bo = os.environ.get("FAKE_BLACKOUT")
        if bo and self.flying:
            self.t_fly = getattr(self, "t_fly", None) or time.time()
            a, b = (float(v) for v in bo.split(":"))
            if a <= time.time() - self.t_fly <= b:
                return img
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        R_map_base = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        R_map_cam = R_map_base @ R_BASE_CAM
        cam = self.p + R_map_base @ CAM_OFFSET
        R_cm = R_map_cam.T                     # map -> camera
        rvec, _ = cv2.Rodrigues(R_cm)
        tvec = -R_cm @ cam
        order = sorted(self.markers.values(), key=lambda m: -np.linalg.norm(m.t - cam))  # far ones first
        for m in order:
            if np.dot(cam - m.t, m.R[:, 2]) <= 0.05:  # seen from behind or edge-on
                continue
            hs = 0.75 * m.size
            local = np.array([[-hs, hs, 0], [hs, hs, 0], [hs, -hs, 0], [-hs, -hs, 0]])
            world = (m.R @ local.T).T + m.t
            if np.min((world @ R_cm.T + tvec)[:, 2]) < 0.15:
                continue  # behind the camera
            uv, _ = cv2.projectPoints(world, rvec, tvec, self.K, self.D)
            uv = uv.reshape(4, 2).astype(np.float32)
            x0, y0 = np.floor(uv.min(0)).astype(int)
            x1, y1 = np.ceil(uv.max(0)).astype(int)
            x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, self.w), min(y1, self.h)
            if x1 - x0 < 2 or y1 - y0 < 2:
                continue
            src = self.images[m.id]
            n = src.shape[0] - 1
            H = cv2.getPerspectiveTransform(np.float32([[0, 0], [n, 0], [n, n], [0, n]]), uv - np.float32([x0, y0]))
            size = (x1 - x0, y1 - y0)
            warped = cv2.warpPerspective(src, H, size, flags=cv2.INTER_LINEAR)
            mask = cv2.warpPerspective(np.full(src.shape[:2], 255, np.uint8), H, size) > 127
            img[y0:y1, x0:x1][mask] = warped[mask]
        return img

    def state(self):
        return {"pitch": 0, "roll": 0, "yaw": int(round(-math.degrees(self.yaw))),
                "vgx": int(round(self.v[0] * 10)), "vgy": int(round(self.v[1] * 10)), "vgz": int(round(-self.v[2] * 10)),
                "templ": 50, "temph": 52, "tof": int(self.p[2] * 100), "h": int(self.p[2] * 100) if self.flying else 0, "bat": 90,
                "baro": float(self.p[2] * 100), "time": 0, "agx": 0.0, "agy": 0.0, "agz": -1000.0,
                "true_x": round(float(self.p[0]), 3), "true_y": round(float(self.p[1]), 3), "true_z": round(float(self.p[2]), 3),
                "true_yaw_deg": round(math.degrees(self.yaw), 1)}
