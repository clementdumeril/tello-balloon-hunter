"""Marker tour: the drone visits every marker of the map (stops 1.2 m in front of it, facing it), then comes
back to its start point and lands. First autonomous mission, with a simple position controller.

    ros2 launch tello_driver real.launch.py          (terminal 1, drone on the floor 2 m from the marker wall)
    ros2 run tello_driver tello_marker_tour           (terminal 2, NO teleop running; you stay next to the drone)

For each marker: turn in place to face it, fly to 1.2 m in front of it at its height (kept between 1.2 and
2.2 m: above the bed and the desk), hold 2 s. Speed at most 0.3 m/s. Flight zone: 0.6 m from every wall that
holds markers, and inside the room scan on the other sides.

SAFETY: ANY KEY = stop and land. Ctrl+C = land. Automatic landing if the drone leaves the flight zone
(+0.3 m tolerance), if no marker is seen for 4 s, or on low battery. The driver's ceiling guard (2.6 m) always applies.
"""
import math
import os
import time
from pathlib import Path

import numpy as np

from aruco_localization.marker_map import load_marker_map
from tello_driver.step_test import Abort, StepTest, run, wrap

REPO = os.environ.get("TELLO_REPO", str(Path.home() / "tello_ws/src/tello-balloon-hunter"))


def clip(v, m):
    return max(-m, min(m, v))


class MarkerTour(StepTest):
    NODE_NAME = "tello_marker_tour"
    PREFIX = "tour"

    def __init__(self):
        super().__init__()
        p = self.declare_parameter
        self.markers_file = p("markers_file", os.path.join(REPO, "config", "markers.yaml")).value
        self.standoff = p("standoff", 1.2).value          # m in front of each marker
        self.margin = p("wall_margin", 0.6).value         # m from the walls
        self.z_lo, self.z_hi = p("tour_z_min", 1.2).value, p("tour_z_max", 2.2).value
        self.v_max = p("max_speed", 0.3).value            # m/s horizontal
        self.vz_max = p("max_vertical_speed", 0.25).value
        self.w_max = p("max_yaw_rate", 0.5).value         # rad/s
        self.kp, self.kp_yaw = p("kp", 0.6).value, p("kp_yaw", 0.9).value
        self.hold = p("hold", 2.0).value
        # room scan extents in the map frame (x min, x max, y min, y max), for the sides without markers
        self.room = list(p("room_box", [-2.99, 1.46, -0.02, 3.12]).value)
        self.lost_max = p("tour_lost_timeout", 4.0).value
        _, self.markers = load_marker_map(self.markers_file)
        self.fence = self.flight_zone()

    def flight_zone(self):
        """Axis-aligned box: room scan shrunk by the margin, and margin from each marker wall."""
        xmin, xmax, ymin, ymax = (self.room[0] + self.margin, self.room[1] - self.margin,
                                  self.room[2] + self.margin, self.room[3] - self.margin)
        for m in self.markers.values():
            n = m.R[:, 2]
            if n[0] > 0.9:
                xmin = max(xmin, m.t[0] + self.margin)
            elif n[0] < -0.9:
                xmax = min(xmax, m.t[0] - self.margin)
            elif n[1] > 0.9:
                ymin = max(ymin, m.t[1] + self.margin)
            elif n[1] < -0.9:
                ymax = min(ymax, m.t[1] - self.margin)
        if xmin >= xmax or ymin >= ymax:
            raise Abort(f"empty flight zone x [{xmin:.2f}, {xmax:.2f}] y [{ymin:.2f}, {ymax:.2f}]")
        return (xmin, xmax, ymin, ymax)

    def waypoints(self):
        """One stop per marker, ordered around the room starting from the one closest to the start point."""
        f = self.fence
        cx, cy = (f[0] + f[1]) / 2, (f[2] + f[3]) / 2
        wps = []
        for m in self.markers.values():
            n = m.R[:, 2].copy()
            n[2] = 0.0
            n /= np.linalg.norm(n)
            x = min(max(m.t[0] + self.standoff * n[0], f[0]), f[1])
            y = min(max(m.t[1] + self.standoff * n[1], f[2]), f[3])
            z = min(max(m.t[2], self.z_lo), self.z_hi)
            yaw = math.atan2(m.t[1] - y, m.t[0] - x)  # facing the marker
            wps.append({"id": m.id, "p": (x, y, z), "yaw": yaw, "ang": math.atan2(m.t[1] - cy, m.t[0] - cx)})
        wps.sort(key=lambda w: w["ang"])
        # start with the wall the drone is facing (no blind turn first), then go round the room
        x0, y0, yaw0 = self.p0[0], self.p0[1], self.p0[3]
        k = min(range(len(wps)), key=lambda i: (round(abs(wrap(wps[i]["yaw"] - yaw0)) / 0.5),
                                                  math.hypot(wps[i]["p"][0] - x0, wps[i]["p"][1] - y0)))
        return wps[k:] + wps[:k]

    # ---------------------------------------------------------------- control
    def goto(self, target, yaw_t, timeout=25.0, tol=0.12, yaw_only=False):
        """Fly to target (x, y, z) with heading yaw_t; yaw_only: turn in place first. Returns when reached."""
        tx, ty, tz = target
        reached_t = [None]

        def step():
            if yaw_only and self.pose_age() > 0.5:
                # markers out of view during a turn: keep turning on the IMU heading (no translation)
                yaw = self.yaw_estimate()
                if yaw is None:
                    self.send()
                    return None
                eyaw = wrap(yaw_t - yaw)
                if abs(eyaw) < math.radians(10):
                    self.send()
                    return None  # wait for the markers to confirm the heading
                self.send(cw=clip(self.kp_yaw * eyaw, self.w_max), blind=True)
                return None
            if self.pose_age() > 0.5:
                self.send()  # no fresh pose: hover (the driver/Tello hold position)
                return None
            _, x, y, z, yaw = self.pose
            eyaw = wrap(yaw_t - yaw)
            if yaw_only:
                ex = ey = ez = 0.0
                ok = abs(eyaw) < math.radians(10)
            else:
                ex, ey, ez = tx - x, ty - y, tz - z
                ok = math.hypot(ex, ey) < tol and abs(ez) < 0.1 and abs(eyaw) < math.radians(10)
            if ok:
                reached_t[0] = reached_t[0] or time.time()
                if time.time() - reached_t[0] > 0.6:
                    self.send()
                    return False
            else:
                reached_t[0] = None
            vx, vy = self.kp * ex, self.kp * ey
            v = math.hypot(vx, vy)
            if v > self.v_max:
                vx, vy = vx * self.v_max / v, vy * self.v_max / v
            c, s = math.cos(yaw), math.sin(yaw)
            self.send(c * vx + s * vy, -s * vx + c * vy, clip(self.kp * ez, self.vz_max),
                      clip(self.kp_yaw * eyaw, self.w_max))
            return None
        t0 = time.time()
        self.run(timeout, step)
        if time.time() - t0 >= timeout - 0.05:
            self.say("  (not reached in time, going on)")

    def main(self):
        self.prepare()
        f = self.fence
        self.say(f"Flight zone x [{f[0]:+.2f}, {f[1]:+.2f}]  y [{f[2]:+.2f}, {f[3]:+.2f}]  "
                 f"z [{self.z_lo:.1f}, {self.z_hi:.1f}] m. Max speed {self.v_max} m/s.")
        x0, y0, z0, yaw0 = self.p0
        if not (f[0] <= x0 <= f[1] and f[2] <= y0 <= f[3]):
            raise Abort(f"start point ({x0:+.2f}, {y0:+.2f}) outside the flight zone: start closer to the room centre")
        wps = self.waypoints()
        self.say("Tour: " + " -> ".join(str(w["id"]) for w in wps))
        for i, w in enumerate(wps):
            self.say(f"[{i + 1}/{len(wps)}] marker {w['id']}: turning to face it")
            _, x, y, z, _ = self.pose
            self.goto((x, y, z), w["yaw"], timeout=12.0, yaw_only=True)
            self.say(f"[{i + 1}/{len(wps)}] marker {w['id']}: flying to ({w['p'][0]:+.2f}, {w['p'][1]:+.2f}, "
                     f"{w['p'][2]:.2f})")
            self.goto(w["p"], w["yaw"])
            self.run(self.hold, lambda: self.send())
        self.say("Back to the start point")
        self.goto((x0, y0, max(z0, self.z_lo)), yaw0)
        self.say("Tour done: landing.")


def main():
    run(MarkerTour)


if __name__ == "__main__":
    main()
