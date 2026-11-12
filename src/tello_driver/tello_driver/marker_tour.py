"""Marker tour: survey, then the drone visits every marker of the map: quick approach to ~0.9 m in front of it,
a short stop, back off to ~1.6 m for the overview, next one. Then back to the start point and landing.

    ros2 launch tello_driver real.launch.py          (terminal 1, drone on the floor 2 m from the marker wall)
    ros2 run tello_driver tello_marker_tour           (terminal 2, NO teleop running; you stay next to the drone)

Survey: a full turn in place after the climb (sees every wall, fixes the position, the EKF learns the drift).
For each marker: turn to face it, approach at up to 0.5 m/s with a braking profile (stops without overshoot),
stop 1 s, back off. Heights 1.2-2.2 m (above the bed and the desk). Waypoints only in the obstacle-free space
of the room scan; the driver also stops before any marker closer than 0.7 m (its size in the image).

SAFETY: ANY KEY = stop and land. Ctrl+C = land. Automatic landing if the drone leaves the flight zone
(+0.3 m tolerance), if no marker is seen for 12 s, or on low battery. Markers lost for more than 1 s:
the drone stops and turns to look for them (towards the marker it is going to, then sweeping +-40 deg). The driver's ceiling guard (2.6 m) always applies.
"""
import math
import os
import time
from pathlib import Path

import numpy as np

from aruco_localization.marker_map import load_marker_map
from tello_driver.step_test import RATE, Abort, StepTest, run, wrap

REPO = os.environ.get("TELLO_REPO", str(Path.home() / "tello_ws/src/tello-balloon-hunter"))


def clip(v, m):
    return max(-m, min(m, v))


class MarkerTour(StepTest):
    NODE_NAME = "tello_marker_tour"
    PREFIX = "tour"
    MOVE_TO_ZONE_CENTRE = False  # the tour moves by itself
    POSE_TOPIC = "/ekf/pose"     # smooth, delay-compensated, predicted for a few seconds without markers

    def __init__(self):
        super().__init__()
        p = self.declare_parameter
        self.markers_file = p("markers_file", os.path.join(REPO, "config", "markers.yaml")).value
        self.standoff = p("standoff", 1.6).value          # m in front of each marker: overview (back off)
        self.approach = p("approach", 0.9).value          # m in front of each marker: the visit
        self.a_max = p("max_accel", 0.6).value            # m/s2, braking profile
        self.survey = p("survey", True).value
        self.margin = p("wall_margin", 0.6).value         # m from the walls
        self.z_lo, self.z_hi = p("tour_z_min", 1.2).value, p("tour_z_max", 2.1).value  # 2.2 overshot to 2.40 (Nov 11)
        if self.band_z:  # band mode: every stop inside the band
            self.z_lo, self.z_hi = self.band_z
        self.v_max = p("max_speed", 0.5).value            # m/s horizontal
        self.vz_max = p("max_vertical_speed", 0.25).value
        self.w_max = p("max_yaw_rate", 0.5).value         # rad/s
        self.kp, self.kp_yaw = p("kp", 0.6).value, p("kp_yaw", 0.9).value
        self.hold = p("hold", 1.0).value
        # room scan extents in the map frame (x min, x max, y min, y max), for the sides without markers
        # (default: no limit, the obstacle-free zone of the scan and the marker walls bound the tour)
        self.room = list(p("room_box", [-99.0, 99.0, -99.0, 99.0]).value)
        self.lost_max = p("tour_lost_timeout", 12.0).value  # searching before giving up and landing
        # sequence mode (tello_goto 49 35): only these markers, in this order, back to the room centre between them
        seq = p("sequence", os.environ.get("GOTO_SEQ", "")).value
        self.sequence = [int(s) for s in str(seq).replace(";", ",").replace(" ", ",").split(",") if s.strip()]
        _, self.markers = load_marker_map(self.markers_file)
        missing = [i for i in self.sequence if i not in self.markers]
        if missing:
            raise SystemExit(f"marker(s) {missing} not in {self.markers_file}")
        self.fence = self.flight_zone()
        self.centroid = (float(np.mean([m.t[0] for m in self.markers.values()])),
                         float(np.mean([m.t[1] for m in self.markers.values()])))
        self.last_xy = None

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
        if self.zone:  # the obstacle-free zone from the room scan wins
            xmin, xmax = max(xmin, self.zone["x_min"]), min(xmax, self.zone["x_max"])
            ymin, ymax = max(ymin, self.zone["y_min"]), min(ymax, self.zone["y_max"])
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
            z = min(max(m.t[2], self.z_lo), self.z_hi)
            # a marker higher/lower than the drone can go: back off until it sits in the middle 60 % of the
            # vertical field of view (43 deg); e.g. the west markers at 2.5-2.7 m were cut at the image top
            d_min = abs(m.t[2] - z) / (0.6 * math.tan(math.radians(21.5)))
            near = self.feasible(m, n, max(self.approach, d_min))
            x, y = self.feasible(m, n, max(self.standoff, d_min, math.hypot(near[0] - m.t[0], near[1] - m.t[1])))
            yaw = math.atan2(m.t[1] - y, m.t[0] - x)  # facing the marker
            wall = round(math.degrees(math.atan2(n[1], n[0])) / 90) % 4  # which wall (normal direction)
            wps.append({"id": m.id, "p": (x, y, z), "near": (near[0], near[1], z), "yaw": yaw, "wall": wall,
                        "ang": math.atan2(m.t[1] - cy, m.t[0] - cx)})
        wps.sort(key=lambda w: w["ang"])
        # start with the wall the drone is facing (no blind turn first), then go round the room
        x0, y0, yaw0 = self.p0[0], self.p0[1], self.p0[3]
        k = min(range(len(wps)), key=lambda i: (round(abs(wrap(wps[i]["yaw"] - yaw0)) / 0.5),
                                                  math.hypot(wps[i]["p"][0] - x0, wps[i]["p"][1] - y0)))
        return wps[k:] + wps[:k]

    def feasible(self, m, n, d):
        """Point d m in front of marker m (normal n), moved further back until it is in the obstacle-free space
        (room scan grid, margin + 10 cm); without grid, clamped into the mission rectangle."""
        f = self.fence
        if self.zone and "grid" in self.zone:
            from tello_driver.driver import clearance
            for k in range(40):
                x, y = m.t[0] + (d + 0.05 * k) * n[0], m.t[1] + (d + 0.05 * k) * n[1]
                if clearance(self.zone, x, y)[0] >= self.zone["margin"] + 0.15:  # +0.10 overshot by 5 cm (sim)
                    return x, y
        x = min(max(m.t[0] + d * n[0], f[0]), f[1])
        y = min(max(m.t[1] + d * n[1], f[2]), f[3])
        return x, y

    def survey_turn(self):
        """A full turn in place at ~0.45 rad/s (IMU heading): every wall seen from the start point."""
        self.say("Survey: one full turn to see every wall")
        total, last = 0.0, self.yaw_estimate()
        lost = self.lost_max
        self.lost_max = max(lost, 15.0)

        def step():
            nonlocal total, last
            yaw = self.yaw_estimate()
            if yaw is not None and last is not None:
                total += abs(wrap(yaw - last))
            last = yaw
            if total >= 2 * math.pi - 0.1:
                self.send()
                return False
            self.send(cw=0.45, blind=True)
            return None
        try:
            self.run(25.0, step, search_on_loss=False)
        finally:
            self.lost_max = lost

    # ---------------------------------------------------------------- control
    def search(self, look_at):
        """No marker in view for > 1 s: turn (IMU heading) towards look_at, then sweep +-40 deg around it,
        without moving, until the markers come back (the safety check lands after tour_lost_timeout)."""
        yaw = self.yaw_estimate()
        if yaw is None or self.last_xy is None:
            self.send()
            return
        self.search_heading(math.atan2(look_at[1] - self.last_xy[1], look_at[0] - self.last_xy[0]))

    def goto(self, target, yaw_t, timeout=25.0, tol=0.12, yaw_only=False, look_at=None):
        """Fly to target (x, y, z) with heading yaw_t; yaw_only: turn in place first. Returns when reached.
        look_at: where to turn to if the markers are lost (default: the markers' centroid)."""
        tx, ty, tz = target
        reached_t = [None]
        integ = [0.0, 0.0]
        look = look_at or self.centroid

        def step():
            if self.pose_age() < 0.5:
                self.last_xy = self.pose[1:3]
            if self.pose_age() > 1.0 and not yaw_only:
                self.search(look)
                return None
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
            # speed: v_max, but brake in time: v <= sqrt(2 a d) (the drone needs ~0.5 s to react, the EKF
            # already compensates the delay) and v <= kp d near the target
            d = math.hypot(ex, ey)
            v = min(self.v_max, self.kp * d, math.sqrt(2 * self.a_max * max(d - 0.5 * tol, 0.0)))
            vx, vy = (ex / d * v, ey / d * v) if d > 1e-3 else (0.0, 0.0)
            # integral near the target: cancels the drift (sim with 10 cm/s drift: a P-only approach stopped
            # ~20 cm short, "not reached in time" three times)
            if d < 0.4:
                integ[0] = clip(integ[0] + 0.4 * ex / RATE, 0.15)
                integ[1] = clip(integ[1] + 0.4 * ey / RATE, 0.15)
            vx, vy = vx + integ[0], vy + integ[1]
            c, s = math.cos(yaw), math.sin(yaw)
            self.send(c * vx + s * vy, -s * vx + c * vy, clip(self.kp * ez, self.vz_max),
                      clip(self.kp_yaw * eyaw, self.w_max))
            return None
        t0 = time.time()
        self.run(timeout, step, search_on_loss=False)  # goto handles the losses itself (turns, look_at)
        if time.time() - t0 >= timeout - 0.05:
            self.say("  (not reached in time, going on)")

    def turn(self, yaw_t):
        """Turn in place. A turn over 100 deg is split: first face the markers' centroid (the main wall), so the
        drone never spends long blind (a 180 deg turn could go round through the side without markers)."""
        _, x, y, z, yaw = self.pose
        cx, cy = np.mean([m.t[0] for m in self.markers.values()]), np.mean([m.t[1] for m in self.markers.values()])
        mid = math.atan2(cy - y, cx - x)
        lost = self.lost_max
        self.lost_max = max(lost, 7.0)  # the IMU heading carries the turn while the markers are out of view
        try:
            if abs(wrap(yaw_t - yaw)) > math.radians(100) and abs(wrap(mid - yaw)) < abs(wrap(yaw_t - yaw)):
                self.goto((x, y, z), mid, timeout=10.0, yaw_only=True)
            self.goto((x, y, z), yaw_t, timeout=12.0, yaw_only=True)
        finally:
            self.lost_max = lost

    def centre_point(self):
        """Middle of the flight zone, moved to the nearest spot with room all around (obstacle grid)."""
        f = self.fence
        cx, cy = (f[0] + f[1]) / 2, (f[2] + f[3]) / 2
        if self.zone and "grid" in self.zone:
            from tello_driver.driver import clearance
            best = None
            for dx in np.arange(-0.6, 0.61, 0.05):
                for dy in np.arange(-0.6, 0.61, 0.05):
                    c = clearance(self.zone, cx + dx, cy + dy)[0]
                    if c >= self.zone["margin"] + 0.2 and (best is None or math.hypot(dx, dy) < best[0]):
                        best = (math.hypot(dx, dy), cx + dx, cy + dy)
            if best:
                cx, cy = best[1], best[2]
        return cx, cy

    def go_sequence(self):
        """tello_goto 49 35: centre -> face 49 -> in front of 49 -> centre -> face 35 -> in front of 35 -> ..."""
        wps = {w["id"]: w for w in self.waypoints()}
        cx, cy = self.centre_point()
        zc = sum(self.band_z) / 2 if self.band_z else min(max(self.p0[2], self.z_lo), self.z_hi)
        _, x, y, z, yaw = self.pose
        self.say(f"Mission: {' -> '.join(map(str, self.sequence))}. Room centre ({cx:+.2f}, {cy:+.2f}, {zc:.2f}) m")
        self.say("Going to the room centre")
        self.goto((cx, cy, zc), yaw)
        n = len(self.sequence)
        for i, mid in enumerate(self.sequence):
            w, m = wps[mid], self.markers[mid]
            self.say(f"[{i + 1}/{n}] marker {mid}: turning to face it")
            self.turn(w["yaw"])
            d = math.hypot(w["near"][0] - m.t[0], w["near"][1] - m.t[1])
            self.say(f"[{i + 1}/{n}] marker {mid}: going in front of it ({d:.2f} m away)")
            self.goto(w["near"], w["yaw"], look_at=(m.t[0], m.t[1]), tol=0.10)
            self.run(self.hold, lambda: self.send())
            _, x, y, _, _ = self.pose
            self.say(f"[{i + 1}/{n}] marker {mid}: reached, {math.hypot(x - w['near'][0], y - w['near'][1]) * 100:.0f} cm "
                     "from the target point")
            self.say(f"[{i + 1}/{n}] back to the room centre")
            self.goto((cx, cy, zc), w["yaw"], look_at=(m.t[0], m.t[1]))
        self.return_home()
        self.say("Mission done: landing.")

    def main(self):
        self.prepare()
        if self.sequence:
            self.go_sequence()
            return
        f = self.fence
        self.say(f"Flight zone x [{f[0]:+.2f}, {f[1]:+.2f}]  y [{f[2]:+.2f}, {f[3]:+.2f}]  "
                 f"z [{self.z_lo:.1f}, {self.z_hi:.1f}] m. Max speed {self.v_max} m/s.")
        x0, y0, z0, yaw0 = self.p0
        if not (f[0] <= x0 <= f[1] and f[2] <= y0 <= f[3]):
            raise Abort(f"start point ({x0:+.2f}, {y0:+.2f}) outside the flight zone: start closer to the room centre")
        if self.survey:
            self.survey_turn()
            self.turn(yaw0)
        wps = self.waypoints()
        self.say("Tour: " + " -> ".join(str(w["id"]) for w in wps))
        centre = ((f[0] + f[1]) / 2, (f[2] + f[3]) / 2)
        wall = wps[0]["wall"]
        for i, w in enumerate(wps):
            if w["wall"] != wall:
                # another wall: turn from the room centre, where its markers are 1.5-2 m away and fully in view
                # (turning next to a wall left them cut at the image edges or out of the field of view)
                wall = w["wall"]
                _, x, y, z, yaw = self.pose
                self.say(f"[{i + 1}/{len(wps)}] next wall: back to the room centre ({centre[0]:+.2f}, {centre[1]:+.2f})")
                # ...at the height of the next marker: the west markers (2.5-2.7 m) are above the field of view
                # from 1.9 m (seen in the simulation)
                self.goto((centre[0], centre[1], w["p"][2]), yaw)
            self.say(f"[{i + 1}/{len(wps)}] marker {w['id']}: turning to face it")
            self.turn(w["yaw"])
            m = self.markers[w["id"]]
            self.say(f"[{i + 1}/{len(wps)}] marker {w['id']}: approach to {math.hypot(w['near'][0] - m.t[0], w['near'][1] - m.t[1]):.2f} m")
            self.goto(w["near"], w["yaw"], look_at=(m.t[0], m.t[1]), tol=0.10)
            self.run(self.hold, lambda: self.send())
            self.say(f"[{i + 1}/{len(wps)}] marker {w['id']}: back off to {math.hypot(w['p'][0] - m.t[0], w['p'][1] - m.t[1]):.2f} m")
            self.goto(w["p"], w["yaw"], look_at=(m.t[0], m.t[1]), tol=0.15)
        self.say("Back to the start point")
        self.goto((x0, y0, max(z0, self.z_lo)), yaw0)
        self.return_home()
        self.say("Tour done: landing.")


def main():
    run(MarkerTour)


if __name__ == "__main__":
    main()
