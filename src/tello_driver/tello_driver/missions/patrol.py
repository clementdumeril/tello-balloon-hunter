"""Patrol: the drone flies laps around the flight zone while turning on itself (it looks at every wall in turn).
Type a marker number + ENTER: it stops, turns until its CAMERA sees that marker, flies in front of it, then goes
back to patrolling.

    ros2 launch tello_driver real.launch.py          (terminal 1)
    ros2 run tello_driver tello_patrol                (terminal 2, NO teleop running; you stay next to the drone)
    ros2 run tello_driver tello_patrol --ros-args -p speed:=0.3 -p yaw_rate:=10

Keys during the flight: a number then ENTER = go to that marker (e.g. 3 5 ENTER), BACKSPACE = erase,
+ / - = patrol speed +-0.05 m/s, * / / = turning speed +-5 deg/s, ENTER alone = print the lap statistics.
ANY OTHER KEY (space, q, Esc...) = stop and land. Ctrl+C = land.

Tuning: after each lap it prints the share of the time a marker was in view, the marker poses per second, the
distance to the patrol path and the EKF uncertainty. Raise the speeds while those stay good: the fastest
stable setting is the patrol of the demo. Everything is saved in ~/tello_ws/sessions/patrol_<time>/laps.csv.

The search uses the camera only (/aruco/ids): the drone turns until it sees the marker. Flying in front of it
then uses the marker map (the position of the marker on the wall), like the rest of the localisation.
"""
import csv
import math
import time

import numpy as np

from tello_driver.core import model as M
from tello_driver.missions.tour import MarkerTour, clip
from tello_driver.missions.base import Abort, run, wrap
from tello_driver.core.teleop import read_key


class Patrol(MarkerTour):
    NODE_NAME = "tello_patrol"
    PREFIX = "patrol"
    BAND_DEFAULT = True

    def __init__(self):
        super().__init__()
        from rcl_interfaces.msg import ParameterDescriptor

        def p(name, default):  # speed:=1 or speed:=1.0 both accepted
            return float(self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True)).value)
        self.v_patrol = p("speed", 0.25)                   # m/s along the patrol path
        # real flights (identification/tour sessions, 1 s windows): markers detected 92 % of the seconds below 5 deg/s,
        # 73 % at 10-15 deg/s, 50 % at 15-20, 18 % at 30-40 deg/s (motion blur); moving at up to 0.3 m/s costs nothing
        self.w_patrol = math.radians(p("yaw_rate", 10.0))  # deg/s parameter, rad/s here: turning while patrolling
        self.w_search = math.radians(p("search_yaw_rate", 12.0))
        # camera coverage along the path (marker map, 55.6 deg view): the higher and the further from the walls, the
        # fewer headings without any marker. Worst blind arc of a full turn: inset 0.15 m at 1.85 m: 124 deg;
        # inset 0.35 m at 2.0 m: 58 deg; at 2.2 m: 20 deg. So: high in the band, 0.35 m inside the zone.
        self.inset = p("inset", 0.35)                      # m inside the flight zone rectangle
        self.z_patrol = p("height", 0.0)                   # m; 0 = top of the band - 0.05
        self.see_frames = int(p("see_frames", 3))          # target marker in this many frames = found
        self.survey = False
        self.fence_tol = 1.0
        self.lost_max = max(self.lost_max, 15.0)
        self.buf, self.request = "", None
        self.seen = {}  # marker id -> list of (t, u, size_px) over the last second
        self.seen_any_t = 0.0
        from std_msgs.msg import String
        self.create_subscription(String, "/aruco/ids", self.on_ids, 20)
        self.f_lap = open(self.out / "laps.csv", "w", newline="")
        self.w_lap = csv.writer(self.f_lap)
        self.w_lap.writerow(["lap", "duration_s", "speed", "yaw_rate_deg", "marker_in_view_pct", "poses_per_s",
                             "path_err_rms_m", "path_err_max_m", "sigma_xy_max_m", "longest_blind_s"])

    # ---------------------------------------------------------------- inputs
    def on_ids(self, m):
        now = time.time()
        for item in filter(None, m.data.split(",")):
            i, u, s = item.split(":")
            self.seen.setdefault(int(i), []).append((now, float(u), float(s)))
            self.seen_any_t = now
        for k in list(self.seen):
            self.seen[k] = [e for e in self.seen[k] if now - e[0] < 1.0]

    def sees(self, mid, window=0.5):
        """(frames, mean horizontal image position) of marker mid in the last `window` s."""
        e = [x for x in self.seen.get(mid, []) if time.time() - x[0] < window]
        return len(e), (float(np.mean([x[1] for x in e])) if e else 0.0)

    def check_keys(self):
        while True:
            k = read_key(0)
            if k is None:
                return
            if k.isdigit():
                self.buf += k
                self.say(f"  marker: {self.buf}")
            elif k in ("\x7f", "\x08"):
                self.buf = self.buf[:-1]
                self.say(f"  marker: {self.buf}")
            elif k in ("\n", "\r"):
                if self.buf:
                    mid, self.buf = int(self.buf), ""
                    if mid in self.markers:
                        self.request = mid
                        self.say(f">> going to marker {mid}")
                    else:
                        self.say(f">> marker {mid} is not in the map ({sorted(self.markers)})")
                else:
                    self.print_lap(partial=True)
            elif k in ("+", "-"):
                self.v_patrol = min(0.8, max(0.05, self.v_patrol + (0.05 if k == "+" else -0.05)))
                self.say(f">> patrol speed {self.v_patrol:.2f} m/s")
            elif k in ("*", "/"):
                self.w_patrol = min(math.radians(90), max(0.0, self.w_patrol + math.radians(5 if k == "*" else -5)))
                self.say(f">> turning speed {math.degrees(self.w_patrol):.0f} deg/s")
            else:
                raise Abort("key pressed")

    # ---------------------------------------------------------------- patrol
    def path(self):
        f, d = self.fence, self.inset
        x0, x1, y0, y1 = f[0] + d, f[1] - d, f[2] + d, f[3] - d
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]

    def new_lap(self):
        self.lap = {"t0": time.time(), "steps": 0, "view": 0, "poses0": self.pose_count, "err": [], "sig": 0.0,
                    "blind": 0.0, "blind_t": None, "v": self.v_patrol, "w": math.degrees(self.w_patrol)}

    def print_lap(self, partial=False):
        L = getattr(self, "lap", None)
        if not L or L["steps"] < 20:
            return
        dur = time.time() - L["t0"]
        err = np.array(L["err"]) if L["err"] else np.zeros(1)
        row = [self.lap_n, f"{dur:.1f}", f"{L['v']:.2f}", f"{L['w']:.0f}", f"{100 * L['view'] / L['steps']:.0f}",
               f"{(self.pose_count - L['poses0']) / dur:.1f}", f"{np.sqrt(np.mean(err ** 2)):.3f}",
               f"{err.max():.3f}", f"{L['sig']:.3f}", f"{L['blind']:.1f}"]
        self.say(f"{'  (so far) ' if partial else ''}lap {self.lap_n}: {dur:.0f} s at {L['v']:.2f} m/s, "
                 f"{L['w']:.0f} deg/s | marker in view {row[4]} % of the time, {row[5]} poses/s, longest blind "
                 f"{row[9]} s | path error rms {float(row[6]) * 100:.0f} cm, max {float(row[7]) * 100:.0f} cm | "
                 f"EKF sigma max {float(row[8]) * 100:.0f} cm")
        if not partial:
            self.w_lap.writerow(row)
            self.f_lap.flush()

    def on_pose(self, m):
        super().on_pose(m)
        self.sigma_xy = math.sqrt(max(m.pose.covariance[0], m.pose.covariance[7], 0.0))

    def patrol(self):
        """Laps around the zone while turning; returns when a marker is requested."""
        wps = self.path()
        _, x, y, _, _ = self.pose
        k = min(range(4), key=lambda i: math.hypot(wps[i][0] - x, wps[i][1] - y))
        z_t = self.z_patrol or (self.band_z[1] - 0.05 if self.band_z else self.p0[2])
        if self.zone and "grid" in self.zone:  # 3D zone: the path must be free at that height (L shape)
            from tello_driver.core.zone import clearance
            for a, b in wps:
                if clearance(self.zone, a, b, z_t)[0] < self.zone["margin"] + 0.1:
                    raise Abort(f"patrol corner ({a:+.2f}, {b:+.2f}) not in the flight zone at {z_t:.2f} m "
                                "(over the east furniture: height >= 2.15 m, or a larger inset)")
        self.say(f"Patrol: {self.v_patrol:.2f} m/s, turning {math.degrees(self.w_patrol):.0f} deg/s, path "
                 + " -> ".join(f"({a:+.2f}, {b:+.2f})" for a, b in wps) + f" at {z_t:.2f} m. "
                 "Type a marker number + ENTER.")
        state = {"k": k, "start_k": k}
        self.new_lap()

        def step():
            if self.request is not None:
                return False
            L = self.lap
            L["steps"] += 1
            in_view = self.marker_age() < 0.25
            L["view"] += in_view
            if in_view:
                L["blind_t"] = None
            else:
                L["blind_t"] = L["blind_t"] or time.time()
                L["blind"] = max(L["blind"], time.time() - L["blind_t"])
            L["sig"] = max(L["sig"], getattr(self, "sigma_xy", 0.0))
            w = self.w_patrol
            if self.pose_age() > 0.5 or self.marker_age() > 0.7:
                self.send(cw=w, blind=True)  # no translation without a marker pose; turning finds them
                return None
            _, x, y, z, yaw = self.pose
            # distance to the current leg of the path (tuning metric)
            a, b = np.array(wps[state["k"] - 1]), np.array(wps[state["k"]])
            ab = b - a
            s = clip(float(np.dot(np.array([x, y]) - a, ab) / max(np.dot(ab, ab), 1e-6)), 1.0)
            L["err"].append(float(np.linalg.norm(a + max(0.0, s) * ab - np.array([x, y]))))
            tx, ty = wps[state["k"]]
            ex, ey = tx - x, ty - y
            d = math.hypot(ex, ey)
            if d < 0.15:  # corner reached: next one (no stop)
                state["k"] = (state["k"] + 1) % 4
                if state["k"] == state["start_k"]:
                    self.print_lap()
                    self.lap_n += 1
                    self.new_lap()
                return None
            v = min(self.v_patrol, math.sqrt(2 * self.a_max * d))  # slows down before the corner
            vx, vy = ex / d * v, ey / d * v
            if time.time() - self.drift_t < 0.5:
                vx, vy = vx - self.drift[0], vy - self.drift[1]
            # the drone turns while it moves: the command acts L_U later, when the nose has turned by w * L_U
            yp = yaw + w * M.L_U
            c, sn = math.cos(yp), math.sin(yp)
            self.send((c * vx + sn * vy) / M.K_X, (-sn * vx + c * vy) / M.K_Y, clip(self.kp * (z_t - z), self.vz_max), w)
            return None
        self.run(1e9, step, search_on_loss=False)

    # ---------------------------------------------------------------- find a marker
    def find(self, mid):
        """Turn on the spot until the camera sees marker mid (no map), centre it, then fly in front of it."""
        self.say(f"[{mid}] looking for it with the camera (turning on the spot)")
        t0, turned, last = time.time(), 0.0, self.yaw_estimate()
        direction = 1.0 if self.w_patrol >= 0 else -1.0

        def search():
            nonlocal turned, last
            n, u = self.sees(mid)
            if n >= self.see_frames:
                return False
            yaw = self.yaw_estimate()
            if yaw is not None and last is not None:
                turned += abs(wrap(yaw - last))
            last = yaw
            if turned > 4 * math.pi:
                raise Abort(f"marker {mid} not seen after two full turns")
            self.send(cw=direction * self.w_search, blind=True)
            return None
        self.run(60.0, search, search_on_loss=False)
        n, u = self.sees(mid)
        self.say(f"[{mid}] seen after {time.time() - t0:.1f} s ({math.degrees(turned):.0f} deg of turn), "
                 f"{'left' if u < 0 else 'right'} of the image centre: facing it")

        def centre():
            n, u = self.sees(mid, window=0.3)
            if n and abs(u) < 0.1:
                self.send()
                return False
            self.send(cw=clip(-1.2 * u, 0.5) if n else direction * 0.3, blind=True)  # u > 0: right -> turn right
            return None
        self.run(8.0, centre, search_on_loss=False)
        w = next(w for w in self.waypoints() if w["id"] == mid)
        m = self.markers[mid]
        self.say(f"[{mid}] flying in front of it ({math.hypot(w['near'][0] - m.t[0], w['near'][1] - m.t[1]):.2f} m)")
        self.goto(w["near"], w["yaw"], look_at=(m.t[0], m.t[1]), tol=0.10, face=(m.t[0], m.t[1]))
        self.run(self.hold, lambda: self.send())
        _, x, y, _, _ = self.pose
        self.say(f"[{mid}] reached in {time.time() - t0:.1f} s, {math.hypot(x - w['near'][0], y - w['near'][1]) * 100:.0f} cm "
                 "from the point in front of it. Back to patrolling.")

    def main(self):
        self.pose_count = 0
        self.lap_n = 1
        self.prepare()
        f = self.fence
        self.say(f"Flight zone x [{f[0]:+.2f}, {f[1]:+.2f}]  y [{f[2]:+.2f}, {f[3]:+.2f}]")
        while True:
            self.patrol()
            mid, self.request = self.request, None
            self.print_lap(partial=True)
            self.find(mid)

    def close(self):
        self.f_lap.close()
        super().close()


def _count_marker_poses(node):
    """Marker poses per second (ArUco, not the EKF), for the lap statistics."""
    from geometry_msgs.msg import PoseWithCovarianceStamped

    def cb(m):
        node.pose_count += 1
        node.marker_t = time.time()
    node.create_subscription(PoseWithCovarianceStamped, "/aruco/pose", cb, 20)


class PatrolNode(Patrol):
    def __init__(self):
        super().__init__()
        self.pose_count = 0
        _count_marker_poses(self)


def main():
    run(PatrolNode)


if __name__ == "__main__":
    main()
