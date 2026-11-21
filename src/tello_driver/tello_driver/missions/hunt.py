"""Balloon hunt: the drone faces the 49/21/22 wall the whole time; when the external camera sees the RED balloon, it
flies straight through it, comes back just before it, then lands.

    tello.bat hunt                                            (Windows: Logitech detection + ROS + RViz)
    ros2 run tello_driver tello_hunt                          (WSL, "Tello commands" window; you stay next to it)
    ros2 run tello_driver tello_hunt --ros-args -p v_max:=1.0 -p repeat:=true -p diameter:=0.20

ANY KEY = stop and land. Ctrl+C = land. The driver's guards (zone, ceiling, wall, no camera) always apply.

Why one fixed heading (Nov 19, after two flights): the first version chose a heading per leg (the most markers in
view) and turned before each leg. On the real drone a turn made it sink 20 cm and it picked the 44/1/3/28 wall whose
markers sit at 2.0-2.5 m, above the camera view: it lost its position. Now:
1. FACE THE WALL. After takeoff ONE turn to `face_yaw_deg` (-90: the camera looks at the y = 0 wall, markers 49, 21,
   22, 12, 11, 9, 39, 47), holding the height, then the heading never changes: every move is a pure translation,
   the camera always sees the same markers, no blind turn.
2. PATROL at `patrol_z` (2.08 m) at `patrol_v`, a loop of 4 points of the flight zone where the camera sees at least
   `enough_markers` markers of that wall (computed from the map), between `y_near` and `y_far` from it.
3. TRIGGER. The red balloon (/balloons, external camera) must be fresh, stable for `confirm` s (spread < 15 cm),
   within reach (centre at most `z_reach` from the flight band, a free point close enough to touch it) and the blue
   balloon, if known, not within `blue_clear` of the line. Otherwise the patrol goes on and says why every 3 s.
4. DASH. Stop, re-read the balloon, one minimum-jerk trajectory (v_max, a_max) aimed `through` m BEYOND its centre
   (the cage touches it while still fast), same heading. Model feedforward per body axis (forward 0.66, left 0.83)
   + P on the EKF; if the pose is lost the reference freezes and the drone hovers (reported as "blind").
5. BACK BEFORE THE BALLOON: back along the same line to `before` m before the balloon centre (on the side it came
   from), hover `hold` s there (the external camera says whether the balloon moved = hit), then patrol again
   (repeat:=true) or fly back above the pad and land.
"""
import csv
import json
import math
import time

import numpy as np
from std_msgs.msg import String

from tello_driver.core import model as M
from tello_driver.core.zone import clearance
from tello_driver.missions.base import Abort, run, wrap
from tello_driver.missions.strike import Strike
from tello_driver.missions.trajectory import Quintic, check

CAGE_R = 0.10      # m: Tello cage radius


class Hunt(Strike):
    NODE_NAME = "tello_hunt"
    PREFIX = "hunt"

    def __init__(self):
        super().__init__()
        p = self.declare_parameter
        self.face = math.radians(p("face_yaw_deg", -90.0).value)  # heading kept all along: the 49/21/22 wall
        self.balloon_r = p("diameter", 0.20).value / 2.0          # contact when centres < CAGE_R + balloon_r
        self.patrol_v = p("patrol_v", 0.4).value
        self.patrol_timeout = p("patrol_timeout", 180.0).value  # s of patrol without a red balloon: land
        self.confirm = p("confirm", 1.0).value                   # s the red balloon must be stable
        self.through = p("through", 0.25).value                  # m aimed beyond the balloon centre
        self.before = p("before", 0.5).value                     # m before the balloon centre, after the hit
        self.hold = p("hold", 2.0).value                         # s hovering there
        self.z_reach = p("z_reach", 0.25).value                  # balloon centre this far from the band: reachable
        self.blue_clear = p("blue_clear", 0.5).value
        self.repeat = p("repeat", False).value
        self.patrol_z = min(p("patrol_z", 2.08).value, self.z_lim[1] - 0.05)   # height of marker 35
        self.y_near, self.y_far = p("y_near", 0.9).value, p("y_far", 2.4).value  # patrol band, m from the wall
        # vertical: only the markers well inside the image count (the ones near the top/bottom edge are often missed)
        self.hfov, self.vfov = math.radians(p("cam_hfov_deg", 60.0).value), math.radians(p("cam_vfov_deg", 30.0).value)
        self.see_max = p("see_max", 3.5).value                   # m: markers farther than that are not detected
        self.enough = p("enough_markers", 3).value
        self.hold_z = None                                       # height held during the turn
        self.K_body = (M.K_X, M.K_Y)
        self.red, self.blue = [], None                           # red: [(t, x, y, z)] fresh detections
        self.create_subscription(String, "/balloons", self.on_balloons, 10)
        self._why, self._why_t = "", 0.0
        self._ready_t, self._ready = 0.0, False
        self.hunts = []

    # ---------------------------------------------------------------- balloons
    def on_balloons(self, m):
        try:
            b = json.loads(m.data)
        except ValueError:
            return
        now = time.time()
        r = b.get("red")
        if r and not r.get("stale") and r.get("age", 9) < 0.5:
            self.red.append((now, r["x"], r["y"], r["z"]))
        self.red = [e for e in self.red if now - e[0] < self.confirm + 0.5]
        if b.get("blue"):
            self.blue = np.array([b["blue"]["x"], b["blue"]["y"], b["blue"]["z"]])

    def red_stable(self):
        """Median red position if it was seen steadily for `confirm` s and did not move, else None."""
        now = time.time()
        h = [e for e in self.red if now - e[0] < self.confirm]
        if len(h) < 0.7 * 10 * self.confirm or now - h[-1][0] > 0.3 or h[0][0] > now - 0.8 * self.confirm:
            return None
        P = np.array([e[1:] for e in h])
        c = np.median(P, axis=0)
        return c if np.max(np.linalg.norm(P - c, axis=1)) < 0.15 else None

    def red_now(self):
        h = [e for e in self.red if time.time() - e[0] < 0.5]
        return np.median(np.array([e[1:] for e in h]), axis=0) if len(h) >= 3 else None

    def wait_reason(self, text):
        if text != self._why or time.time() - self._why_t > 3.0:
            self.say(f"  red balloon: {text}")
            self._why, self._why_t = text, time.time()

    # ---------------------------------------------------------------- what the Tello camera sees
    def visible(self, p, yaw):
        """(number of markers the camera sees from p with heading yaw, distance of the nearest one)."""
        n, near = 0, 9.0
        for m in self.markers.values():
            v = m.t - p
            d = float(np.linalg.norm(v))
            if d < 0.2 or d > self.see_max:
                continue
            if abs(wrap(math.atan2(v[1], v[0]) - yaw)) > self.hfov / 2:
                continue
            if abs(math.atan2(v[2], math.hypot(v[0], v[1]))) > self.vfov / 2:
                continue
            if float(np.dot(m.R[:, 2], -v / d)) < 0.5:  # seen at more than 60 deg from its front: not detected
                continue
            n += 1
            near = min(near, d)
        return n, near

    # ---------------------------------------------------------------- planning
    def plan_dash(self, start, B):
        """Trajectory from start through the balloon centre B (to `through` m beyond, shortened until it is safe);
        (traj, end, direction) or (None, reason, None). The duration uses the drone's gain in the direction of
        the dash relative to the fixed heading (sideways 0.83 is faster than forward 0.66)."""
        zt = min(max(B[2], self.z_lim[0]), self.z_lim[1])
        if abs(B[2] - zt) > self.z_reach:
            return None, f"centre at {B[2]:.2f} m, the drone flies {self.z_lim[0]:.2f}-{self.z_lim[1]:.2f} m: " \
                         f"put it between {self.z_lim[0] - self.z_reach + 0.05:.2f} and {self.z_lim[1] + self.z_reach - 0.05:.2f} m", None
        u = np.array([B[0] - start[0], B[1] - start[1], 0.0])
        if np.linalg.norm(u) < 0.05:
            u = np.array([math.cos(self.face), math.sin(self.face), 0.0])
        u /= np.linalg.norm(u)
        Bt = np.array([B[0], B[1], zt])
        if self.blue is not None:
            a, b = start[:2], Bt[:2]
            s = np.clip(np.dot(self.blue[:2] - a, b - a) / max(np.dot(b - a, b - a), 1e-6), 0, 1)
            if np.linalg.norm(a + s * (b - a) - self.blue[:2]) < self.blue_clear:
                return None, "the blue balloon is on the way", None
        kw = self.qkw()
        phi = math.atan2(u[1], u[0]) - self.face  # direction of the dash in the body frame
        kw["K"] = 1.0 / math.hypot(math.cos(phi) / self.K_body[0], math.sin(phi) / self.K_body[1])
        for k in range(int((self.through + 0.25) / 0.05) + 1):
            beyond = self.through - 0.05 * k
            if beyond < -(CAGE_R + self.balloon_r - 0.04):
                break
            end = Bt + beyond * u
            traj = Quintic(start, end, self.v_max, self.a_max, **kw)
            if check(traj, self.zone, clearance, self.z_lim) is None:
                return traj, end, u
        return None, f"at ({B[0]:+.2f}, {B[1]:+.2f}): too close to an obstacle / outside the flight zone", None

    def qkw(self):
        return {"u_max": self.u_max, "tau": self.tau, "K": min(self.K_body)}

    # ---------------------------------------------------------------- patrol
    def patrol_points(self):
        """4 corners of the part of the flight zone between y_near and y_far from the wall, each pulled towards the
        middle until it has room and the camera (fixed heading) sees `enough_markers` markers from it."""
        f = self.zone
        z = self.patrol_z
        x0, x1 = f["x_min"], f["x_max"]
        y0, y1 = max(f["y_min"], self.y_near), min(f["y_max"], self.y_far)
        c = np.array([(x0 + x1) / 2, (y0 + y1) / 2])
        pts = []
        for cx, cy in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
            best = None
            for s in np.linspace(0.0, 1.0, 41):
                q = np.array([cx, cy]) + s * (c - np.array([cx, cy]))
                n, near = self.visible(np.array([q[0], q[1], z]), self.face)
                if self.roomy(q[0], q[1], 0.30) and n >= self.enough:
                    best = (q, n)
                    break
            if best is not None:
                pts.append((np.array([best[0][0], best[0][1], z]), best[1]))
        if not pts:  # the camera sees too few markers anywhere: patrol around the middle anyway
            pts = [(np.array([c[0], c[1], z]), self.visible(np.array([c[0], c[1], z]), self.face)[0])]
        return pts

    def red_ready(self):
        """Balloon stable and reachable? Evaluated at most 5 times a second (it plans a trajectory)."""
        if time.time() - self._ready_t < 0.2:
            return self._ready
        self._ready_t = time.time()
        self._ready = False
        B = self.red_stable()
        if B is not None:
            traj, why, _ = self.plan_dash(self.here(), B)
            if traj is None:
                self.wait_reason(why)
            else:
                self._ready = True
        return self._ready

    def face_wall(self):
        """The one turn of the flight: to the wall heading, holding the patrol height."""
        self.hold_z = self.patrol_z
        e = math.degrees(wrap(self.face - self.pose[4]))
        self.say(f"Turning {e:+.0f} deg to face the 49/21/22 wall (heading {math.degrees(self.face):.0f}), "
                 f"climbing to {self.patrol_z:.2f} m")
        self.turn_to(self.face, timeout=8.0)
        up = self.here().copy()
        up[2] = self.patrol_z
        self.track(Quintic(self.here(), up, 0.3, 0.6, **self.qkw()), self.face, "climb", settle=0.5, tol=0.08)

    def patrol_until_red(self):
        pts = self.patrol_points()
        self.say("Patrol (facing the wall, no turn): " +
                 " -> ".join(f"({p[0]:+.2f}, {p[1]:+.2f}) {n} markers" for p, n in pts) +
                 f" at {self.patrol_z:.2f} m, {self.patrol_v} m/s. Waiting for the red balloon (external camera).")
        t0, k = time.time(), 0
        while time.time() - t0 < self.patrol_timeout:
            if self.red_ready():
                return True
            dst = pts[k % len(pts)][0]
            self.track(Quintic(self.here(), dst, self.patrol_v, 0.6, **self.qkw()), self.face, "patrol",
                       settle=0.5, tol=0.15, stop=self.red_ready)
            if len(pts) == 1:
                self.run(0.5, lambda: self.send())
            k += 1
        self.say(f"No red balloon in {self.patrol_timeout:.0f} s.")
        return False

    def turn_to(self, yaw_t, timeout=6.0):
        """Turn in place HOLDING THE HEIGHT (Nov 19: a plain yaw command let the drone sink 1.79 -> 1.59 m) and
        slowly enough (0.6) for the markers to stay detected while the image sweeps."""
        def step():
            fresh = self.pose_age() <= 0.6
            yaw = self.pose[4] if fresh else self.yaw_estimate()
            if yaw is None:
                self.send()
                return None
            cz = 0.0
            if fresh and self.hold_z is not None:
                cz = max(-0.4, min(0.4, 1.5 * (self.hold_z - self.pose[3])))
            e = wrap(yaw_t - yaw)
            if abs(e) < math.radians(6):
                self.send(cz=cz)
                return False
            self.send(cz=cz, cw=max(-0.6, min(0.6, 1.2 * e)), blind=True)
            return None
        self.run(timeout, step, search_on_loss=False)

    # ---------------------------------------------------------------- attack
    def attack(self):
        """True when a dash was flown."""
        t_seen = time.time()
        B = self.red_stable()
        if B is None:
            return False
        self.run(0.2, lambda: self.send())  # stop the patrol leg
        B2 = self.red_now()                   # aim at where the balloon is NOW (if it moved a little)
        if B2 is not None and 0.03 < np.linalg.norm(B2 - B) < 0.4:
            B = B2
        start = self.here()
        traj, end, u = self.plan_dash(start, B)
        if traj is None:
            self.say(f"  cannot attack: {end}")
            return False
        self.say(f"RED BALLOON at ({B[0]:+.2f}, {B[1]:+.2f}, {B[2]:.2f}) m: dash {traj.d:.2f} m in {traj.T:.1f} s, "
                 f"same heading ({self.visible(start, self.face)[0]} markers in view)")
        t_dash = time.time()
        r = self.track(traj, self.face, "dash", settle=0.4, tol=0.15)
        closest = self.closest_to(B, t_dash)
        # back along the same line to `before` m before the balloon centre (the side it came from)
        zt = end[2]
        back = np.array([B[0], B[1], zt]) - self.before * u
        if not self.roomy(back[0], back[1], 0.0):
            back = start
        self.track(Quintic(self.here(), back, 0.5, 0.6, **self.qkw()), self.face, "back", settle=0.5, tol=0.12)
        self.run(self.hold, lambda: self.send())
        moved = None
        after = self.red_now()
        if after is not None:
            moved = float(np.linalg.norm(after - B))
        contact = CAGE_R + self.balloon_r
        hit = closest < contact + 0.03 or (moved is not None and moved > 0.10)
        res = {"t_react": t_dash - t_seen, "t_dash": r["elapsed"] if r else float("nan"),
               "closest": closest, "blind": r["frozen"] if r else float("nan"), "moved": moved,
               "rms": r["rms"] if r else float("nan"), "hit": hit, "B": B}
        self.hunts.append(res)
        self.say(f"  {'HIT' if hit else 'MISSED'}: closest approach {closest * 100:.0f} cm from the balloon centre "
                 f"(contact below ~{contact * 100:.0f} cm), dash {res['t_dash']:.1f} s, "
                 f"{res['t_react']:.1f} s from detection to dash, blind {res['blind']:.1f} s, "
                 f"balloon moved {'?' if moved is None else f'{moved * 100:.0f} cm'}; now {self.before:.1f} m before it")
        return True

    def closest_to(self, B, t0):
        self.log_f.flush()
        best = 9.0
        with open(self.out / "tracking.csv") as f:
            for r in csv.DictReader(f):
                if float(r["t"]) >= t0:
                    best = min(best, math.dist((float(r["x"]), float(r["y"]), float(r["z"])), B))
        return best

    # ---------------------------------------------------------------- mission
    def main(self):
        self.prepare()
        if not self.zone:
            raise Abort("no flight zone (config/flight_zone_band.yaml): the hunt needs the free space")
        self.face_wall()
        while self.patrol_until_red():
            if self.attack() and not self.repeat:   # an attack refused at the last moment: patrol on
                break
        self.summary()
        self.return_home()
        self.say("Done: landing.")

    def summary(self):
        if not self.hunts:
            return
        lines = ["hit,closest_cm,react_s,dash_s,blind_s,moved_cm,rms_cm,x,y,z"]
        for h in self.hunts:
            mv = "" if h["moved"] is None else f"{h['moved'] * 100:.0f}"
            lines.append(f"{int(h['hit'])},{h['closest'] * 100:.0f},{h['t_react']:.2f},{h['t_dash']:.2f},"
                         f"{h['blind']:.2f},{mv},{h['rms'] * 100:.0f},{h['B'][0]:.2f},{h['B'][1]:.2f},{h['B'][2]:.2f}")
        (self.out / "hunts.csv").write_text("\n".join(lines) + "\n")
        self.say("Hunts:\n  " + "\n  ".join(lines))


def main():
    run(Hunt)


if __name__ == "__main__":
    main()
