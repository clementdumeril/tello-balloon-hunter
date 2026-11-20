"""Balloon hunt: patrol the zone; when the external camera sees the RED balloon, hit it fast, come back, land.

    ros2 launch tello_driver real.launch.py                  (WSL, terminal 1: starts tello_balloons)
    python tools/ext_camera.py detect --diameter 0.28        (Windows: the Logitech camera, does not move)
    ros2 run tello_driver tello_hunt                         (WSL, terminal 2; you stay next to the drone)
    ros2 run tello_driver tello_hunt --ros-args -p v_max:=1.0 -p repeat:=true

ANY KEY = stop and land. Ctrl+C = land. The driver's guards (zone, ceiling, wall, no camera) always apply.

1. PATROL. The drone flies a loop between 4 points of the flight zone at `patrol_v`. Before each leg it turns ONCE to
   the heading that keeps the most markers in view along the whole leg, and never turns while it moves (a turning
   camera blurs the markers): the localization stays as good as it gets.
2. TRIGGER. The red balloon (/balloons, external camera) must be fresh, stable for `confirm` s (spread < 15 cm:
   a balloon still being placed or swinging is not attacked), within reach (its centre at most `z_reach` from the
   flight band, a point of the free space close enough to touch it) and the blue balloon, if known, must not be
   within `blue_clear` of the dash line. Otherwise the patrol goes on and says why, every 3 s.
3. DASH. Stop, turn to the heading that sees the most markers ALL ALONG the dash (camera field of view, marker
   distance and facing), excluding headings where a marker is closer than 0.9 m (the driver's wall guard would
   brake the dash). Re-read the balloon position, then one minimum-jerk trajectory (v_max, a_max) aimed `through`
   metres BEYOND the balloon centre: the cage touches the balloon while still moving fast, not at the end of the
   braking. Model feedforward per body axis (forward K 0.66, left 0.83) + P on the EKF; if the pose is lost the
   reference freezes and the drone hovers instead of flying blind (reported as "blind").
4. BACK. Short stop, retreat `retreat` m back along the dash line (away from the string), back to the patrol, and
   either land (default) or hunt again (repeat:=true). Report: reaction time, dash time, closest approach to the
   balloon centre, blind time, and whether the external camera saw the balloon move (= hit).
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

CAGE_R, BALLOON_R = 0.10, 0.14     # m: contact when the centres are closer than ~0.24 m


class Hunt(Strike):
    NODE_NAME = "tello_hunt"
    PREFIX = "hunt"

    def __init__(self):
        super().__init__()
        p = self.declare_parameter
        self.patrol_v = p("patrol_v", 0.4).value
        self.patrol_timeout = p("patrol_timeout", 180.0).value  # s of patrol without a red balloon: land
        self.confirm = p("confirm", 1.0).value                   # s the red balloon must be stable
        self.through = p("through", 0.25).value                  # m aimed beyond the balloon centre
        self.retreat_m = p("retreat", 0.6).value
        self.z_reach = p("z_reach", 0.25).value                  # balloon centre this far from the band: reachable
        self.blue_clear = p("blue_clear", 0.5).value
        self.repeat = p("repeat", False).value
        self.hfov, self.vfov = math.radians(p("cam_hfov_deg", 60.0).value), math.radians(p("cam_vfov_deg", 44.0).value)
        self.see_max = p("see_max", 3.5).value                   # m: markers farther than that are not detected
        self.enough = p("enough_markers", 3).value               # markers in view all along a move: enough
        self.K_body = (M.K_X, M.K_Y)
        self.red, self.blue = [], None                           # red: [(t, x, y, z)] fresh detections
        self.create_subscription(String, "/balloons", self.on_balloons, 10)
        self._why, self._why_t = "", 0.0
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

    def best_heading(self, pts, prefer):
        """Heading (every 10 deg) maximising the markers in view at the WORST point of the path, then the total;
        never one where a marker gets closer than 0.9 m (wall guard). Among those seeing enough, the least turning."""
        cands = []
        for k in range(36):
            yaw = math.radians(10 * k) - math.pi
            seen = [self.visible(p, yaw) for p in pts]
            if min(s[1] for s in seen) < 0.9:
                continue
            cands.append((min(s[0] for s in seen), sum(s[0] for s in seen), yaw))
        if not cands:
            return prefer, 0
        # enough markers all along (>= `enough`, or the best possible if fewer): then the least turning wins,
        # a 110 deg turn costs ~2.5 s before the dash (seen in the simulation)
        need = min(max(c[0] for c in cands), self.enough)
        ok = [c for c in cands if c[0] >= need]
        c = min(ok, key=lambda c: (abs(wrap(c[2] - prefer)), -c[1]))
        return c[2], c[0]

    # ---------------------------------------------------------------- planning
    def plan_dash(self, start, B, yaw=None):
        """Trajectory from start through the balloon centre B (to `through` m beyond, shortened until it is safe);
        (traj, end, direction) or (None, reason, None). With the heading known, the duration uses the drone's gain
        in THAT direction (sideways 0.83 is faster than forward 0.66), else the slowest one."""
        zt = min(max(B[2], self.z_lim[0]), self.z_lim[1])
        if abs(B[2] - zt) > self.z_reach:
            return None, f"centre at {B[2]:.2f} m, the drone flies {self.z_lim[0]:.2f}-{self.z_lim[1]:.2f} m: " \
                         f"put it between {self.z_lim[0] - self.z_reach + 0.05:.2f} and {self.z_lim[1] + self.z_reach - 0.05:.2f} m", None
        u = np.array([B[0] - start[0], B[1] - start[1], 0.0])
        if np.linalg.norm(u) < 0.05:
            u = np.array([math.cos(self.pose[4]), math.sin(self.pose[4]), 0.0])
        u /= np.linalg.norm(u)
        Bt = np.array([B[0], B[1], zt])
        if self.blue is not None:
            a, b = start[:2], Bt[:2]
            s = np.clip(np.dot(self.blue[:2] - a, b - a) / max(np.dot(b - a, b - a), 1e-6), 0, 1)
            if np.linalg.norm(a + s * (b - a) - self.blue[:2]) < self.blue_clear:
                return None, "the blue balloon is on the way", None
        kw = self.qkw()
        if yaw is not None:
            phi = math.atan2(u[1], u[0]) - yaw  # direction of the dash in the body frame
            kw["K"] = 1.0 / math.hypot(math.cos(phi) / self.K_body[0], math.sin(phi) / self.K_body[1])
        for k in range(int((self.through + 0.25) / 0.05) + 1):
            beyond = self.through - 0.05 * k
            end = Bt + beyond * u
            if beyond < -(CAGE_R + BALLOON_R - 0.04):
                break
            traj = Quintic(start, end, self.v_max, self.a_max, **kw)
            if check(traj, self.zone, clearance, self.z_lim) is None:
                return traj, end, u
        return None, f"at ({B[0]:+.2f}, {B[1]:+.2f}): too close to an obstacle / outside the flight zone", None

    def qkw(self):
        return {"u_max": self.u_max, "tau": self.tau, "K": min(self.K_body)}

    def path_pts(self, traj, n=12):
        return [traj.at(traj.T * k / n)[0] for k in range(n + 1)]

    # ---------------------------------------------------------------- patrol
    def patrol_points(self):
        """4 points of the zone (its corners pulled towards the centre until they have room), in a loop."""
        f = self.zone
        z = min(max(self.z_fly, self.z_lim[0]), self.z_lim[1])
        c = np.array([(f["x_min"] + f["x_max"]) / 2, (f["y_min"] + f["y_max"]) / 2])
        pts = []
        for cx, cy in ((f["x_min"], f["y_min"]), (f["x_max"], f["y_min"]), (f["x_max"], f["y_max"]), (f["x_min"], f["y_max"])):
            q = np.array([cx, cy], float)
            for s in np.linspace(0.0, 1.0, 41):
                q = np.array([cx, cy]) + s * (c - np.array([cx, cy]))
                if self.roomy(q[0], q[1], 0.30):
                    break
            pts.append(np.array([q[0], q[1], z]))
        return pts

    def red_ready(self):
        B = self.red_stable()
        if B is None:
            return False
        traj, why, _ = self.plan_dash(self.here(), B)
        if traj is None:
            self.wait_reason(why)
            return False
        return True

    def patrol_until_red(self):
        pts = self.patrol_points()
        self.say("Patrol: " + " -> ".join(f"({p[0]:+.2f}, {p[1]:+.2f})" for p in pts) +
                 f" at {self.patrol_v} m/s. Waiting for the red balloon (external camera).")
        t0, k = time.time(), 0
        if self.red_ready():
            return True
        while time.time() - t0 < self.patrol_timeout:
            dst = pts[k % len(pts)]
            traj = Quintic(self.here(), dst, self.patrol_v, 0.6, **self.qkw())
            yaw, n = self.best_heading(self.path_pts(traj), self.pose[4])
            self.turn_to(yaw)
            if self.red_ready():
                return True
            self.track(Quintic(self.here(), dst, self.patrol_v, 0.6, **self.qkw()), yaw, "patrol",
                       settle=0.5, tol=0.15, stop=self.red_ready)
            if self.red_ready():
                return True
            k += 1
        self.say(f"No red balloon in {self.patrol_timeout:.0f} s.")
        return False

    # ---------------------------------------------------------------- attack
    def attack(self):
        t_seen = time.time()
        B = self.red_stable()
        self.run(0.2, lambda: self.send())  # stop the patrol leg
        start = self.here()
        traj, end, u = self.plan_dash(start, B)
        if traj is None:
            self.say(f"  cannot attack: {end}")
            return
        yaw, n = self.best_heading(self.path_pts(traj), self.pose[4])
        self.say(f"RED BALLOON at ({B[0]:+.2f}, {B[1]:+.2f}, {B[2]:.2f}) m: turn {math.degrees(wrap(yaw - self.pose[4])):+.0f} "
                 f"deg to heading {math.degrees(yaw):.0f} ({n} markers in view at the worst point of the dash)")
        self.turn_to(yaw)
        B2 = self.red_now()  # the turn took a second: aim at where the balloon is NOW (if it moved a little)
        if B2 is not None and 0.03 < np.linalg.norm(B2 - B) < 0.4:
            B = B2
        traj, end, u = self.plan_dash(self.here(), B, yaw)
        if traj is None:
            self.say(f"  cannot attack: {end}")
            return
        self.say(f"  dash {traj.d:.2f} m in {traj.T:.1f} s planned")
        t_dash = time.time()
        r = self.track(traj, yaw, "dash", settle=0.4, tol=0.15)
        closest = self.closest_to(B, t_dash)
        self.run(0.3, lambda: self.send())
        back = self.here() - self.retreat_m * u
        if not self.roomy(back[0], back[1], 0.0):
            back = self.here() - 0.3 * u
        self.track(Quintic(self.here(), back, 0.5, 0.6, **self.qkw()), yaw, "retreat", settle=0.5, tol=0.15)
        moved = None
        after = self.red_now()
        if after is not None:
            moved = float(np.linalg.norm(after - B))
        hit = closest < CAGE_R + BALLOON_R + 0.03 or (moved is not None and moved > 0.10)
        res = {"t_react": t_dash - t_seen, "t_dash": r["elapsed"] if r else float("nan"),
               "closest": closest, "blind": r["frozen"] if r else float("nan"), "moved": moved,
               "rms": r["rms"] if r else float("nan"), "hit": hit, "B": B}
        self.hunts.append(res)
        self.say(f"  {'HIT' if hit else 'MISSED'}: closest approach {closest * 100:.0f} cm from the balloon centre "
                 f"(contact below ~{(CAGE_R + BALLOON_R) * 100:.0f} cm), dash {res['t_dash']:.1f} s, "
                 f"{res['t_react']:.1f} s from detection to dash, blind {res['blind']:.1f} s, "
                 f"balloon moved {'?' if moved is None else f'{moved * 100:.0f} cm'}")

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
        while self.patrol_until_red():
            self.attack()
            if not self.repeat:
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
