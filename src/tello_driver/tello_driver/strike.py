"""Fast moves with planned trajectories: the balloon attack, rehearsed on markers.

    ros2 launch tello_driver real.launch.py          (terminal 1)
    ros2 run tello_driver tello_strike                (terminal 2; no teleop; you stay next to the drone)
    ros2 run tello_driver tello_strike --ros-args -p mode:=patrol -p v_max:=0.6

Patrol: quick left-right legs across the safe zone, facing the marker wall (the patroller of the final demo).
Strike: for each target marker (stand-in for a balloon): face it, dash to `strike_distance` in front of it,
stop, retreat. Each move is a minimum-jerk trajectory (quintic) planned with v_max / a_max and checked against
the free space of the room scan BEFORE flying it. It is tracked with the identified model as feedforward
(u = (v_d + tau a_d) / K, taken L_u ahead to cover the command delay) plus feedback on the EKF position.
Prints and saves the tracking error, the time to target and the closest approach.

SAFETY: ANY KEY = stop and land. Ctrl+C = land. Landing if the drone leaves the free space, if no marker is seen
for 15 s, or on low battery. The driver's guards (zone, ceiling, wall ahead from the marker size) always apply.
"""
import csv
import math
import os
import time
from pathlib import Path

import numpy as np

from aruco_localization.marker_map import load_marker_map
from tello_driver.zone import clearance
from tello_driver.step_test import RATE, Abort, StepTest, run, wrap
from tello_driver.trajectory import Quintic, check

REPO = os.environ.get("TELLO_REPO", str(Path.home() / "tello_ws/src/tello-balloon-hunter"))


class Strike(StepTest):
    NODE_NAME = "tello_strike"
    PREFIX = "strike"
    MOVE_TO_ZONE_CENTRE = True
    POSE_TOPIC = "/ekf/pose"

    def __init__(self):
        super().__init__()
        p = self.declare_parameter
        self.mode = [m.strip() for m in p("mode", "patrol,strike").value.split(",") if m.strip()]
        self.targets = [int(v) for v in p("targets", "6,10,5").value.split(",") if v.strip()]
        self.legs = int(p("legs", 4).value)
        self.v_max, self.a_max = p("v_max", 0.8).value, p("a_max", 1.0).value
        self.u_max = p("u_max", 0.9).value  # command limit used by the planner (driver max_rc 80 ~ 1.0 m/s)
        self.kp, self.kp_z, self.kp_yaw = p("kp", 0.8).value, p("kp_z", 0.8).value, p("kp_yaw", 1.0).value
        from tello_driver import model as M  # identified model (model.py); one K: the mean of forward and left
        self.K, self.tau, self.L_u = p("model_K", M.K_MEAN).value, p("model_tau", M.TAU_XY).value, p("model_L_u", M.L_U).value
        self.ki = p("ki", 0.3).value  # integral: cancels the drift (the Tello's own hold does not work here)
        self.integ = np.zeros(2)
        self.strike_d, self.retreat_d = p("strike_distance", 0.9).value, p("retreat_distance", 1.6).value
        self.z_lim = self.band_z or (1.2, 2.1)
        # flight height: the middle of the band in band mode (0 = auto)
        self.z_fly = p("height", 0.0).value or (sum(self.band_z) / 2 if self.band_z else 2.0)
        _, self.markers = load_marker_map(p("markers_file", os.path.join(REPO, "config", "markers.yaml")).value)
        self.lost_max = p("strike_lost_timeout", 15.0).value
        self.box = 99.0  # no box around the start point: the free space of the room scan is the limit
        self.log_f = open(self.out / "tracking.csv", "w", newline="")
        self.log = csv.writer(self.log_f)
        self.log.writerow(["t", "phase", "xd", "yd", "zd", "x", "y", "z", "vxd", "vyd"])
        self.results = []

    # ---------------------------------------------------------------- tracking
    def track(self, traj, yaw_t, phase, settle=2.0, tol=0.12):
        """Follow the trajectory: feedforward from the model + P on the EKF position. The reference is frozen
        while the pose is not trusted (markers lost), and the drone hovers."""
        problem = check(traj, self.zone, clearance, self.z_lim)
        if problem:
            self.say(f"  {phase}: trajectory refused ({problem}), skipped")
            return None
        t0 = time.time()
        frozen = 0.0
        errs = []
        state = {"done": False}

        def step():
            nonlocal frozen
            now = time.time()
            if self.pose_age() > 0.6:
                frozen += 1.0 / RATE  # pause the reference until the markers come back
                self.send()
                return None
            t = now - t0 - frozen
            pd, vd, _ = traj.at(t)
            _, vff, aff = traj.at(t + self.L_u)
            _, x, y, z, yaw = self.pose
            ex, ey, ez = pd[0] - x, pd[1] - y, pd[2] - z
            if abs(ex) < 0.3 and abs(ey) < 0.3:
                self.integ = np.clip(self.integ + self.ki * np.array([ex, ey]) / RATE, -0.15, 0.15)
            ux = (vff[0] + self.tau * aff[0]) / self.K + self.kp * ex + self.integ[0]
            uy = (vff[1] + self.tau * aff[1]) / self.K + self.kp * ey + self.integ[1]
            n = math.hypot(ux, uy)
            if n > 1.3 * self.v_max:
                ux, uy = ux * 1.3 * self.v_max / n, uy * 1.3 * self.v_max / n
            uz = max(-0.4, min(0.4, vff[2] + self.kp_z * ez))
            wz = max(-0.8, min(0.8, self.kp_yaw * wrap(yaw_t - yaw)))
            c, s = math.cos(yaw), math.sin(yaw)
            self.send(c * ux + s * uy, -s * ux + c * uy, uz, wz)
            e = math.hypot(ex, ey)
            errs.append(e)
            self.log.writerow([f"{now:.3f}", phase, *(f"{v:.3f}" for v in pd), f"{x:.3f}", f"{y:.3f}", f"{z:.3f}",
                               f"{vd[0]:.3f}", f"{vd[1]:.3f}"])
            if t > traj.T and (e < tol or t > traj.T + settle):
                state["done"] = True
                return False
            return None
        self.run(traj.T + settle + 20.0, step, search_on_loss=False)
        if not errs:
            return None
        return {"T": traj.T, "rms": float(np.sqrt(np.mean(np.square(errs)))), "max": float(np.max(errs)),
                "dist": traj.d, "elapsed": time.time() - t0}

    def turn_to(self, yaw_t, timeout=6.0):
        def step():
            if self.pose_age() > 0.6:
                yaw = self.yaw_estimate()
                if yaw is None:
                    self.send()
                    return None
            else:
                yaw = self.pose[4]
            e = wrap(yaw_t - yaw)
            if abs(e) < math.radians(6):
                self.send()
                return False
            self.send(cw=max(-0.9, min(0.9, 1.2 * e)), blind=True)
            return None
        self.run(timeout, step, search_on_loss=False)

    def qkw(self):
        return {"u_max": self.u_max, "tau": self.tau, "K": self.K}

    def here(self):
        """Current position, height brought back inside the limits (a small overshoot above 2.1 m must not make
        every next trajectory 'unsafe')."""
        _, x, y, z, _ = self.pose
        return np.array([x, y, min(max(z, self.z_lim[0]), self.z_lim[1])])

    def roomy(self, x, y, extra):
        return not self.zone or "grid" not in self.zone or clearance(self.zone, x, y)[0] >= self.zone["margin"] + extra

    # ---------------------------------------------------------------- missions
    def patrol(self):
        f = self.zone
        yc = (f["y_min"] + f["y_max"]) / 2
        z = min(max(self.z_fly, self.z_lim[0]), self.z_lim[1])
        # leg ends with room around them (margin + 35 cm): at 0.8 m/s the driver's guard, which looks ~1 s ahead,
        # would otherwise brake before the end of every leg (seen in the simulation)
        xc = (f["x_min"] + f["x_max"]) / 2
        xa = next((x for x in np.arange(xc, f["x_min"] - 0.5, -0.05) if not self.roomy(x, yc, 0.35)), f["x_min"]) + 0.05
        xb = next((x for x in np.arange(xc, f["x_max"] + 0.5, 0.05) if not self.roomy(x, yc, 0.35)), f["x_max"]) - 0.05
        a = np.array([xa, yc, z])
        b = np.array([xb, yc, z])
        yaw = self.p0[3]
        self.say(f"Patrol: {self.legs} legs between x {a[0]:+.2f} and {b[0]:+.2f} m (y {yc:+.2f}), "
                 f"v_max {self.v_max} m/s, a_max {self.a_max} m/s2")
        self.track(Quintic(self.here(), a, 0.4, 0.5, **self.qkw()), yaw, "patrol-start")
        for k in range(self.legs):
            src, dst = (a, b) if k % 2 == 0 else (b, a)
            r = self.track(Quintic(self.here(), dst, self.v_max, self.a_max, **self.qkw()), yaw, f"patrol-{k + 1}")
            if r:
                self.results.append(("patrol", k + 1, r))
                self.say(f"  leg {k + 1}: {r['dist']:.2f} m in {r['T']:.1f} s planned ({r['elapsed']:.1f} s real), "
                         f"tracking error rms {r['rms'] * 100:.0f} cm, max {r['max'] * 100:.0f} cm")

    def point_in_front(self, m, d):
        n = m.R[:, 2].copy()
        n[2] = 0.0
        n /= np.linalg.norm(n)
        z = min(max(m.t[2], self.z_lim[0]), self.z_lim[1])
        for k in range(40):
            q = np.array([m.t[0] + (d + 0.05 * k) * n[0], m.t[1] + (d + 0.05 * k) * n[1], z])
            if self.roomy(q[0], q[1], 0.15):
                return q, n
        return None, n

    def strike(self):
        home = self.here()
        for mid in self.targets:
            m = self.markers.get(mid)
            if m is None:
                self.say(f"marker {mid} not in the map, skipped")
                continue
            hit, n = self.point_in_front(m, self.strike_d)
            back, _ = self.point_in_front(m, self.retreat_d)
            if hit is None or back is None:
                self.say(f"marker {mid}: no safe strike point, skipped")
                continue
            yaw = math.atan2(-n[1], -n[0])
            self.say(f"Strike marker {mid}: get ready at {self.retreat_d:.1f} m, dash to "
                     f"{np.linalg.norm(hit[:2] - m.t[:2]):.2f} m, retreat")
            self.track(Quintic(self.here(), back, 0.5, 0.6, **self.qkw()), yaw, f"ready-{mid}")
            self.turn_to(yaw)
            t_dash = time.time()
            r = self.track(Quintic(self.here(), hit, self.v_max, self.a_max, **self.qkw()), yaw, f"dash-{mid}", settle=1.0, tol=0.10)
            closest = self.closest_since(t_dash, m, n)
            self.run(0.3, lambda: self.send())
            r2 = self.track(Quintic(self.here(), back, self.v_max, self.a_max, **self.qkw()), yaw, f"retreat-{mid}")
            if r:
                r["closest"] = closest
                r["overshoot"] = max(0.0, (np.linalg.norm(hit[:2] - m.t[:2])) - closest)
                self.results.append(("strike", mid, r))
                self.say(f"  dash {r['dist']:.2f} m in {r['elapsed']:.1f} s, closest approach {closest:.2f} m "
                         f"(target {np.linalg.norm(hit[:2] - m.t[:2]):.2f}, overshoot {r['overshoot'] * 100:.0f} cm), "
                         f"tracking rms {r['rms'] * 100:.0f} cm")
            if r2 is None:
                break
        self.track(Quintic(self.here(), home, 0.5, 0.6, **self.qkw()), self.p0[3], "home")

    def closest_since(self, t0, m, n):
        """Closest horizontal distance to the marker plane since t0 (from the tracking log of this flight)."""
        self.log_f.flush()
        best = 9.0
        with open(self.out / "tracking.csv") as f:
            for r in csv.DictReader(f):
                if float(r["t"]) >= t0:
                    d = (float(r["x"]) - m.t[0]) * n[0] + (float(r["y"]) - m.t[1]) * n[1]
                    best = min(best, d)
        return best

    def main(self):
        self.prepare()
        if not self.zone:
            raise Abort("no flight zone (config/flight_zone.yaml): strike missions need the free space")
        if "patrol" in self.mode:
            self.patrol()
        if "strike" in self.mode:
            self.strike()
        self.summary()
        self.return_home()
        self.say("Done: landing.")

    def summary(self):
        if not self.results:
            return
        lines = ["kind,id,distance_m,planned_s,real_s,rms_cm,max_cm,closest_m,overshoot_cm"]
        for kind, k, r in self.results:
            lines.append(f"{kind},{k},{r['dist']:.2f},{r['T']:.2f},{r['elapsed']:.2f},{r['rms'] * 100:.1f},"
                         f"{r['max'] * 100:.1f},{r.get('closest', float('nan')):.2f},{r.get('overshoot', float('nan')) * 100:.1f}")
        (self.out / "results.csv").write_text("\n".join(lines) + "\n")
        self.say("Results:\n  " + "\n  ".join(lines))

    def close(self):
        super().close()
        self.log_f.close()


def main():
    run(Strike)


if __name__ == "__main__":
    main()
