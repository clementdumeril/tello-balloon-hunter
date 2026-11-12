"""A reinforcement-learning policy flies the Tello to points and holds them (policy trained in rl/, see rl/README.md).

    ros2 launch tello_driver real.launch.py          (terminal 1)
    ros2 run tello_driver tello_rl_hold --ros-args -p actor:=<path>/actor.npz      (terminal 2; no teleop)

From the start point, the policy flies to each point of `targets` in turn (offsets in the map frame, metres:
"dx,dy,dz;dx,dy,dz;..." , default a 0.6 m square and back) and holds it for `hold_s` seconds. It sees the EKF error
to the point, velocity and drift and its own last 8 commands, exactly as in training (rl/tello_env.py), and
commands a map-frame velocity, rotated into the body frame; the heading is held by a proportional term.
Prints and saves per point the RMS error over the second half of the hold and the time to come within 10 cm.

SAFETY: ANY KEY = stop and land. Ctrl+C = land. Landing if no marker is seen for 15 s or on low battery. Points
outside the free space of the room scan are skipped. The driver's guards (zone, ceiling, wall ahead) always apply.
"""
import csv
import math
import os
import time
from pathlib import Path

import numpy as np
from geometry_msgs.msg import TwistStamped, Vector3Stamped

from tello_driver.step_test import RATE, Abort, StepTest, run, wrap

REPO = os.environ.get("TELLO_REPO", str(Path.home() / "tello_ws/src/tello-balloon-hunter"))
U_MAX = np.array([0.6, 0.6, 0.4])  # m/s for an action of 1, as in rl/tello_env.py
HISTORY = 8


class NumpyActor:
    """relu(W0 x + b0) -> relu(W1 h + b1) -> tanh(Wmu h + bmu): the deterministic actor exported by rl/train.py."""

    def __init__(self, path):
        w = np.load(path)
        self.layers = [(w[f"W{i}"], w[f"b{i}"]) for i in range(10) if f"W{i}" in w]
        self.mu = (w["Wmu"], w["bmu"])

    def __call__(self, obs):
        h = np.asarray(obs, dtype=np.float64)
        for W, b in self.layers:
            h = np.maximum(W @ h + b, 0.0)
        return np.tanh(self.mu[0] @ h + self.mu[1])


class RLHold(StepTest):
    NODE_NAME = "tello_rl_hold"
    PREFIX = "rl"
    MOVE_TO_ZONE_CENTRE = True
    POSE_TOPIC = "/ekf/pose"

    def __init__(self):
        super().__init__()
        p = self.declare_parameter
        self.actor = NumpyActor(p("actor", os.path.join(REPO, "rl", "actor.npz")).value)
        spec = p("targets", "0.6,0,0;0.6,0.6,0;0,0.6,0;0,0,0;0,0,0.3;0,0,0").value
        self.targets = [np.array([float(v) for v in t.split(",")]) for t in spec.split(";") if t.strip()]
        self.hold_s = p("hold_s", 8.0).value
        self.kp_yaw = p("kp_yaw", 0.8).value
        self.vel, self.vel_t, self.drift = np.zeros(3), 0.0, np.zeros(2)
        self.create_subscription(TwistStamped, "/ekf/velocity", self.on_vel, 10)
        self.create_subscription(Vector3Stamped, "/ekf/drift", self.on_drift, 10)
        self.history = np.zeros((HISTORY, 3))
        self.log_f = open(self.out / "tracking.csv", "w", newline="")
        self.log = csv.writer(self.log_f)
        self.log.writerow(["t", "target", "xd", "yd", "zd", "x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az"])
        self.results = []

    def on_vel(self, m):
        self.vel, self.vel_t = np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z]), time.time()

    def on_drift(self, m):
        self.drift = np.array([m.vector.x, m.vector.y])

    def observe(self, target):
        _, x, y, z, _ = self.pose
        e = target - np.array([x, y, z])
        v = self.vel if time.time() - self.vel_t < 0.3 else np.zeros(3)
        return np.concatenate([np.clip(e, -3, 3), v / 0.5, self.drift / 0.1, np.tanh(e / 0.1), self.history.ravel()])

    def fly_to(self, k, target, yaw0):
        errs, reach = [], None
        t0 = time.time()

        def step():
            nonlocal reach
            if self.pose_age() > 0.6:
                self.history = np.roll(self.history, 1, axis=0)
                self.history[0] = 0.0
                self.send()  # no fresh pose: hover (the driver's hold takes over)
                return None
            a = np.clip(self.actor(self.observe(target)), -1, 1)
            self.history = np.roll(self.history, 1, axis=0)
            self.history[0] = a
            ux, uy, uz = a * U_MAX
            _, x, y, z, yaw = self.pose
            c, s = math.cos(yaw), math.sin(yaw)
            wz = max(-0.5, min(0.5, self.kp_yaw * wrap(yaw0 - yaw)))
            self.send(c * ux + s * uy, -s * ux + c * uy, uz, wz)
            e = float(np.linalg.norm(target - np.array([x, y, z])))
            t = time.time() - t0
            errs.append((t, e))
            if e < 0.10 and reach is None:
                reach = t
            elif e >= 0.10:
                reach = None
            self.log.writerow([f"{time.time():.3f}", k, *(f"{v:.3f}" for v in target), f"{x:.3f}", f"{y:.3f}",
                               f"{z:.3f}", *(f"{v:.3f}" for v in self.vel), *(f"{v:.3f}" for v in a)])
            return None
        self.run(self.hold_s, step, search_on_loss=False)
        late = [e for t, e in errs if t > self.hold_s / 2]
        return {"rms": float(np.sqrt(np.mean(np.square(late)))) if late else float("nan"), "reach": reach}

    def main(self):
        self.prepare()
        _, x0, y0, z0, yaw0 = self.pose
        start = np.array([x0, y0, z0])
        from tello_driver.driver import clearance
        for k, offset in enumerate(self.targets):
            target = start + offset
            if self.zone and "grid" in self.zone:
                c = clearance(self.zone, target[0], target[1])
                if c is None or c[0] < self.zone["margin"]:
                    self.say(f"point {k} ({target[0]:+.2f}, {target[1]:+.2f}) outside the free space: skipped")
                    continue
            if self.band_z and not (self.band_z[0] - 0.05 <= target[2] <= self.band_z[1] + 0.05):
                self.say(f"point {k} at {target[2]:.2f} m outside the band: skipped")
                continue
            self.say(f"point {k}: ({target[0]:+.2f}, {target[1]:+.2f}, {target[2]:.2f})")
            r = self.fly_to(k, target, yaw0)
            self.results.append((k, r))
            reach = f"{r['reach']:.1f} s" if r["reach"] is not None else "not within 10 cm at the end"
            self.say(f"  RMS over the last {self.hold_s / 2:.0f} s: {r['rms'] * 100:.1f} cm, within 10 cm after {reach}")
        lines = ["point,rms_cm,reach_s"] + [f"{k},{r['rms'] * 100:.1f},{'' if r['reach'] is None else round(r['reach'], 2)}"
                                             for k, r in self.results]
        (self.out / "results.csv").write_text("\n".join(lines) + "\n")
        self.return_home()
        self.say("Done: landing.")

    def close(self):
        super().close()
        self.log_f.close()


def main():
    run(RLHold)


if __name__ == "__main__":
    main()
