"""Identification flight: small velocity doublets on each axis, recorded with the ArUco pose, then fitted.

    ros2 launch tello_driver real.launch.py          (terminal 1, drone 2 m from the marker wall, markers in view)
    ros2 run tello_driver tello_step_test             (terminal 2, NO teleop running; you stay next to the drone)

Each test: hover 2 s, +A for T, -A for T (a doublet: the drone comes back), hover 3 s. Axes: forward,
left, up, yaw; 2 doublets each (opposite signs). Between tests the drone re-centres on its start point.
About 1 min 30 of flight. Then it lands and runs tello_step_fit (K, tau, delay per axis -> model.yaml).

SAFETY: ANY KEY = stop and land. Ctrl+C = land. Automatic landing if the drone leaves a box around its start
point (default +-0.8 m horizontally, 0.4-2.4 m high), if no marker is seen for 3 s, or on low battery.
It refuses to start if another node (e.g. the teleop) also sends velocity commands.
"""
import csv
import json
import math
import os
import sys
import termios
import time
import tty
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger

from tello_driver.teleop import read_key

RATE = 20.0


class Abort(Exception):
    pass


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class StepTest(Node):
    NODE_NAME = "tello_step_test"
    PREFIX = "step"

    def __init__(self):
        super().__init__(self.NODE_NAME)
        p = self.declare_parameter
        self.amp = p("amplitude", 0.3).value            # m/s
        self.dur = p("duration", 1.2).value             # s per half of the doublet
        self.yaw_amp = p("yaw_amplitude", 0.5).value    # rad/s
        self.yaw_dur = p("yaw_duration", 1.0).value
        self.repeats = int(p("repeats", 2).value)
        self.axes = [a.strip() for a in p("axes", "x,y,z,yaw").value.split(",") if a.strip()]
        self.box = p("box", 0.8).value                  # m, horizontal half-size around the start point
        self.z_min, self.z_max = p("z_min", 0.4).value, p("z_max", 2.4).value
        self.lost_max = p("lost_timeout", 3.0).value
        self.min_battery = p("min_battery", 30).value
        out = Path(p("out_dir", str(Path.home() / "tello_ws/sessions")).value) / datetime.now().strftime(f"{self.PREFIX}_%Y%m%d_%H%M%S")
        out.mkdir(parents=True, exist_ok=True)
        self.out = out

        self.pub = self.create_publisher(Twist, "/tello/cmd_vel_safe", 10)
        self.cli = {n: self.create_client(Trigger, f"/tello_driver/{n}") for n in ("takeoff", "land")}
        self.create_subscription(PoseWithCovarianceStamped, "/aruco/pose", self.on_pose, 50)
        self.create_subscription(String, "/tello/telemetry", self.on_tel, 10)
        self.create_subscription(CameraInfo, "/tello/camera_info", self.on_video, 2)
        self.video_t = 0.0
        self.pose = None          # (t_received, x, y, z, yaw)
        self.tel, self.tel_t = {}, 0.0
        self.p0 = None
        self.t_start = time.time()
        self.f_pose = open(out / "poses.csv", "w", newline="")
        self.w_pose = csv.writer(self.f_pose)
        self.w_pose.writerow(["t", "x", "y", "z", "yaw", "t_image"])
        self.f_cmd = open(out / "commands.csv", "w", newline="")
        self.w_cmd = csv.writer(self.f_cmd)
        self.w_cmd.writerow(["t", "segment", "axis", "u", "cx", "cy", "cz", "cw"])
        self.f_tel = open(out / "telemetry.csv", "w", newline="")
        self.w_tel = None

    # ---------------------------------------------------------------- inputs
    def on_pose(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        now = time.time()
        self.pose = (now, p.x, p.y, p.z, yaw)
        t_img = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.w_pose.writerow([f"{now:.4f}", f"{p.x:.4f}", f"{p.y:.4f}", f"{p.z:.4f}", f"{yaw:.5f}", f"{t_img:.4f}"])

    def on_tel(self, m):
        try:
            st = json.loads(m.data)
        except ValueError:
            return
        self.tel, self.tel_t = st, time.time()
        st = dict(st, t=round(self.tel_t, 4))
        if self.w_tel is None:
            self.w_tel = csv.DictWriter(self.f_tel, fieldnames=list(st.keys()), extrasaction="ignore")
            self.w_tel.writeheader()
        self.w_tel.writerow(st)

    def on_video(self, m):
        self.video_t = time.time()  # one camera_info per video frame

    def in_air(self):
        h = self.tel.get("h", 0)
        return bool(self.tel.get("flying")) or (isinstance(h, (int, float)) and h >= 20)

    def pose_age(self):
        return time.time() - self.pose[0] if self.pose else float("inf")

    # ---------------------------------------------------------------- helpers
    def say(self, text):
        print(f"\r{text}\r", flush=True)

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def call(self, name, timeout=5.0):
        c = self.cli[name]
        if not c.wait_for_service(timeout_sec=2.0):
            raise Abort(f"/tello_driver/{name} not available: is the driver running?")
        fut = c.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def check_keys(self):
        k = read_key(0)
        if k is not None:
            raise Abort("key pressed")

    def safety(self):
        self.check_keys()
        bat = self.tel.get("bat")
        if isinstance(bat, (int, float)) and bat < 20:
            raise Abort(f"battery {bat} %")
        if self.pose and self.p0 is not None and self.pose_age() < 0.5:
            _, x, y, z, _ = self.pose
            f = getattr(self, "fence", None)  # (xmin, xmax, ymin, ymax) for missions; else a box around the start
            if f is not None:
                tol = 0.3
                if not (f[0] - tol <= x <= f[1] + tol and f[2] - tol <= y <= f[3] + tol):
                    raise Abort(f"left the flight zone at ({x:+.2f}, {y:+.2f}) m")
            elif abs(x - self.p0[0]) > self.box or abs(y - self.p0[1]) > self.box:
                raise Abort(f"left the safety box ({x - self.p0[0]:+.2f}, {y - self.p0[1]:+.2f} m from the start)")
            if not self.z_min <= z <= self.z_max:
                raise Abort(f"height {z:.2f} m outside [{self.z_min}, {self.z_max}]")
        if self.pose_age() > self.lost_max:
            raise Abort(f"no marker seen for {self.pose_age():.1f} s")

    def send(self, cx=0.0, cy=0.0, cz=0.0, cw=0.0, segment=-1, axis="", u=0.0):
        t = Twist()
        # no fresh pose: hover instead of continuing a test blind
        if self.pose_age() > 0.7:
            cx = cy = cz = cw = u = 0.0
        t.linear.x, t.linear.y, t.linear.z, t.angular.z = cx, cy, cz, cw
        self.pub.publish(t)
        self.w_cmd.writerow([f"{time.time():.4f}", segment, axis, f"{u:.4f}",
                             f"{cx:.3f}", f"{cy:.3f}", f"{cz:.3f}", f"{cw:.3f}"])

    def run(self, seconds, fn):
        """Call fn() at RATE for `seconds` (fn sends the command), with the safety checks."""
        end = time.time() + seconds
        nxt = time.time()
        while time.time() < end:
            self.safety()
            if fn() is False:
                return
            nxt += 1.0 / RATE
            while time.time() < nxt:
                rclpy.spin_once(self, timeout_sec=max(0.0, nxt - time.time()))

    # ---------------------------------------------------------------- phases
    def recentre(self, timeout=6.0):
        """Simple proportional return to the start point (and heading)."""
        x0, y0, z0, yaw0 = self.p0

        def step():
            if self.pose_age() > 0.5:
                self.send()
                return None
            _, x, y, z, yaw = self.pose
            ex, ey, ez, eyaw = x0 - x, y0 - y, z0 - z, wrap(yaw0 - yaw)
            c, s = math.cos(yaw), math.sin(yaw)
            bx, by = c * ex + s * ey, -s * ex + c * ey
            if math.hypot(ex, ey) < 0.08 and abs(ez) < 0.08 and abs(eyaw) < math.radians(5):
                self.send()
                return False
            clip = lambda v, m: max(-m, min(m, v))  # noqa: E731
            self.send(clip(0.6 * bx, 0.25), clip(0.6 * by, 0.25), clip(0.6 * ez, 0.25), clip(0.8 * eyaw, 0.4))
            return None
        self.run(timeout, step)
        self.run(1.0, lambda: self.send())

    def doublet(self, segment, axis, sign):
        amp, dur = (self.yaw_amp, self.yaw_dur) if axis == "yaw" else (self.amp, self.dur)
        key = {"x": "cx", "y": "cy", "z": "cz", "yaw": "cw"}[axis]
        for u, d in ((0.0, 2.0), (sign * amp, dur), (-sign * amp, dur), (0.0, 3.0)):
            self.run(d, lambda u=u: self.send(**{key: u}, segment=segment, axis=axis, u=u))

    def wait_ready(self):
        self.say("Waiting for the driver and the telemetry...")
        t0 = time.time()
        while time.time() - t0 < 10.0 and (time.time() - self.tel_t > 1.0):
            self.spin(0.2)
        if time.time() - self.tel_t > 1.0:
            raise Abort("no telemetry: start  ros2 launch tello_driver real.launch.py  first")
        self.spin(1.0)
        others = [i.node_name for i in self.get_publishers_info_by_topic("/tello/cmd_vel_safe")
                  if i.node_name != self.get_name()]
        if others:
            raise Abort(f"another node sends commands ({', '.join(others)}): close the teleop first")
        if time.time() - self.video_t > 1.0:
            raise Abort("no video from the drone: the PC lost the Tello Wi-Fi? Reconnect, wait 30 s, relaunch")
        bat = self.tel.get("bat", 0)
        if isinstance(bat, (int, float)) and bat < self.min_battery:
            raise Abort(f"battery {bat} % < {self.min_battery} %: charge it first")

    def main(self):
        self.prepare()
        self.run_tests()

    def prepare(self):
        """Checks, takeoff + climb (done by the driver), start point. Shared with the marker tour."""
        self.wait_ready()
        if not self.in_air():
            self.say(f"Battery {self.tel.get('bat')} %. Drone 2 m from the marker wall, markers in view, nothing around.")
            self.say("Press ENTER to take off and start (any other key = quit). During the test: ANY KEY = land.")
            k = None
            while k is None:
                rclpy.spin_once(self, timeout_sec=0.05)
                k = read_key(0.05)
            if k not in ("\n", "\r"):
                raise Abort("cancelled")
            r = self.call("takeoff", timeout=3.0)
            self.say(f"takeoff: {r.message if r else '?'}")
            # the driver takes off then climbs to the height where the markers are visible: wait for both
            t0 = time.time()
            self.spin(0.5)
            while (not self.in_air() or self.tel.get("busy")) and time.time() - t0 < 25.0:
                self.check_keys()
                self.spin(0.1)
            self.say(f"In the air at {self.tel.get('tof', '?')} cm (distance sensor)")
            self.spin(1.5)
        # start point = median of the poses over 1.5 s of hover
        self.say("Hovering, measuring the start point...")
        samples = []
        t0 = time.time()
        while time.time() - t0 < 8.0 and len(samples) < 15:
            self.check_keys()
            self.send()
            self.spin(0.05)
            if self.pose_age() < 0.2:
                samples.append(self.pose[1:])
        if len(samples) < 5:
            if time.time() - self.video_t > 1.0:
                raise Abort("no video from the drone (Wi-Fi drop?): reconnect, wait 30 s, relaunch")
            raise Abort("no marker in view: place the drone facing the marker wall (2-3 markers in the picture)")
        S = np.array(samples)
        self.p0 = (float(np.median(S[:, 0])), float(np.median(S[:, 1])), float(np.median(S[:, 2])),
                   float(np.median(S[:, 3])))
        if not self.z_min + 0.3 <= self.p0[2] <= self.z_max - 0.3:
            raise Abort(f"start height {self.p0[2]:.2f} m too close to the limits [{self.z_min}, {self.z_max}]")
        self.say(f"Start point ({self.p0[0]:+.2f}, {self.p0[1]:+.2f}, {self.p0[2]:.2f}) m, "
                 f"yaw {math.degrees(self.p0[3]):.0f} deg.")

    def run_tests(self):
        names = {"x": "forward/back", "y": "left/right", "z": "up/down", "yaw": "turn"}
        seg = 0
        for axis in self.axes:
            for rep in range(self.repeats):
                self.say(f"[{seg + 1}/{len(self.axes) * self.repeats}] {names[axis]} doublet {rep + 1}")
                self.recentre()
                self.doublet(seg, axis, 1 if rep % 2 == 0 else -1)
                seg += 1
        self.recentre()
        self.say("Done: landing.")

    def close(self):
        for f in (self.f_pose, self.f_cmd, self.f_tel):
            f.close()


def main():
    run(StepTest, fit_after=True)


def run(cls, fit_after=False):
    os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "LARGE_DATA")
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl+C must still be able to land
    node = cls()
    old = termios.tcgetattr(sys.stdin)
    doc = sys.modules[cls.__module__].__doc__ or ""
    print("\n\n".join(doc.split("\n\n")[2:4]) + "\n")
    ok = False
    try:
        tty.setcbreak(sys.stdin.fileno())
        node.main()
        ok = True
    except Abort as e:
        node.say(f"STOP: {e}")
    except KeyboardInterrupt:
        node.say("Ctrl+C")
    finally:
        try:
            node.pub.publish(Twist())
            if node.in_air():
                r = node.call("land", timeout=3.0)
                node.say(f"land: {r.message if r else 'sent'}")
        except Exception as e:  # noqa: BLE001
            node.say(f"land request failed ({e}): press g in a teleop or x for emergency")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.close()
        print(f"Data saved in {node.out}")
    if ok and fit_after:
        from tello_driver.step_fit import fit
        fit(node.out)
        print(f"Model: {node.out / 'model.yaml'}   plot: {node.out / 'step_fit.png'}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
