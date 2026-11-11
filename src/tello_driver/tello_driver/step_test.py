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


class Lost(Exception):
    """Markers lost during a test segment: search, then repeat the segment."""


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
        # no marker for that long (searching all the time) -> land; shorter losses: the drone stays up and searches
        self.lost_max = p("lost_timeout", 15.0).value
        self.failed_segments = []
        zf = p("flight_zone_file", os.path.join(os.environ.get("TELLO_REPO", str(Path.home() / "tello_ws/src/tello-balloon-hunter")),
                                                 "config", "flight_zone.yaml")).value
        from tello_driver.driver import load_zone
        self.zone = load_zone(zf)
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
        imu = self.imu_yaw()
        if imu is not None:
            self.yaw_offset = wrap(yaw - imu)
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

    def imu_yaw(self):
        """Tello IMU heading in the map convention (rad): its 'yaw' is clockwise-positive degrees
        (checked on 4 real sessions: ArUco yaw = -IMU yaw + offset, slope -0.93 to -1.08)."""
        y = self.tel.get("yaw")
        if not isinstance(y, (int, float)) or time.time() - self.tel_t > 0.5:
            return None
        return -math.radians(y)

    def yaw_estimate(self):
        """ArUco yaw when fresh, else IMU yaw + the offset learnt from the last pose (markers out of view)."""
        if self.pose_age() < 0.5:
            return self.pose[4]
        imu = self.imu_yaw()
        off = getattr(self, "yaw_offset", None)
        return None if imu is None or off is None else wrap(imu + off)

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
            zn = self.zone
            if zn and "grid" in zn:
                from tello_driver.driver import clearance
                d, _ = clearance(zn, x, y)
                if d < zn["margin"] - 0.2:
                    raise Abort(f"{d:.2f} m from an obstacle at ({x:+.2f}, {y:+.2f}) m")
            elif zn and not (zn["x_min"] - 0.2 <= x <= zn["x_max"] + 0.2 and zn["y_min"] - 0.2 <= y <= zn["y_max"] + 0.2):
                raise Abort(f"left the obstacle-free flight zone at ({x:+.2f}, {y:+.2f}) m")
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

    def send(self, cx=0.0, cy=0.0, cz=0.0, cw=0.0, segment=-1, axis="", u=0.0, blind=False):
        t = Twist()
        # no fresh pose: hover instead of continuing a test blind (blind=True: a turn guided by the IMU)
        if self.pose_age() > 0.7 and not blind:
            cx = cy = cz = cw = u = 0.0
        t.linear.x, t.linear.y, t.linear.z, t.angular.z = cx, cy, cz, cw
        self.pub.publish(t)
        self.w_cmd.writerow([f"{time.time():.4f}", segment, axis, f"{u:.4f}",
                             f"{cx:.3f}", f"{cy:.3f}", f"{cz:.3f}", f"{cw:.3f}"])

    def search_heading(self, base_yaw):
        """Markers out of view: turn (IMU heading) to base_yaw, then sweep +-40 deg around it. No translation:
        the driver blocks horizontal moves without a pose anyway."""
        yaw = self.yaw_estimate()
        lost = self.pose_age()
        if yaw is None:
            self.send()
            return
        target = base_yaw + (math.radians(40) * math.sin(2 * math.pi * (lost - 3.0) / 6.0) if lost > 3.0 else 0.0)
        if int(lost) != getattr(self, "_said", -1) and int(lost) % 3 == 1:
            self._said = int(lost)
            self.say(f"  markers lost for {lost:.0f} s: turning to find them")
        self.send(cw=max(-0.5, min(0.5, 0.9 * wrap(target - yaw))), blind=True)

    def search_until_found(self, base_yaw, timeout=None):
        t0 = time.time()
        while self.pose_age() > 0.3:
            self.safety()
            self.search_heading(base_yaw)
            self.spin(1.0 / RATE)
            if timeout and time.time() - t0 > timeout:
                return False
        return True

    def run(self, seconds, fn, restart_on_loss=False, search_on_loss=True):
        """Call fn() at RATE for `seconds` (fn sends the command), with the safety checks. Markers lost for more
        than 1 s: Lost (the segment will be repeated) or, outside the tests, a search on the spot."""
        end = time.time() + seconds
        nxt = time.time()
        while time.time() < end:
            self.safety()
            if self.pose_age() > 1.0 and self.p0 is not None and (restart_on_loss or search_on_loss):
                if restart_on_loss:
                    raise Lost()
                self.search_until_found(self.p0[3])
                nxt = time.time()
                continue
            if fn() is False:
                return
            nxt += 1.0 / RATE
            while time.time() < nxt:
                rclpy.spin_once(self, timeout_sec=max(0.0, nxt - time.time()))

    # ---------------------------------------------------------------- phases
    def recentre(self, timeout=6.0, target=None):
        """Simple proportional return to the start point (and heading), or to target (x, y, z, yaw)."""
        x0, y0, z0, yaw0 = target or self.p0

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

    def hold_cmd(self):
        """Body-frame command that pulls the drone back to its start point (the Tello drifts ~10 cm/s on its
        own: 37 cm in 4 s of 'hover' on Nov 10, and the test left its box)."""
        if self.pose_age() > 0.5:
            return {"cx": 0.0, "cy": 0.0, "cz": 0.0, "cw": 0.0}
        x0, y0, z0, yaw0 = self.p0
        _, x, y, z, yaw = self.pose
        ex, ey = x0 - x, y0 - y
        c, s = math.cos(yaw), math.sin(yaw)
        clip = lambda v, m: max(-m, min(m, v))  # noqa: E731
        return {"cx": clip(0.6 * (c * ex + s * ey), 0.2), "cy": clip(0.6 * (-s * ex + c * ey), 0.2),
                "cz": clip(0.6 * (z0 - z), 0.2), "cw": clip(0.8 * wrap(yaw0 - yaw), 0.3)}

    def doublet(self, segment, axis, sign):
        """Hover (position held) - pulse +A - pulse -A - hover. During the pulses the tested axis is open loop
        and the other axes keep holding the position; the logged u is what was really sent on the tested axis."""
        amp, dur = (self.yaw_amp, self.yaw_dur) if axis == "yaw" else (self.amp, self.dur)
        key = {"x": "cx", "y": "cy", "z": "cz", "yaw": "cw"}[axis]

        def cmd(u):
            c = self.hold_cmd()
            if u is not None:
                c[key] = u
            self.send(**c, segment=segment, axis=axis, u=c[key])
        for u, d in ((None, 2.0), (sign * amp, dur), (-sign * amp, dur), (None, 3.0)):
            self.run(d, lambda u=u: cmd(u), restart_on_loss=True)

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
        imu0 = self.imu_yaw()
        while time.time() - t0 < 20.0 and len(samples) < 15:
            self.check_keys()
            if time.time() - self.video_t > 1.0 and time.time() - t0 > 3.0:
                raise Abort("no video from the drone (Wi-Fi drop?): reconnect, wait 30 s, relaunch")
            if self.pose_age() < 0.2:
                samples.append(self.pose[1:])
                self.send()
            elif imu0 is not None and time.time() - t0 > 2.0:
                # no marker yet: sweep +-40 deg around the takeoff heading (IMU) to find them
                imu = self.imu_yaw()
                tgt = imu0 + math.radians(40) * math.sin(2 * math.pi * (time.time() - t0 - 2.0) / 6.0)
                if imu is not None:
                    self.send(cw=max(-0.5, min(0.5, 0.9 * wrap(tgt - imu))), blind=True)
                if int(time.time() - t0) % 4 == 0 and int(time.time() - t0) != getattr(self, "_said0", -1):
                    self._said0 = int(time.time() - t0)
                    self.say("  no marker yet: turning to find them")
            else:
                self.send()
            self.spin(0.05)
        if len(samples) < 5:
            raise Abort("no marker found in 20 s: place the drone facing the marker wall")
        S = np.array(samples)
        self.p0 = (float(np.median(S[:, 0])), float(np.median(S[:, 1])), float(np.median(S[:, 2])),
                   float(np.median(S[:, 3])))
        zn = self.zone
        if zn and not (zn["x_min"] + 0.1 <= self.p0[0] <= zn["x_max"] - 0.1 and zn["y_min"] + 0.1 <= self.p0[1] <= zn["y_max"] - 0.1):
            # the takeoff drifted (or the drone was put a bit off): fly back into the mission rectangle
            tx = min(max(self.p0[0], zn["x_min"] + 0.2), zn["x_max"] - 0.2)
            ty = min(max(self.p0[1], zn["y_min"] + 0.2), zn["y_max"] - 0.2)
            self.say(f"Start point ({self.p0[0]:+.2f}, {self.p0[1]:+.2f}) outside the flight zone: moving to ({tx:+.2f}, {ty:+.2f})")
            self.recentre(timeout=12.0, target=(tx, ty, self.p0[2], self.p0[3]))
            _, x, y, z, yaw = self.pose
            self.p0 = (x, y, z, self.p0[3])
        if not self.z_min + 0.3 <= self.p0[2] <= self.z_max - 0.3:
            raise Abort(f"start height {self.p0[2]:.2f} m too close to the limits [{self.z_min}, {self.z_max}]")
        self.say(f"Start point ({self.p0[0]:+.2f}, {self.p0[1]:+.2f}, {self.p0[2]:.2f}) m, "
                 f"yaw {math.degrees(self.p0[3]):.0f} deg.")

    def run_tests(self):
        names = {"x": "forward/back", "y": "left/right", "z": "up/down", "yaw": "turn"}
        seg = 0
        n_tot = len(self.axes) * self.repeats
        for axis in self.axes:
            for rep in range(self.repeats):
                for attempt in range(3):
                    self.say(f"[{rep + 1 + self.repeats * self.axes.index(axis)}/{n_tot}] {names[axis]} doublet {rep + 1}"
                             + (f" (retry {attempt})" if attempt else ""))
                    self.recentre()
                    try:
                        self.doublet(seg, axis, 1 if rep % 2 == 0 else -1)
                        seg += 1
                        break
                    except Lost:
                        self.failed_segments.append(seg)
                        seg += 1
                        self.say("  markers lost during the test: searching, then repeating it")
                        self.search_until_found(self.p0[3])
        self.recentre()
        self.say("Done: landing.")

    def close(self):
        for f in (self.f_pose, self.f_cmd, self.f_tel):
            f.close()
        (self.out / "failed_segments.txt").write_text(" ".join(map(str, self.failed_segments)))


def main():
    run(StepTest, fit_after=True)


def run(cls, fit_after=False):
    os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "LARGE_DATA")
    from tello_driver.ros_env import local_ros_env
    local_ros_env()  # + the Fast DDS profile (loopback only, small packets)
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
