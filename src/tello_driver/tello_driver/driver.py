"""ROS 2 driver for the real DJI Tello. Publishes the SAME topics as the simulator (tello_sim/sim_adapter).

Publishes   /tello/image_raw (rgb8), /tello/camera_info   stamped when the frame reaches the PC
            /tello/telemetry (std_msgs/String, JSON with the djitellopy state fields)
Subscribes  /tello/cmd_vel_safe (Twist, body frame, m/s and rad/s) -> rc commands at 20 Hz
Services    ~/takeoff, ~/land, ~/emergency (std_srvs/Trigger)

Safety: watchdog (hover if no command for cmd_timeout), speed limits, auto-land on low battery,
land on shutdown. Velocity -> rc gains come from the Nov 7 flights (rc 30 ~ 0.38 m/s, yaw rc 30 ~ 18 deg/s).
The Tello answers only the first program that talked to it since power-on: start this driver first.
"""
import json
import math
import threading
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger


# ---------------------------------------------------------------- fake drone (tests without hardware)
class FakeFrameRead:
    """Without a marker map: a plain moving test pattern. With one: the virtual camera view of the markers,
    delivered VIDEO_DELAY late like the real video (the EKF assumes that latency)."""
    def __init__(self, world=None):
        self.world = world
        self._t = -1
        self._frame = None
        self._queue = []

    @property
    def frame(self):
        k = int(time.time() * 30)
        if k != self._t:
            self._t = k
            if self.world is not None:
                from tello_driver.fake_world import VIDEO_DELAY
                now = time.time()
                self._queue.append((now, self.world.render()))
                while len(self._queue) > 1 and self._queue[1][0] <= now - VIDEO_DELAY:
                    self._queue.pop(0)
                self._frame = self._queue[0][1]
            else:
                img = np.full((720, 960, 3), 90, np.uint8)
                img[:, :, 0] = (np.arange(960) * 255 // 960).astype(np.uint8)
                img[(k * 7) % 720, :, :] = 255
                self._frame = img
        return self._frame


class FakeTello:
    def __init__(self, markers_file=""):
        self.world = None
        if markers_file:
            from tello_driver.fake_world import FakeWorld
            self.world = FakeWorld(markers_file)
        self.is_flying = False

    def connect(self): pass
    def streamon(self): pass
    def streamoff(self): pass
    def end(self): pass
    def get_battery(self): return 90
    def get_frame_read(self): return FakeFrameRead(self.world)

    def takeoff(self):
        if self.world is not None:
            self.world.flying = True
            self.world.z_target = max(self.world.p[2], 0.85)  # the real Tello stops around 0.8-0.9 m
            self.wait_z()
        else:
            time.sleep(1.0)
        self.is_flying = True

    def move_up(self, cm):
        if self.world is not None:
            self.world.z_target = self.world.p[2] + cm / 100.0
            self.wait_z()

    def wait_z(self, timeout=8.0):
        t0 = time.time()
        while self.world.z_target is not None and time.time() - t0 < timeout:
            time.sleep(0.05)

    def get_height(self):
        return int(self.world.p[2] * 100) if self.world is not None else 80

    def land(self):
        if self.world is not None:
            self.world.z_target = 0.05
        time.sleep(1.0)
        self.is_flying = False
        if self.world is not None:
            self.world.flying = False

    def emergency(self):
        self.is_flying = False
        if self.world is not None:
            self.world.flying, self.world.z_target = False, 0.05

    def send_command_without_return(self, cmd):
        if cmd == "land":
            threading.Thread(target=self.land, daemon=True).start()

    def send_rc_control(self, a, b, c, d):
        if self.world is not None:
            self.world.rc = (a, b, c, d)

    def get_current_state(self):
        if self.world is not None:
            return self.world.state()
        return {"pitch": 0, "roll": 0, "yaw": 0, "vgx": 0, "vgy": 0, "vgz": 0, "templ": 50, "temph": 52,
                "tof": 80 if self.is_flying else 10, "h": 80 if self.is_flying else 0, "bat": 90,
                "baro": 80.0, "time": 0, "agx": 0.0, "agy": 0.0, "agz": -1000.0}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def load_zone(path):
    """config/flight_zone.yaml -> dict: the mission rectangle (x_min ... z_max), the margin, and when present the
    free-space grid (distance to the nearest obstacle, tools/free_zone.py). None if no file."""
    if not path:
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            z = yaml.safe_load(f)
        zone = {k: float(z[k]) for k in ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max")}
        zone["margin"] = float(z.get("margin", 0.35))
        if z.get("grid_file"):
            from pathlib import Path
            g = np.load(Path(path).parent / z["grid_file"])
            c = g["clearance"].astype(float)
            gy, gx = np.gradient(c)
            zone["grid"] = {"d": c, "gx": gx, "gy": gy, "x0": float(g["x0"]), "y0": float(g["y0"]),
                            "cell": float(g["cell"])}
        return zone
    except Exception:  # noqa: BLE001
        return None


def clearance(zone, x, y):
    """(distance to the nearest obstacle in m, unit vector pointing away from it) at (x, y); None without grid."""
    g = zone.get("grid") if zone else None
    if g is None:
        return None
    j = int(round((x - g["x0"]) / g["cell"]))
    i = int(round((y - g["y0"]) / g["cell"]))
    if not (0 <= i < g["d"].shape[0] and 0 <= j < g["d"].shape[1]):
        return 0.0, (0.0, 0.0)
    gx, gy = g["gx"][i, j], g["gy"][i, j]
    n = math.hypot(gx, gy)
    return float(g["d"][i, j]), ((gx / n, gy / n) if n > 1e-6 else (0.0, 0.0))


# ---------------------------------------------------------------- camera info
def load_camera_info(path, width, height, frame_id):
    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width, info.height = width, height
    if path:
        with open(path, "r", encoding="utf-8") as f:
            c = yaml.safe_load(f)
        info.k = [float(v) for v in c["camera_matrix"]["data"]]
        info.d = [float(v) for v in c["distortion_coefficients"]["data"]]
        info.r = [float(v) for v in c.get("rectification_matrix", {}).get("data", [1, 0, 0, 0, 1, 0, 0, 0, 1])]
        info.p = [float(v) for v in c.get("projection_matrix", {}).get("data", info.k[:3] + [0] + info.k[3:6] + [0] + info.k[6:] + [0])]
        info.distortion_model = c.get("distortion_model", "plumb_bob")
        if c.get("image_width") and (c["image_width"], c["image_height"]) != (width, height):
            raise ValueError(f"calibration is for {c['image_width']}x{c['image_height']}, stream is {width}x{height}")
        return info, True
    # uncalibrated: ideal pinhole with the measured video field of view (55.6 deg horizontal, Nov 9 calibration).
    # DJI's 82.6 deg is the photo mode, not the 960x720 video stream.
    f = width / 2 / math.tan(math.radians(55.6) / 2)
    info.k = [f, 0.0, width / 2, 0.0, f, height / 2, 0.0, 0.0, 1.0]
    info.d = [0.0] * 5
    info.r = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
    info.p = [f, 0.0, width / 2, 0.0, 0.0, f, height / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
    info.distortion_model = "plumb_bob"
    return info, False


class TelloDriver(Node):
    def __init__(self):
        super().__init__("tello_driver")
        p = self.declare_parameter
        self.mock = p("mock", False).value
        self.calib_file = p("camera_info_file", "").value
        self.frame_id = p("camera_frame", "tello_camera_optical").value
        self.lin_gain = p("lin_gain", 0.38 / 30).value       # m/s per rc point (measured Nov 7)
        self.yaw_gain = p("yaw_gain", math.radians(18) / 30).value  # rad/s per rc point (measured Nov 7)
        self.max_rc = int(p("max_rc", 80).value)  # ~1.0 m/s; the zone / ceiling / wall guards still apply
        self.cmd_timeout = p("cmd_timeout", 0.5).value
        self.low_battery = p("low_battery", 12).value
        # after the takeoff (~0.8 m), climb to this height (m above the takeoff point, 0 = stay) so that the
        # camera sees the markers; real.launch.py computes it from the marker map
        self.takeoff_height = p("takeoff_height", 0.0).value
        # highest allowed height above the floor (m): ceiling 2.80 m in the room, 2.60 m max (user, Nov 9)
        self.max_height = p("max_height", 2.6).value
        self.pose_z, self.pose_t = 0.0, 0.0
        self.pose_xy, self.pose_yaw = None, 0.0
        # position hold: when no horizontal move is commanded, hold the position (and heading) with a PI loop on
        # the ArUco pose. The Tello's own optical-flow hold does not work in this room (vgx = vgy = 0 while it
        # drifted 0.4 m in 8 s, Nov 11: dim light / plain floor).
        self.hold_enabled = p("position_hold", True).value
        self.hold_kp, self.hold_ki = p("hold_kp", 0.6).value, p("hold_ki", 0.15).value
        self.hold_target = None
        self.hold_i = [0.0, 0.0]
        self.hold_t = 0.0
        # obstacle-free flight zone from the room scan (tools/free_zone.py): commands that would take the drone
        # out of it are cut, whatever program sends them (Nov 10: a test drifted 1 m back towards furniture)
        self.zone = load_zone(p("flight_zone_file", "").value)
        if self.zone:
            self.get_logger().info("flight zone x [{x_min:.2f}, {x_max:.2f}] y [{y_min:.2f}, {y_max:.2f}]".format(**self.zone))
        self.climb_target = None  # set after takeoff: send_rc climbs to it with joystick commands

        # Refuse to run next to another driver (e.g. a forgotten fake drone): two sources of /tello/image_raw
        # make the camera view flicker and the pose jump between them.
        t0 = time.time()
        while time.time() - t0 < 2.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        others = [i.node_name for i in self.get_publishers_info_by_topic("/tello/image_raw")]
        if others:
            self.get_logger().fatal(f"Another node already publishes /tello/image_raw ({', '.join(others)}): stop it "
                                    "first (close its terminal, or: pkill -f tello_driver/tello_driver).")
            raise SystemExit(1)

        self.pub_img = self.create_publisher(Image, "/tello/image_raw", 5)
        self.pub_info = self.create_publisher(CameraInfo, "/tello/camera_info", 5)
        self.pub_tel = self.create_publisher(String, "/tello/telemetry", 10)
        # the command really sent (after hold / guards), in m/s and rad/s, body frame: the EKF's input
        self.pub_sent = self.create_publisher(Twist, "/tello/rc_sent", 10)
        self.create_subscription(Twist, "/tello/cmd_vel_safe", self.on_cmd, 10)
        # pose used by the hold and the guards: the EKF (smooth, predicted between detections, delay compensated)
        # or the raw ArUco pose; trusted while its uncertainty is small
        self.pose_sigma = 1.0
        self.create_subscription(PoseWithCovarianceStamped, p("pose_topic", "/ekf/pose").value, self.on_pose, 10)
        # wall ahead, from the size of the closest marker in view (works without map and without pose)
        from std_msgs.msg import Float32
        self.near_d, self.near_t = 9.0, 0.0
        self.min_wall = p("min_wall_distance", 0.7).value
        self.wall_band = p("wall_slow_band", 0.15).value  # forward speed shrinks over this band above min_wall
        self.create_subscription(Float32, "/aruco/nearest_marker", self.on_near, 5)
        self.create_service(Trigger, "~/takeoff", self.srv_takeoff)
        self.create_service(Trigger, "~/land", self.srv_land)
        self.create_service(Trigger, "~/emergency", self.srv_emergency)

        if self.mock:
            # with a marker map, the fake drone renders the markers and flies with the rc commands
            self.tello = FakeTello(p("mock_markers_file", "").value)
            if self.zone and self.tello.world is not None:
                # start where the real one should: in the zone, 0.3 m from its edge furthest from the main marker
                # wall (the drone faces that wall; from the zone centre the markers were cut at the image edges)
                w = self.tello.world
                cx, cy = (self.zone["x_min"] + self.zone["x_max"]) / 2, (self.zone["y_min"] + self.zone["y_max"]) / 2
                n = (-math.cos(w.yaw), -math.sin(w.yaw))  # away from the wall
                hx, hy = (self.zone["x_max"] - self.zone["x_min"]) / 2, (self.zone["y_max"] - self.zone["y_min"]) / 2
                d = max(0.0, abs(n[0]) * hx + abs(n[1]) * hy - 0.3)
                w.p[0], w.p[1] = cx + n[0] * d, cy + n[1] * d
        else:
            self.wait_for_tello_network(p("network_timeout", 30.0).value)
            import logging
            from djitellopy import Tello
            Tello.LOGGER.setLevel(logging.WARNING)  # no 'rc 0 0 0 0' line 20 times per second
            self.tello = Tello()
        self.get_logger().info("Connecting to the Tello" + (" (FAKE)" if self.mock else "") + "...")
        try:
            self.tello.connect()
        except Exception as e:  # noqa: BLE001
            self.get_logger().fatal(f"No answer from the Tello ({e}). Switch it OFF and on, then start this launch "
                                    "FIRST (the Tello answers only its first client), and stay on its Wi-Fi.")
            raise SystemExit(1)
        self.get_logger().info(f"Connected, battery {self.tello.get_battery()} %")
        self.tello.streamon()
        self.get_logger().info("Video stream on, waiting for the first frames...")
        self.reader = self.tello.get_frame_read()

        self.info = None
        self.last_frame = None
        self.rc_target = (0, 0, 0, 0)
        self.last_cmd = 0.0
        self.busy = threading.Lock()
        self.state = {}
        self.last_frame_t = time.time()
        self.video_timeout = p("video_timeout", 3.0).value
        self.restarting = False
        self.create_timer(1 / 60, self.poll_video)
        self.create_timer(0.1, self.poll_state)
        self.create_timer(0.05, self.send_rc)
        self.create_timer(1.0, self.video_watchdog)

    def video_watchdog(self):
        """The Wi-Fi/WSL route to the drone can drop for a few seconds (Windows re-checks the network ~20 s
        after joining the Tello's Wi-Fi). djitellopy's reader blocks on its UDP port 11111 with no timeout: if
        its thread is still alive it resumes by itself once packets come back, so only 'streamon' is re-sent.
        A NEW reader while the old one holds the port fails with 'Address already in use' (Nov 9): a new one is
        made only if the old thread is dead, after closing its container."""
        if self.mock or self.restarting or time.time() - self.last_frame_t < self.video_timeout:
            return
        self.restarting = True
        r = self.tello.background_frame_read
        alive = r is not None and r.worker.is_alive()
        self.get_logger().warn(f"No video for {time.time() - self.last_frame_t:.0f} s: "
                               + ("re-sending streamon (reader still waiting)" if alive else "restarting the reader"))

        def restart():
            try:
                self.tello.send_command_without_return("streamon")
                if not alive:
                    try:
                        if r is not None:
                            r.stopped = True
                            r.container.close()  # frees UDP port 11111
                    except Exception:  # noqa: BLE001
                        pass
                    self.tello.background_frame_read = None
                    time.sleep(0.5)
                    self.reader = self.tello.get_frame_read()
                    self.get_logger().info("Video reader restarted")
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"Video restart failed ({e}): retrying", throttle_duration_sec=5.0)
            finally:
                self.last_frame_t = time.time()  # give the stream time to come back
                self.restarting = False
        threading.Thread(target=restart, daemon=True).start()

    def wait_for_tello_network(self, timeout):
        """WSL mirrored networking takes a few seconds to follow a Wi-Fi change: wait for a route to the drone."""
        import socket
        t0, warned = time.time(), False
        while True:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(("192.168.10.1", 8889))  # sends nothing: only asks for a route
                ip = s.getsockname()[0]
                s.close()
                if ip.startswith("192.168.10."):
                    self.get_logger().info(f"On the Tello Wi-Fi ({ip})")
                    return
            except OSError:
                ip = "no route"
            if time.time() - t0 > timeout:
                self.get_logger().fatal(f"Not on the Tello Wi-Fi after {timeout:.0f} s ({ip}). Connect Windows to "
                                        "TELLO-xxxxxx, wait ~10 s, then relaunch.")
                raise SystemExit(1)
            if not warned:
                self.get_logger().warn("Waiting for the Tello Wi-Fi (connect Windows to TELLO-xxxxxx)...")
                warned = True
            time.sleep(1.0)

    # ---------------------------------------------------------------- video
    def poll_video(self):
        frame = self.reader.frame
        if frame is None or frame is self.last_frame:
            return
        # djitellopy starts with a black 400x300 placeholder before the first decoded frame: skip it
        if frame.shape[:2] == (300, 400) and not frame.any():
            return
        self.last_frame = frame
        self.last_frame_t = time.time()
        h, w = frame.shape[:2]
        if self.info is None or (self.info.width, self.info.height) != (w, h):
            self.info, calibrated = load_camera_info(self.calib_file, w, h, self.frame_id)
            if self.mock and getattr(self.tello, "world", None) is not None:
                # fake camera: ideal lens (the renderer cannot distort inside a marker), the same in camera_info
                self.info.d = [0.0] * len(self.info.d)
                self.tello.world.set_camera(self.info.k, self.info.d)
            self.get_logger().info(f"Video {w}x{h}, camera_info: " +
                                   (f"calibration {self.calib_file}" if calibrated else "UNCALIBRATED (ideal pinhole from the FOV)"))
        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, self.frame_id
        msg.height, msg.width, msg.encoding, msg.is_bigendian, msg.step = h, w, "rgb8", 0, w * 3
        msg.data = np.ascontiguousarray(frame).tobytes()
        self.pub_img.publish(msg)
        self.info.header.stamp = stamp
        self.pub_info.publish(self.info)

    # ---------------------------------------------------------------- telemetry + safety
    def poll_state(self):
        try:
            self.state = dict(self.tello.get_current_state() or {})
        except Exception:  # noqa: BLE001
            return
        st = dict(self.state)
        st["stamp"] = self.get_clock().now().nanoseconds * 1e-9
        st["flying"] = bool(self.tello.is_flying)
        # takeoff / climb / land running: not ready for velocity commands
        st["busy"] = self.busy.locked() or self.climb_target is not None
        self.pub_tel.publish(String(data=json.dumps(st)))
        bat = self.state.get("bat")
        if self.tello.is_flying and isinstance(bat, (int, float)) and bat <= self.low_battery:
            self.get_logger().warn(f"Battery {bat} %: landing")
            self.run_async(self.tello.land, "low-battery land")

    # ---------------------------------------------------------------- commands
    def on_cmd(self, m):
        clip = lambda v: int(max(-self.max_rc, min(self.max_rc, round(v))))  # noqa: E731
        # Tello rc: (left/right, forward/back, up/down, yaw), right/forward/up/clockwise positive.
        # ROS body frame: x forward, y LEFT, z up, yaw counter-clockwise positive.
        self.rc_target = (clip(-m.linear.y / self.lin_gain), clip(m.linear.x / self.lin_gain),
                          clip(m.linear.z / self.lin_gain), clip(-m.angular.z / self.yaw_gain))
        self.last_cmd = time.time()

    def send_rc(self):
        if self.busy.locked():
            return  # a takeoff / climb / land command is running: rc would interfere with it
        rc = self.rc_target if time.time() - self.last_cmd < self.cmd_timeout else (0, 0, 0, 0)
        hgt = self.height_estimate() if self.tello.is_flying else None
        vz = 0.0
        if hgt is not None:
            now = time.time()
            self.h_hist = [(t, h) for t, h in getattr(self, "h_hist", []) if now - t < 0.6] + [(now, hgt)]
            t_old, h_old = self.h_hist[0]
            vz = (hgt - h_old) / (now - t_old) if now - t_old > 0.2 else 0.0
        if self.climb_target is not None and self.tello.is_flying:
            ud = self.climb_rc(hgt, vz)
            if ud is not None:
                rc = (0, 0, ud, 0)  # climbing: other commands wait (the position hold keeps it in place)
        rc = self.position_hold(rc)
        # ceiling guard, whatever program sends the commands. The drone needs ~1 s to stop (tau ~0.8 s + delay),
        # so the guard works on the height predicted 1 s ahead, not on the current one.
        if hgt is not None:
            h_pred = hgt + max(vz, 0.0) * 1.0
            limit = self.max_height - 0.15
            lr, fb, ud, yw = rc
            if hgt > self.max_height:
                ud = -30
                self.get_logger().warn(f"height {hgt:.2f} m > {self.max_height:.2f} m: descending",
                                       throttle_duration_sec=1.0)
            elif h_pred > limit:
                ud = min(ud, -20)  # brake now, or it overshoots
                self.get_logger().warn(f"height {hgt:.2f} m rising {vz:+.2f} m/s: braking before the "
                                       f"{self.max_height:.2f} m limit", throttle_duration_sec=1.0)
            else:
                ud = min(ud, int((limit - h_pred) / self.lin_gain))  # slower and slower towards the limit
            rc = (lr, fb, ud, yw)
        rc = self.zone_guard(rc)
        rc = self.wall_guard(rc)
        try:
            self.tello.send_rc_control(*rc)  # also keeps the link alive (the Tello lands after 15 s of silence)
            t = Twist()
            t.linear.x, t.linear.y, t.linear.z = rc[1] * self.lin_gain, -rc[0] * self.lin_gain, rc[2] * self.lin_gain
            t.angular.z = -rc[3] * self.yaw_gain
            self.pub_sent.publish(t)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"rc failed: {e}", throttle_duration_sec=2.0)

    def run_async(self, fn, label):
        def worker():
            if not self.busy.acquire(blocking=False):
                return
            try:
                self.get_logger().info(label)
                fn()
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"{label} failed: {e}")
            finally:
                self.busy.release()
        threading.Thread(target=worker, daemon=True).start()

    def srv_takeoff(self, req, res):
        bat = self.state.get("bat", 100)
        if isinstance(bat, (int, float)) and bat < 25:
            res.success, res.message = False, f"battery {bat} % < 25 %: takeoff refused"
            return res
        self.run_async(self.takeoff_and_climb, "takeoff")
        res.success, res.message = True, ("taking off" + (f", then climbing to {self.takeoff_height:.2f} m"
                                                         if self.takeoff_height > 0 else ""))
        return res

    def takeoff_and_climb(self):
        """Takeoff (SDK command), then the climb is done by send_rc with joystick commands, closed-loop on the
        measured height. The SDK 'up' command was refused right after takeoff ('error Not joystick', Nov 9),
        and 'h' (barometer) read 20 cm at 80 cm: distance sensor + ArUco pose only."""
        self.tello.takeoff()
        if self.takeoff_height > 0:
            self.climb_target = min(self.takeoff_height, self.max_height - 0.6)
            self.climb_t0 = time.time()
            self.climb_stable = None
            self.get_logger().info(f"climbing to {self.climb_target:.2f} m (markers in view)")

    def climb_rc(self, hgt, vz):
        """Vertical rc during the climb phase (None when done)."""
        now = time.time()
        if hgt is None:
            return 0 if now - self.climb_t0 < 10.0 else self.end_climb("no height measurement")
        err = self.climb_target - (hgt + 0.8 * vz)  # predicted ahead: the drone keeps moving ~1 s
        if abs(self.climb_target - hgt) < 0.10 and abs(vz) < 0.15:  # real sensors are noisier than the sim
            self.climb_stable = self.climb_stable or now
            if now - self.climb_stable > 0.5:
                return self.end_climb(f"climb done at {hgt:.2f} m")
        else:
            self.climb_stable = None
        if now - self.climb_t0 > 12.0:
            return self.end_climb(f"climb stopped after 12 s at {hgt:.2f} m")
        return int(max(-25, min(40, err * 100)))

    def end_climb(self, msg):
        self.climb_target = None
        self.get_logger().info(msg)
        return None

    def grid_guard(self, x, y, vx, vy, wx, wy):
        """Free-space guard: distance to the nearest scanned obstacle at the current and the predicted (1 s)
        position. Closer than the margin: the velocity towards the obstacle is removed and the drone is pushed
        away; approaching it: that velocity shrinks to zero at the margin."""
        mg = self.zone["margin"]
        d, (gx, gy) = clearance(self.zone, x, y)
        dp, (gpx, gpy) = clearance(self.zone, x + vx, y + vy)
        if dp < d:
            d_use, (ux, uy) = dp, (gpx, gpy)
        else:
            d_use, (ux, uy) = d, (gx, gy)
        if ux == 0.0 and uy == 0.0:
            return wx, wy
        toward = -(wx * ux + wy * uy)  # > 0: moving towards the obstacle
        if d < mg:
            if toward > -0.25:
                wx, wy = wx + (toward + 0.25) * ux, wy + (toward + 0.25) * uy
            self.get_logger().warn(f"flight zone: {d:.2f} m from an obstacle at ({x:+.2f}, {y:+.2f}): pushing away",
                                   throttle_duration_sec=1.0)
        elif d_use < mg + 0.1:
            if toward > -0.15:
                wx, wy = wx + (toward + 0.15) * ux, wy + (toward + 0.15) * uy
            self.get_logger().warn(f"flight zone: braking ({d_use:.2f} m from an obstacle)", throttle_duration_sec=1.0)
        else:
            allowed = (d_use - mg) * 1.0
            if toward > allowed:
                wx, wy = wx + (toward - allowed) * ux, wy + (toward - allowed) * uy
        return wx, wy

    def on_near(self, m):
        self.near_d, self.near_t = float(m.data), time.time()

    def wall_guard(self, rc):
        """A marker in view closer than min_wall_distance: no forward motion, back off. Between that and +0.3 m the
        forward speed shrinks. Independent of the map and of the pose (camera = forward)."""
        if not self.tello.is_flying or time.time() - self.near_t > 0.4:
            return rc
        lr, fb, ud, yw = rc
        d = self.near_d
        if d < self.min_wall:
            fb = min(fb, -12)
            self.get_logger().warn(f"wall {d:.2f} m ahead (marker size): backing off", throttle_duration_sec=1.0)
        elif d < self.min_wall + self.wall_band and fb > 0:
            fb = int(fb * (d - self.min_wall) / self.wall_band)
        return (lr, fb, ud, yw)

    def pose_valid(self, max_age, max_sigma):
        """A pose recent enough and certain enough (the EKF keeps publishing without markers, with a growing
        uncertainty: beyond max_sigma it is not trusted)."""
        return (self.pose_xy is not None and time.time() - self.pose_t < max_age and self.pose_sigma < max_sigma)

    def on_pose(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.pose_sigma = math.sqrt(max(m.pose.covariance[0], m.pose.covariance[7], 0.0))
        self.pose_z, self.pose_t = p.z, time.time()
        self.pose_xy = (p.x, p.y)
        self.pose_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        now = time.time()
        self.xy_hist = [(t, xy) for t, xy in getattr(self, "xy_hist", []) if now - t < 0.6] + [(now, (p.x, p.y))]

    def velocity_xy(self):
        hist = getattr(self, "xy_hist", [])
        if len(hist) < 2:
            return 0.0, 0.0
        (t0, (x0, y0)), (t1, (x1, y1)) = hist[0], hist[-1]
        dt = t1 - t0
        return ((x1 - x0) / dt, (y1 - y0) / dt) if dt > 0.2 else (0.0, 0.0)

    def position_hold(self, rc):
        """No horizontal/yaw command -> hold where the drone stops (PI on the ArUco pose, integral for the drift).
        A command releases the hold; no fresh pose -> nothing to hold on (the integral is kept)."""
        now = time.time()
        dt = min(0.1, now - self.hold_t) if self.hold_t else 0.05
        self.hold_t = now
        if not (self.hold_enabled and self.tello.is_flying) or self.busy.locked():
            self.hold_target = None
            return rc
        if rc[0] or rc[1] or rc[3]:
            self.hold_target = None  # the pilot / the mission is moving the drone
            self.hold_i = [0.0, 0.0]
            return rc
        if not self.pose_valid(0.4, 0.12):
            return rc
        x, y = self.pose_xy
        if self.hold_target is None:
            vx, vy = self.velocity_xy()
            # hold where it will stop (it keeps moving ~0.8 s), not where it was when the keys were released
            self.hold_target = (x + 0.8 * vx, y + 0.8 * vy, self.pose_yaw)
            self.hold_i = [0.0, 0.0]
        tx, ty, tyaw = self.hold_target
        ex, ey = tx - x, ty - y
        for k, e in enumerate((ex, ey)):
            if abs(e) < 0.4:
                self.hold_i[k] = max(-0.15, min(0.15, self.hold_i[k] + self.hold_ki * e * dt))
        wx = max(-0.25, min(0.25, self.hold_kp * ex + self.hold_i[0]))
        wy = max(-0.25, min(0.25, self.hold_kp * ey + self.hold_i[1]))
        c, s = math.cos(self.pose_yaw), math.sin(self.pose_yaw)
        bx, by = c * wx + s * wy, -s * wx + c * wy
        wz = max(-0.3, min(0.3, 0.8 * wrap(tyaw - self.pose_yaw)))
        clip = lambda v: int(max(-self.max_rc, min(self.max_rc, round(v))))  # noqa: E731
        return (clip(-by / self.lin_gain), clip(bx / self.lin_gain), rc[2], clip(-wz / self.yaw_gain))

    def zone_guard(self, rc):
        """Keep the drone inside the flight zone: like the ceiling guard, on the position predicted 1 s ahead
        (the drone needs ~1 s to stop). Outward commands shrink near the edge, brake before it, push back in
        beyond it. Needs a fresh ArUco pose; without one, horizontal moves are blocked."""
        z = self.zone
        if not z or not self.tello.is_flying or self.climb_target is not None:
            return rc
        if not self.pose_valid(0.5, 0.25):
            # position unknown: the zone cannot be enforced, so no horizontal move at all (turning and up/down stay
            # allowed, to find the markers again). Without this the drone flew blind through the wall (sim, Nov 10).
            if rc[0] or rc[1]:
                self.get_logger().warn("no marker pose: horizontal moves blocked (turn to see the markers)",
                                       throttle_duration_sec=2.0)
            return (0, 0, rc[2], rc[3])
        x, y = self.pose_xy
        vx, vy = self.velocity_xy()
        lr, fb, ud, yw = rc
        c, s = math.cos(self.pose_yaw), math.sin(self.pose_yaw)
        bx, by = fb * self.lin_gain, -lr * self.lin_gain          # body velocity command
        wx, wy = c * bx - s * by, s * bx + c * by                 # world
        if "grid" in z:
            wx, wy = self.grid_guard(x, y, vx, vy, wx, wy)
            bx, by = c * wx + s * wy, -s * wx + c * wy
            clip = lambda v: int(max(-self.max_rc, min(self.max_rc, round(v))))  # noqa: E731
            return (clip(-by / self.lin_gain), clip(bx / self.lin_gain), ud, yw)
        m = 0.15
        hit = []
        for name, pos, vel, lo, hi in (("x", x, vx, z["x_min"], z["x_max"]), ("y", y, vy, z["y_min"], z["y_max"])):
            pred = pos + vel * 1.0
            w = wx if name == "x" else wy
            if pos > hi:
                w = min(w, -0.25); hit.append(f"{name}>{hi:.2f}")
            elif pred > hi - m:
                w = min(w, -0.15); hit.append(f"{name}->{hi:.2f}")
            else:
                w = min(w, (hi - m - pred) * 1.0)
            if pos < lo:
                w = max(w, 0.25); hit.append(f"{name}<{lo:.2f}")
            elif pred < lo + m:
                w = max(w, 0.15); hit.append(f"{name}->{lo:.2f}")
            else:
                w = max(w, (lo + m - pred) * 1.0)
            if name == "x":
                wx = w
            else:
                wy = w
        if hit:
            self.get_logger().warn(f"flight zone: braking/pushing back ({', '.join(hit)}) at ({x:+.2f}, {y:+.2f})",
                                   throttle_duration_sec=1.0)
        bx, by = c * wx + s * wy, -s * wx + c * wy
        clip = lambda v: int(max(-self.max_rc, min(self.max_rc, round(v))))  # noqa: E731
        return (clip(-by / self.lin_gain), clip(bx / self.lin_gain), ud, yw)

    def height_estimate(self):
        """Height above the floor (m): the larger of the distance sensor and a fresh ArUco pose (conservative
        for the ceiling; the sensor reads less over the bed). None if neither is valid."""
        c = []
        tof = self.state.get("tof")
        if isinstance(tof, (int, float)) and 15 <= tof <= 400:
            c.append(tof / 100.0)
        if self.pose_valid(0.5, 0.15):
            c.append(self.pose_z)
        return max(c) if c else None

    def srv_land(self, req, res):
        self.rc_target = (0, 0, 0, 0)
        self.climb_target = None
        if self.busy.locked():
            # another command (e.g. a takeoff still waiting for its reply) holds the lock: a landing order must
            # never be dropped, so send it right away without waiting for an answer
            self.get_logger().warn("land while another command is running: sending 'land' immediately")
            try:
                self.tello.send_command_without_return("land")
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"land failed: {e}")
            res.success, res.message = True, "landing (sent immediately)"
            return res
        self.run_async(self.tello.land, "land")
        res.success, res.message = True, "landing"
        return res

    def srv_emergency(self, req, res):
        self.climb_target = None
        self.tello.emergency()
        res.success, res.message = True, "EMERGENCY: motors off"
        return res

    def shutdown(self):
        """Stop quickly, even when the PC already left the Tello Wi-Fi (no 7 s retries per command)."""
        try:
            if self.tello.is_flying:
                self.get_logger().info("Shutting down: landing")
                self.tello.send_rc_control(0, 0, 0, 0)
                self.tello.land()
        except Exception:  # noqa: BLE001
            pass
        if self.mock:
            return
        try:
            self.tello.send_command_without_return("streamoff")  # fire and forget
        except Exception:  # noqa: BLE001
            pass
        try:
            if self.tello.background_frame_read is not None:
                self.tello.background_frame_read.stop()
                self.tello.background_frame_read = None
        except Exception:  # noqa: BLE001
            pass
        # djitellopy's __del__ calls end() -> streamoff with retries: skip it, everything is already stopped
        type(self.tello).__del__ = lambda _self: None


def main():
    rclpy.init()
    node = TelloDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
