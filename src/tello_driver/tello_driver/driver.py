"""ROS 2 driver for the DJI Tello: the link to the drone, the video, takeoff / landing, and the ROS wiring.

Publishes   /tello/image_raw (rgb8), /tello/camera_info   stamped when the frame reaches the PC
            /tello/telemetry (std_msgs/String, JSON with the djitellopy state fields + driver fields)
            /tello/rc_sent (Twist, body frame): the command REALLY sent, after the hold and the guards (EKF input)
Subscribes  /tello/cmd_vel_safe (Twist, body frame, m/s and rad/s) -> rc commands at 20 Hz
            /ekf/pose, /ekf/velocity, /ekf/drift, /aruco/nearest_marker: for the hold and the guards
Services    ~/takeoff, ~/land, ~/emergency (std_srvs/Trigger)

Every command goes through safety.CommandFilter (position hold, ceiling, height band, flight zone, wall) before it
is sent: whatever program sends it (teleop, missions). The fake drone (mock:=true) is in fake_tello.py, the flight
zone of the room scan in zone.py. Velocity -> rc gains from the Nov 7 flights (rc 30 ~ 0.38 m/s, yaw rc 30 ~
18 deg/s). The Tello answers only the first program that talked to it since power-on: start this driver first.
"""
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistStamped, Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

from tello_driver.fake_tello import FakeFrameRead, FakeTello  # noqa: F401  (re-exported for older imports)
from tello_driver import model as M
from tello_driver.safety import CommandFilter
from tello_driver.zone import clearance, load_zone, wrap  # noqa: F401  (re-exported: missions import them here)


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
        # camera sees the markers; real.launch.py computes it from the marker map (band mode: the band middle)
        self.takeoff_height = p("takeoff_height", 0.0).value
        # highest allowed height above the floor (m): ceiling 2.80 m in the room, 2.60 m max (user, Nov 9)
        self.max_height = p("max_height", 2.6).value
        # obstacle-free flight zone from the room scan (tools/free_zone.py): commands that would take the drone
        # out of it are cut, whatever program sends them (Nov 10: a test drifted 1 m back towards furniture)
        self.zone = load_zone(p("flight_zone_file", "").value)
        if self.zone:
            self.get_logger().info("flight zone x [{x_min:.2f}, {x_max:.2f}] y [{y_min:.2f}, {y_max:.2f}]".format(**self.zone))
        # the hold and the guards (safety.py), on the identified model (Nov 11)
        self.safety = CommandFilter(
            zone=self.zone, lin_gain=self.lin_gain, yaw_gain=self.yaw_gain, max_rc=self.max_rc,
            max_height=self.max_height, model_Kx=p("model_K_x", M.K_X).value, model_Ky=p("model_K_y", M.K_Y).value,
            model_tau=p("model_tau", M.TAU_XY).value, model_Lu=p("model_L_u", M.L_U).value,
            model_Kz=p("model_K_z", M.K_Z).value,
            min_wall=p("min_wall_distance", 0.7).value, wall_band=p("wall_slow_band", 0.15).value,
            blind_speed=p("blind_speed", 0.15).value, hold_enabled=p("position_hold", True).value,
            hold_kp=p("hold_kp", 0.8).value, hold_kv=p("hold_kv", 0.4).value, hold_ki=p("hold_ki", 0.15).value,
            hold_vmax=p("hold_vmax", 0.4).value, hold_z=p("height_hold", True).value,
            hold_kpz=p("hold_kp_z", 1.0).value, hold_drift_ff=p("hold_drift_ff", True).value,
            warn=lambda m: self.get_logger().warn(m))

        # the hold gains can be changed in flight: ros2 param set /tello_driver hold_kp 1.0
        def on_params(params):
            from rcl_interfaces.msg import SetParametersResult
            for prm in params:
                if self.safety.set_param(prm.name, prm.value):
                    self.get_logger().info(f"{prm.name} = {prm.value}")
            return SetParametersResult(successful=True)
        self.add_on_set_parameters_callback(on_params)
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
        self.pub_sent = self.create_publisher(Twist, "/tello/rc_sent", 10)
        self.create_subscription(Twist, "/tello/cmd_vel_safe", self.on_cmd, 10)
        # pose for the hold and the guards: the EKF (smooth, predicted between detections, delay compensated)
        self.create_subscription(PoseWithCovarianceStamped, p("pose_topic", "/ekf/pose").value, self.on_pose, 10)
        self.create_subscription(TwistStamped, "/ekf/velocity",
                                 lambda m: self.safety.on_ekf_velocity(m.twist.linear.x, m.twist.linear.y, m.twist.linear.z), 10)
        self.create_subscription(Vector3Stamped, "/ekf/drift", lambda m: self.safety.on_ekf_drift(m.vector.x, m.vector.y), 10)
        # wall ahead, from the size of the closest marker in view (works without map and without pose)
        self.create_subscription(Float32, "/aruco/nearest_marker", lambda m: self.safety.on_nearest_marker(m.data), 5)
        self.create_service(Trigger, "~/takeoff", self.srv_takeoff)
        self.create_service(Trigger, "~/land", self.srv_land)
        self.create_service(Trigger, "~/emergency", self.srv_emergency)

        if self.mock:
            # with a marker map, the fake drone renders the markers and flies with the rc commands
            self.tello = FakeTello(p("mock_markers_file", "").value)
            if self.zone and self.tello.world is not None and not os.environ.get("FAKE_START"):
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
        # Video reader: djitellopy by default again (Nov 13: video_reader.py failed to open the real stream, InvalidDataError
        # in a loop; low_latency_video:=true to use it once fixed). Why it was tried (Nov 12): djitellopy decodes AND converts
        # every image with FFmpeg's large UDP buffer, so with a busy CPU the images pile up and the delay keeps
        # growing (a minute seen in RViz). Ours drops late images to catch up (synthetic stream, decoder at ~16 fps:
        # 0.2-0.6 s instead of 15 s and growing), but only tested on a synthetic stream.
        self.low_latency_video = p("low_latency_video", False).value and not self.mock
        if self.low_latency_video:
            from tello_driver.video_reader import LowLatencyReader
            self.reader = LowLatencyReader(log=lambda m: self.get_logger().warn(m, throttle_duration_sec=2.0))
        else:
            self.reader = self.tello.get_frame_read()
        self.get_logger().info("video reader: " + ("low latency (video_reader.py)" if self.low_latency_video
                                                   else "fake" if self.mock else "djitellopy"))
        self.video_count, self.video_fps, self.video_t0 = 0, 0.0, time.time()

        self.info = None
        self.last_frame = None
        self.rc_target = (0, 0, 0, 0)
        self.last_cmd = 0.0
        self.busy = threading.Lock()
        self.state = {}
        self.last_frame_t = time.time()
        self.video_timeout = p("video_timeout", 3.0).value
        self.restarting = False
        self.create_timer(1 / 100, self.poll_video)
        self.create_timer(0.1, self.poll_state)
        self.create_timer(0.05, self.send_rc)
        self.create_timer(1.0, self.video_watchdog)

    # ---------------------------------------------------------------- link
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

    def video_watchdog(self):
        """The Wi-Fi/WSL route to the drone can drop for a few seconds (Windows re-checks the network ~20 s
        after joining the Tello's Wi-Fi). djitellopy's reader blocks on its UDP port 11111 with no timeout: if
        its thread is still alive it resumes by itself once packets come back, so only 'streamon' is re-sent.
        A NEW reader while the old one holds the port fails with 'Address already in use' (Nov 9): a new one is
        made only if the old thread is dead, after closing its container."""
        if self.mock or self.restarting or time.time() - self.last_frame_t < self.video_timeout:
            return
        if self.low_latency_video:
            # the reader reopens the stream by itself; the drone may only need to be told to stream again
            self.get_logger().warn(f"No video for {time.time() - self.last_frame_t:.0f} s: re-sending streamon",
                                   throttle_duration_sec=2.0)
            try:
                self.tello.send_command_without_return("streamon")
            except Exception:  # noqa: BLE001
                pass
            self.last_frame_t = time.time()
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
        self.video_count += 1
        if self.last_frame_t - self.video_t0 >= 2.0:
            self.video_fps = self.video_count / (self.last_frame_t - self.video_t0)
            self.video_count, self.video_t0 = 0, self.last_frame_t
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

    # ---------------------------------------------------------------- telemetry
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
        st["video_fps"] = round(self.video_fps, 1)  # images published per second (the Tello sends 30 at most)
        st["hold"] = self.safety.hold_target is not None  # position hold active (no horizontal command): flight_report
        st["guard"] = self.safety.guard_now()               # the guard that changed the command, if any
        self.pub_tel.publish(String(data=json.dumps(st)))
        bat = self.state.get("bat")
        if self.tello.is_flying and isinstance(bat, (int, float)) and bat <= self.low_battery:
            self.get_logger().warn(f"Battery {bat} %: landing")
            self.run_async(self.tello.land, "low-battery land")

    # ---------------------------------------------------------------- commands
    def on_pose(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.safety.on_pose(p.x, p.y, p.z, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
                            math.sqrt(max(m.pose.covariance[0], m.pose.covariance[7], 0.0)),
                            math.sqrt(max(m.pose.covariance[14], 0.0)))

    def on_cmd(self, m):
        clip = self.safety.clip
        # Tello rc: (left/right, forward/back, up/down, yaw), right/forward/up/clockwise positive.
        # ROS body frame: x forward, y LEFT, z up, yaw counter-clockwise positive.
        self.rc_target = (clip(-m.linear.y / self.lin_gain), clip(m.linear.x / self.lin_gain),
                          clip(m.linear.z / self.lin_gain), clip(-m.angular.z / self.yaw_gain))
        self.last_cmd = time.time()

    def send_rc(self):
        """20 Hz: the requested command (zero after cmd_timeout), the climb after takeoff, then the safety chain
        (safety.py), then the Tello. The command really sent is published on /tello/rc_sent for the EKF."""
        if self.busy.locked():
            return  # a takeoff / climb / land command is running: rc would interfere with it
        rc = self.rc_target if time.time() - self.last_cmd < self.cmd_timeout else (0, 0, 0, 0)
        flying = self.tello.is_flying
        hgt, vz = self.safety.height_and_rate(self.state.get("tof"), flying)
        if self.climb_target is not None and flying:
            ud = self.climb_rc(hgt, vz)
            if ud is not None:
                rc = (0, 0, ud, 0)  # climbing: other commands wait (the position hold keeps it in place)
        rc = self.safety.filter(rc, hgt, vz, flying, busy=self.busy.locked(), climbing=self.climb_target is not None)
        try:
            self.tello.send_rc_control(*rc)  # also keeps the link alive (the Tello lands after 15 s of silence)
            t = Twist()
            t.linear.x, t.linear.y, t.linear.z = rc[1] * self.lin_gain, -rc[0] * self.lin_gain, rc[2] * self.lin_gain
            t.angular.z = -rc[3] * self.yaw_gain
            self.pub_sent.publish(t)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"rc failed: {e}", throttle_duration_sec=2.0)

    def stop_rc(self):
        """The Tello keeps flying with the LAST rc command while no new one comes, and none comes while a takeoff /
        land command runs (flight 225048: after 'g' it flew on backwards at 0.3 m/s for 5 s, 1.2 m, before the
        landing started). Zero it before such a command."""
        try:
            self.tello.send_rc_control(0, 0, 0, 0)
            self.pub_sent.publish(Twist())
        except Exception:  # noqa: BLE001
            pass

    def run_async(self, fn, label):
        """Blocking SDK command (takeoff / land) in a thread; 'busy' stops the rc commands meanwhile."""
        def worker():
            if not self.busy.acquire(blocking=False):
                return
            self.stop_rc()
            try:
                self.get_logger().info(label)
                fn()
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"{label} failed: {e}")
            finally:
                self.busy.release()
        threading.Thread(target=worker, daemon=True).start()

    # ---------------------------------------------------------------- takeoff / landing
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
        """Takeoff (SDK command, the Tello stops at ~0.8 m), then the climb is done by send_rc with joystick
        commands, closed-loop on the measured height. The SDK 'up' command was refused right after takeoff ('error
        Not joystick', Nov 9), and 'h' (barometer) read 20 cm at 80 cm: distance sensor + EKF height only."""
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

    def srv_land(self, req, res):
        self.rc_target = (0, 0, 0, 0)
        self.climb_target = None
        if self.busy.locked():
            # another command (e.g. a takeoff still waiting for its reply) holds the lock: a landing order must
            # never be dropped, so send it right away without waiting for an answer
            self.get_logger().warn("land while another command is running: sending 'land' immediately")
            self.stop_rc()
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
            if self.low_latency_video:
                self.reader.stop()
            elif self.tello.background_frame_read is not None:
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
