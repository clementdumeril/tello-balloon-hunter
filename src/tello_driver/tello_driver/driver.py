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
    """Without a marker map: a plain moving test pattern. With one: the virtual camera view of the markers."""
    def __init__(self, world=None):
        self.world = world
        self._t = -1
        self._frame = None

    @property
    def frame(self):
        k = int(time.time() * 30)
        if k != self._t:
            self._t = k
            if self.world is not None:
                self._frame = self.world.render()
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
        self.max_rc = int(p("max_rc", 60).value)
        self.cmd_timeout = p("cmd_timeout", 0.5).value
        self.low_battery = p("low_battery", 12).value
        # after the takeoff (~0.8 m), climb to this height (m above the takeoff point, 0 = stay) so that the
        # camera sees the markers; real.launch.py computes it from the marker map
        self.takeoff_height = p("takeoff_height", 0.0).value
        # highest allowed height above the floor (m): ceiling 2.80 m in the room, 2.60 m max (user, Nov 9)
        self.max_height = p("max_height", 2.6).value
        self.pose_z, self.pose_t = 0.0, 0.0

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
        self.create_subscription(Twist, "/tello/cmd_vel_safe", self.on_cmd, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/aruco/pose", self.on_pose, 10)
        self.create_service(Trigger, "~/takeoff", self.srv_takeoff)
        self.create_service(Trigger, "~/land", self.srv_land)
        self.create_service(Trigger, "~/emergency", self.srv_emergency)

        if self.mock:
            # with a marker map, the fake drone renders the markers and flies with the rc commands
            self.tello = FakeTello(p("mock_markers_file", "").value)
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
        after joining the Tello's Wi-Fi). That kills djitellopy's decoder thread for good: restart the stream."""
        if self.mock or self.restarting or time.time() - self.last_frame_t < self.video_timeout:
            return
        self.restarting = True
        self.get_logger().warn(f"No video for {time.time() - self.last_frame_t:.0f} s: restarting the stream")

        def restart():
            try:
                try:
                    self.tello.streamoff()
                except Exception:  # noqa: BLE001
                    self.tello.background_frame_read = None
                time.sleep(0.5)
                self.tello.streamon()
                self.reader = self.tello.get_frame_read()
                self.get_logger().info("Video stream restarted")
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"Video restart failed ({e}): retrying", throttle_duration_sec=5.0)
            finally:
                self.last_frame_t = time.time()  # give the new stream time to start
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
                self.tello.world.set_camera(self.info.k, self.info.d)  # render with the same intrinsics
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
        st["busy"] = self.busy.locked()  # takeoff / climb / land running: not ready for velocity commands
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
        # ceiling guard, whatever program sends the commands: no climbing in the last 20 cm, descend above
        hgt = self.height_estimate() if self.tello.is_flying else None
        if hgt is not None:
            lr, fb, ud, yw = rc
            if hgt > self.max_height:
                ud = -25
                self.get_logger().warn(f"height {hgt:.2f} m > {self.max_height:.2f} m: descending",
                                       throttle_duration_sec=1.0)
            elif hgt > self.max_height - 0.2 and ud > 0:
                ud = 0
                self.get_logger().warn(f"height {hgt:.2f} m: close to the {self.max_height:.2f} m limit, no climbing",
                                       throttle_duration_sec=2.0)
            rc = (lr, fb, ud, yw)
        try:
            self.tello.send_rc_control(*rc)  # also keeps the link alive (the Tello lands after 15 s of silence)
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
        self.tello.takeoff()
        target = min(self.takeoff_height, self.max_height - 0.6)
        if self.takeoff_height <= 0:
            return
        # Measure the real height, NOT the 'h' field: right after takeoff 'h' (barometer) read 20 cm while the
        # drone was at 80 cm (Nov 9), and the climb went up to the ceiling. Distance sensor + ArUco pose instead.
        time.sleep(1.0)
        samples = []
        t0 = time.time()
        while time.time() - t0 < 1.0:
            hgt = self.height_estimate()
            if hgt is not None:
                samples.append(hgt)
            time.sleep(0.1)
        if not samples:
            self.get_logger().warn("no valid height measurement after takeoff: not climbing")
            return
        hgt = float(np.median(samples))
        delta = int(round((target - hgt) * 100))
        delta = min(delta, 100)  # never more than 1 m in one go
        if delta >= 20:  # the SDK refuses moves under 20 cm
            self.get_logger().info(f"at {hgt:.2f} m: climbing {delta} cm towards {target:.2f} m (markers in view)")
            self.tello.move_up(delta)
        else:
            self.get_logger().info(f"at {hgt:.2f} m after takeoff: no climb needed (target {target:.2f} m)")

    def on_pose(self, m):
        self.pose_z, self.pose_t = m.pose.pose.position.z, time.time()

    def height_estimate(self):
        """Height above the floor (m): the larger of the distance sensor and a fresh ArUco pose (conservative
        for the ceiling; the sensor reads less over the bed). None if neither is valid."""
        c = []
        tof = self.state.get("tof")
        if isinstance(tof, (int, float)) and 15 <= tof <= 400:
            c.append(tof / 100.0)
        if time.time() - self.pose_t < 0.5:
            c.append(self.pose_z)
        return max(c) if c else None

    def srv_land(self, req, res):
        self.rc_target = (0, 0, 0, 0)
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
