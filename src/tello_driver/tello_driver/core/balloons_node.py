"""Balloons seen by the external camera, in the `map` frame, for the missions and RViz.

tools/ext_camera.py detect (Windows, the USB camera) sends UDP JSON to 127.0.0.1:9870 (WSL mirrored networking):
    {"t": <unix time>, "balloons": [{"colour": "red", "x": .., "y": .., "z": .., "dist": .., "score": ..}, ...]}
This node keeps, per colour, the detections of the last `window` seconds and publishes their median:
    /balloons          std_msgs/String JSON {"red": {"x", "y", "z", "age", "n", "stale"}, "blue": {...}}  (10 Hz)
    /balloons/markers  visualization_msgs/MarkerArray: spheres in RViz + the external camera position
A balloon, once seen, stays at its last position (the drone hides it, the camera loses it...): "stale" is true and
the sphere is pale after `max_age` s without detection. One balloon per colour (checkpoint 1: one red, one blue);
with several of one colour, the most confident detection of each image is kept.

    ros2 run tello_driver tello_balloons          (or real.launch.py balloons:=true)
"""
import json
import math
import os
import socket
import struct
import time
from pathlib import Path

import numpy as np
import rclpy
import rclpy.executors
import yaml
from geometry_msgs.msg import Point
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

RGB = {"red": (1.0, 0.1, 0.1), "blue": (0.1, 0.3, 1.0)}


class Balloons(Node):
    def __init__(self):
        super().__init__("tello_balloons")
        p = self.declare_parameter
        self.port = int(p("port", 9870).value)
        self.window = p("window", 0.5).value     # s of detections averaged (median): noise vs lag
        self.max_age = p("max_age", 1.0).value   # s without detection: "stale" (kept, drawn pale)
        self.diameter = p("diameter", 0.28).value
        repo = os.environ.get("TELLO_REPO", str(Path.home() / "tello_ws/src/tello-balloon-hunter"))
        self.cam_file = p("ext_camera_file", os.path.join(repo, "config", "ext_camera.yaml")).value
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.port))
        self.sock.setblocking(False)
        # the Logitech image with its detections (JPEG on port+1) -> /balloons/image, shown in RViz
        self.sock_img = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_img.bind(("0.0.0.0", self.port + 1))
        self.sock_img.setblocking(False)
        self.img_frame, self.img_parts = -1, {}
        self.pub_img = self.create_publisher(Image, "/balloons/image", 1)
        self.pub_info = self.create_publisher(CameraInfo, "/balloons/camera_info", 1)
        # the Logitech camera in the SAME TF tree as the drone: map -> ext_camera_optical (static, from its
        # self-calibration on the wall markers); the Tello camera is map -> base_link -> tello_camera_optical
        self.static_tf = StaticTransformBroadcaster(self)
        self.cal = None
        try:
            self.cal = yaml.safe_load(open(self.cam_file))
            self.static_tf.sendTransform(self.camera_tf())
        except OSError:
            self.get_logger().warn(f"no {self.cam_file}: the external camera is not in TF")
        self.hist = {c: [] for c in RGB}   # colour -> [(t_received, x, y, z)]
        self.last = {}                     # colour -> (t, x, y, z, n): last estimate, kept forever
        self.n_rx, self.t_rx = 0, 0.0
        self.pub = self.create_publisher(String, "/balloons", 10)
        self.pub_m = self.create_publisher(MarkerArray, "/balloons/markers", 10)
        self.create_timer(0.02, self.poll)
        self.create_timer(0.1, self.publish)
        self.create_timer(5.0, self.report)
        self.get_logger().info(f"listening for tools/ext_camera.py detect on UDP {self.port}")

    def poll_image(self):
        """The image comes in 1200-byte pieces (frame, index, count: 3 x uint16, then data); a complete frame
        is published, an incomplete one is dropped when the next frame starts."""
        jpg = None
        while True:
            try:
                pkt, _ = self.sock_img.recvfrom(65536)
            except BlockingIOError:
                break
            if len(pkt) < 6:
                continue
            f, k, n = struct.unpack("!HHH", pkt[:6])
            if f != self.img_frame:
                self.img_frame, self.img_parts = f, {}
            self.img_parts[k] = pkt[6:]
            if len(self.img_parts) == n:
                jpg = b"".join(self.img_parts[i] for i in range(n))
                self.img_parts = {}
        if jpg is None:
            return
        import cv2
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        m = Image()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), "ext_camera_optical"
        m.height, m.width = img.shape[:2]
        m.encoding, m.step = "bgr8", img.shape[1] * 3
        m.data = img.tobytes()
        self.pub_img.publish(m)
        if self.cal:  # intrinsics scaled to the sent image (1280x720 -> 640x360)
            s = m.width / float(self.cal["image_size"][0])
            K = np.array(self.cal["K"], float)
            K[:2] *= s
            info = CameraInfo()
            info.header = m.header
            info.width, info.height = m.width, m.height
            info.distortion_model = "plumb_bob"
            info.d = [float(v) for v in self.cal["D"]]
            info.k = K.ravel().tolist()
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [K[0, 0], 0.0, K[0, 2], 0.0, 0.0, K[1, 1], K[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]
            self.pub_info.publish(info)

    def camera_tf(self):
        """map -> ext_camera_optical: rotation R_cam_to_map (camera axes in the map), origin C_map."""
        R = np.array(self.cal["R_cam_to_map"], float)
        C = np.array(self.cal["C_map"], float)
        w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
        if w > 1e-3:
            q = ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w)
        else:  # 180 deg rotation: largest diagonal term
            i = int(np.argmax(np.diag(R)))
            j, k = (i + 1) % 3, (i + 2) % 3
            r = math.sqrt(max(0.0, 1.0 + R[i, i] - R[j, j] - R[k, k]))
            v = [0.0, 0.0, 0.0]
            v[i], v[j], v[k] = r / 2, (R[j, i] + R[i, j]) / (2 * r), (R[k, i] + R[i, k]) / (2 * r)
            q = (*v, (R[k, j] - R[j, k]) / (2 * r))
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = self.get_clock().now().to_msg(), "map", "ext_camera_optical"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, C)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, q)
        return t

    def poll(self):
        self.poll_image()
        while True:
            try:
                data, _ = self.sock.recvfrom(65536)
            except BlockingIOError:
                return
            try:
                msg = json.loads(data)
            except ValueError:
                continue
            now = time.time()
            self.n_rx, self.t_rx = self.n_rx + 1, now
            best = {}
            for b in msg.get("balloons", []):
                c = b.get("colour")
                if c in RGB and b.get("score", 0) >= best.get(c, {}).get("score", -1):
                    best[c] = b
            for c, b in best.items():
                self.hist[c].append((now, b["x"], b["y"], b["z"]))

    def estimate(self):
        """Median of the last `window` s per colour. A balloon does not move: once seen it STAYS at its last
        position (hidden by the drone, out of view...), with "stale": true after `max_age` s without detection."""
        now = time.time()
        for c, h in self.hist.items():
            h[:] = [e for e in h if now - e[0] < self.window]
            if h:
                P = np.median(np.array([e[1:] for e in h]), axis=0)
                self.last[c] = (h[-1][0], round(float(P[0]), 3), round(float(P[1]), 3), round(float(P[2]), 3), len(h))
        out = {}
        for c, (t, x, y, z, n) in self.last.items():
            age = now - t
            out[c] = {"x": x, "y": y, "z": z, "age": round(age, 2), "n": n, "stale": age > self.max_age}
        return out

    def publish(self):
        est = self.estimate()
        self.pub.publish(String(data=json.dumps(est)))
        ma = MarkerArray()
        for k, c in enumerate(RGB):
            m = Marker()
            m.header.frame_id, m.header.stamp = "map", self.get_clock().now().to_msg()
            m.ns, m.id, m.type = "balloons", k, Marker.SPHERE
            if c in est:
                e = est[c]
                m.action = Marker.ADD
                m.pose.position.x, m.pose.position.y, m.pose.position.z = e["x"], e["y"], e["z"]
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = self.diameter
                m.color.r, m.color.g, m.color.b = RGB[c]
                m.color.a = 0.3 if e["stale"] else 0.8   # pale: last known place, not seen right now
            else:
                m.action = Marker.DELETE
            ma.markers.append(m)
        cam = self.camera_marker()
        if cam is not None:
            ma.markers.append(cam)
        self.pub_m.publish(ma)

    def camera_marker(self):
        """The external camera (config/ext_camera.yaml): an arrow along its optical axis."""
        try:
            cal = yaml.safe_load(open(self.cam_file))
        except OSError:
            return None
        C = np.array(cal["C_map"])
        axis = np.array(cal["R_cam_to_map"]) @ np.array([0.0, 0.0, 1.0])
        m = Marker()
        m.header.frame_id, m.header.stamp = "map", self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = "ext_camera", 0, Marker.ARROW, Marker.ADD
        m.points = [Point(x=float(C[0]), y=float(C[1]), z=float(C[2])),
                    Point(x=float(C[0] + 0.4 * axis[0]), y=float(C[1] + 0.4 * axis[1]), z=float(C[2] + 0.4 * axis[2]))]
        m.scale.x, m.scale.y, m.scale.z = 0.04, 0.08, 0.1
        m.color.r = m.color.g = m.color.b = 0.9
        m.color.a = 1.0
        return m

    def report(self):
        if time.time() - self.t_rx > 5.0:
            self.get_logger().warn("nothing from tools/ext_camera.py for 5 s (is 'detect' running on Windows?)",
                                   throttle_duration_sec=30.0)
            return
        est = self.estimate()
        txt = ", ".join(f"{c} ({e['x']:+.2f}, {e['y']:+.2f}, {e['z']:.2f})" for c, e in est.items()) or "no balloon"
        self.get_logger().info(f"{txt}   [{self.n_rx} camera frames]")


def main():
    from tello_driver.core.ros_env import local_ros_env
    local_ros_env()
    rclpy.init()
    node = Balloons()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:  # noqa: BLE001
        pass


if __name__ == "__main__":
    main()
