"""Record a simulated flight for videos: annotated Tello frames, fixed-camera frames, ground truth,
ArUco poses and the test-flight step, all stamped in simulation time.

    <out_dir>/tello/<ms>.jpg   /aruco/debug_image (Tello camera with detected markers)
    <out_dir>/logitech/<ms>.jpg  /logitech/image_raw (fixed camera)
    <out_dir>/gt.csv, est.csv, events.csv
"""
import csv
import math
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Recorder(Node):
    def __init__(self):
        super().__init__("record_flight")
        self.out = Path(self.declare_parameter("out_dir", str(Path.home() / "tello_ws/sim_out/rec")).value)
        for d in ("tello", "logitech"):
            (self.out / d).mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.files = {k: open(self.out / f"{k}.csv", "w", newline="") for k in ("gt", "est", "events")}
        self.w = {k: csv.writer(f) for k, f in self.files.items()}
        self.w["gt"].writerow(["t", "x", "y", "z", "yaw"])
        self.w["est"].writerow(["t", "x", "y", "z", "yaw", "sigma"])
        self.w["events"].writerow(["t", "event"])
        self.last_t = 0.0
        self.create_subscription(Image, "/aruco/debug_image", lambda m: self.save_img(m, "tello"), 5)
        self.create_subscription(Image, "/logitech/image_raw", lambda m: self.save_img(m, "logitech"), 5)
        self.create_subscription(Odometry, "/sim/ground_truth", self.on_gt, 50)
        self.create_subscription(PoseWithCovarianceStamped, "/aruco/pose", self.on_est, 20)
        self.create_subscription(String, "/test_flight/status", self.on_event, 10)

    @staticmethod
    def t_of(msg):
        return Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

    def save_img(self, msg, kind):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(str(self.out / kind / f"{int(self.t_of(msg) * 1000):08d}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 90])

    def on_gt(self, m):
        p = m.pose.pose
        t = self.t_of(m)
        self.last_t = t
        self.w["gt"].writerow([f"{t:.3f}", p.position.x, p.position.y, p.position.z, yaw_of(p.orientation)])

    def on_est(self, m):
        p = m.pose.pose
        self.w["est"].writerow([f"{self.t_of(m):.3f}", p.position.x, p.position.y, p.position.z,
                                yaw_of(p.orientation), math.sqrt(m.pose.covariance[0])])

    def on_event(self, m):
        self.w["events"].writerow([f"{self.last_t:.3f}", m.data])

    def close(self):
        for f in self.files.values():
            f.close()


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
