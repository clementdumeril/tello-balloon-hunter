"""Record what happened during a real-drone session, to analyse it afterwards (no description needed).

Writes ~/tello_ws/sessions/<time>/:
  frames/<t>.jpg    the annotated camera image (/aruco/debug_image), at most one per second
  poses.csv         every ArUco pose (t, x, y, z, yaw, sigma)
  telemetry.csv     battery, height, temperature... at 2 Hz
  summary.json      counts, how often markers were seen, pose spread, battery, duration
"""
import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class SessionRecorder(Node):
    def __init__(self):
        super().__init__("session_recorder")
        root = Path(self.declare_parameter("out_dir", str(Path.home() / "tello_ws/sessions")).value)
        self.out = root / datetime.now().strftime("%Y%m%d_%H%M%S")
        (self.out / "frames").mkdir(parents=True, exist_ok=True)
        self.t0 = time.time()
        self.n_img = self.n_saved = 0
        self.last_save = 0.0
        self.poses = []
        self.tel_last = 0.0
        self.tel = []
        self.pose_f = open(self.out / "poses.csv", "w", newline="")
        self.pose_w = csv.writer(self.pose_f)
        self.pose_w.writerow(["t", "x", "y", "z", "yaw_deg", "sigma_m"])
        self.tel_f = open(self.out / "telemetry.csv", "w", newline="")
        self.tel_w = None
        self.create_subscription(Image, "/aruco/debug_image", self.on_img, 1)
        self.create_subscription(PoseWithCovarianceStamped, "/aruco/pose", self.on_pose, 20)
        self.create_subscription(String, "/tello/telemetry", self.on_tel, 10)
        # for EKF replays: the commands really sent and the EKF estimate
        self.sent_f = open(self.out / "rc_sent.csv", "w", newline="")
        self.sent_w = csv.writer(self.sent_f)
        self.sent_w.writerow(["t", "cx", "cy", "cz", "cw"])
        self.create_subscription(Twist, "/tello/rc_sent", lambda m: self.sent_w.writerow(
            [f"{time.time():.4f}", f"{m.linear.x:.3f}", f"{m.linear.y:.3f}", f"{m.linear.z:.3f}", f"{m.angular.z:.3f}"]), 50)
        # what the pilot / the mission asked for, before the driver's hold and guards (compare with rc_sent)
        self.cmd_f = open(self.out / "cmd_in.csv", "w", newline="")
        self.cmd_w = csv.writer(self.cmd_f)
        self.cmd_w.writerow(["t", "cx", "cy", "cz", "cw"])
        self.create_subscription(Twist, "/tello/cmd_vel_safe", lambda m: self.cmd_w.writerow(
            [f"{time.time():.4f}", f"{m.linear.x:.3f}", f"{m.linear.y:.3f}", f"{m.linear.z:.3f}", f"{m.angular.z:.3f}"]), 50)
        self.ekf_f = open(self.out / "ekf.csv", "w", newline="")
        self.ekf_w = csv.writer(self.ekf_f)
        self.ekf_w.writerow(["t", "x", "y", "z", "sigma_xy"])
        self.create_subscription(PoseWithCovarianceStamped, "/ekf/pose", lambda m: self.ekf_w.writerow(
            [f"{time.time():.4f}", f"{m.pose.pose.position.x:.4f}", f"{m.pose.pose.position.y:.4f}",
             f"{m.pose.pose.position.z:.4f}", f"{max(m.pose.covariance[0], m.pose.covariance[7]) ** 0.5:.4f}"]), 50)
        # flush every 2 s: when the launch is killed, close() never runs (flight 225048 lost its last 40 s of poses)
        self.create_timer(2.0, lambda: [f.flush() for f in (self.pose_f, self.tel_f, self.sent_f, self.cmd_f, self.ekf_f)])
        self.get_logger().info(f"Recording the session in {self.out}")

    def on_img(self, m):
        self.n_img += 1
        now = time.time()
        if now - self.last_save < 1.0:
            return
        self.last_save = now
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        if m.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(self.out / "frames" / f"{now - self.t0:06.1f}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        self.n_saved += 1

    def on_pose(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        row = [round(time.time() - self.t0, 3), p.x, p.y, p.z, yaw, math.sqrt(max(m.pose.covariance[0], 0))]
        self.poses.append(row)
        self.pose_w.writerow([f"{v:.4f}" if isinstance(v, float) else v for v in row])

    def on_tel(self, m):
        now = time.time()
        if now - self.tel_last < 0.5:
            return
        self.tel_last = now
        st = json.loads(m.data)
        st["t"] = round(now - self.t0, 2)
        self.tel.append(st)
        if self.tel_w is None:
            self.tel_w = csv.DictWriter(self.tel_f, fieldnames=list(st.keys()), extrasaction="ignore")
            self.tel_w.writeheader()
        self.tel_w.writerow(st)

    def close(self):
        dur = time.time() - self.t0
        s = {"duration_s": round(dur, 1), "debug_images": self.n_img, "debug_image_hz": round(self.n_img / max(dur, 1e-3), 1),
             "frames_saved": self.n_saved, "poses": len(self.poses),
             "pose_rate_vs_debug_images": round(len(self.poses) / max(self.n_img * 3, 1), 2)}
        if self.poses:
            a = np.array(self.poses)[:, 1:4]
            s.update(first_pose_s=self.poses[0][0], pose_mean=np.round(a.mean(0), 2).tolist(),
                     pose_min=np.round(a.min(0), 2).tolist(), pose_max=np.round(a.max(0), 2).tolist())
        if self.tel:
            s.update(battery_start=self.tel[0].get("bat"), battery_end=self.tel[-1].get("bat"),
                     temp_max=max(t.get("temph", 0) for t in self.tel))
        (self.out / "summary.json").write_text(json.dumps(s, indent=1))
        self.pose_f.close()
        self.tel_f.close()
        self.sent_f.close()
        self.cmd_f.close()
        self.ekf_f.close()


def main():
    rclpy.init()
    node = SessionRecorder()
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
