"""Compare /aruco/pose with the simulator ground truth and write a summary.

For each ArUco pose (stamped at frame arrival, like the real driver), the error is computed twice:
  - against ground truth at the stamp            -> what a naive filter would see (includes latency)
  - against ground truth at stamp - true latency -> pure geometric accuracy of the localizer
"""
import bisect
import csv
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Eval(Node):
    def __init__(self):
        super().__init__("eval_localization")
        self.latency = self.declare_parameter("latency", 0.20).value
        self.out_dir = Path(self.declare_parameter("out_dir", str(Path.home() / "tello_ws/sim_out")).value)
        self.gt_t, self.gt = [], []
        self.rows = []
        self.images = 0
        self.create_subscription(Odometry, "/sim/ground_truth", self.on_gt, 50)
        self.create_subscription(PoseWithCovarianceStamped, "/aruco/pose", self.on_pose, 20)
        from sensor_msgs.msg import Image
        self.create_subscription(Image, "/tello/image_raw", lambda m: setattr(self, "images", self.images + 1), 5)
        self.create_timer(5.0, self.report)

    def on_gt(self, m):
        p = m.pose.pose
        self.gt_t.append(Time.from_msg(m.header.stamp).nanoseconds * 1e-9)
        self.gt.append((p.position.x, p.position.y, p.position.z, yaw_of(p.orientation)))

    def gt_at(self, t):
        i = bisect.bisect_left(self.gt_t, t)
        if i <= 0 or i >= len(self.gt_t):
            return None
        return self.gt[i] if abs(self.gt_t[i] - t) < abs(self.gt_t[i - 1] - t) else self.gt[i - 1]

    def on_pose(self, m):
        t = Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        p = m.pose.pose
        est = (p.position.x, p.position.y, p.position.z, yaw_of(p.orientation))
        g_now, g_cap = self.gt_at(t), self.gt_at(t - self.latency)
        if g_now is None or g_cap is None:
            return
        def err(g):
            d = math.dist(est[:3], g[:3])
            dy = math.degrees(math.atan2(math.sin(est[3] - g[3]), math.cos(est[3] - g[3])))
            return d, dy
        (dn, yn), (dc, yc) = err(g_now), err(g_cap)
        self.rows.append({"t": round(t, 3), "x": est[0], "y": est[1], "z": est[2], "yaw_deg": math.degrees(est[3]),
                          "gt_x": g_cap[0], "gt_y": g_cap[1], "gt_z": g_cap[2],
                          "err_pos_m": dc, "err_yaw_deg": yc, "err_pos_vs_now_m": dn, "err_yaw_vs_now_deg": yn,
                          "sigma_pos_m": math.sqrt(m.pose.covariance[0])})

    def stats(self):
        if not self.rows:
            return None
        def s(key):
            v = sorted(abs(r[key]) for r in self.rows)
            return {"rmse": round(math.sqrt(sum(x * x for x in v) / len(v)), 4),
                    "median": round(v[len(v) // 2], 4), "p95": round(v[int(0.95 * (len(v) - 1))], 4),
                    "max": round(v[-1], 4)}
        return {"poses": len(self.rows), "images": self.images,
                "pose_rate_vs_images": round(len(self.rows) / max(1, self.images), 3),
                "position_error_m": s("err_pos_m"), "yaw_error_deg": s("err_yaw_deg"),
                "position_error_if_latency_ignored_m": s("err_pos_vs_now_m")}

    def report(self):
        st = self.stats()
        if st:
            self.get_logger().info(f"{st['poses']} poses, pos RMSE {st['position_error_m']['rmse'] * 100:.1f} cm, "
                                   f"yaw RMSE {st['yaw_error_deg']['rmse']:.2f} deg, "
                                   f"(ignoring latency: {st['position_error_if_latency_ignored_m']['rmse'] * 100:.1f} cm)")

    def save(self):
        st = self.stats()
        if not st:
            print("eval_localization: no pose received")
            return
        self.out_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        with open(self.out_dir / f"eval_{stamp}.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            w.writeheader()
            w.writerows(self.rows)
        (self.out_dir / f"eval_{stamp}.json").write_text(json.dumps(st, indent=2))
        (self.out_dir / "eval_latest.json").write_text(json.dumps(st, indent=2))
        print("EVAL SUMMARY " + json.dumps(st))


def main():
    rclpy.init()
    node = Eval()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
