"""Drone state EKF node (model in ekf_core.py).

Subscribes  /aruco/pose (ArUco, delayed ~L_v), /tello/telemetry (distance sensor, IMU heading),
            /tello/rc_sent (the velocity command the driver really sent, after its hold and guards)
Publishes   /ekf/pose (PoseWithCovarianceStamped, map, 30 Hz), /ekf/velocity (TwistStamped, map frame),
            /ekf/path, TF map -> base_link (the drone in RViz follows the EKF, also between marker detections)
"""
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from tello_driver.ekf_core import IB, IDX, IDY, IVX, IVY, IVZ, IX, IY, IYAW, IZ, DroneEKF


def quat_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class EkfNode(Node):
    def __init__(self):
        super().__init__("tello_ekf")
        p = self.declare_parameter
        self.ekf = DroneEKF(K_xy=p("K_xy", 1.15).value, tau_xy=p("tau_xy", 0.55).value, K_z=p("K_z", 0.9).value,
                            tau_z=p("tau_z", 0.45).value, L_u=p("L_u", 0.30).value, L_v=p("L_v", 0.20).value,
                            accel_noise=p("accel_noise", 0.6).value, drift_noise=p("drift_noise", 0.01).value)
        self.child = p("child_frame", "base_link").value
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, "/ekf/pose", 10)
        self.pub_vel = self.create_publisher(TwistStamped, "/ekf/velocity", 10)
        self.pub_path = self.create_publisher(PathMsg, "/ekf/path", 2)
        self.tf = TransformBroadcaster(self)
        self.path = PathMsg()
        self.path.header.frame_id = "map"
        self.last_path = 0.0
        self.create_subscription(PoseWithCovarianceStamped, "/aruco/pose", self.on_pose, 20)
        self.create_subscription(String, "/tello/telemetry", self.on_tel, 10)
        self.create_subscription(Twist, "/tello/rc_sent", self.on_cmd, 20)
        self.create_timer(1 / 30, self.publish)
        self.n_pose = self.n_rej = 0
        self.create_timer(10.0, self.report)

    def now(self):
        return time.time()

    def on_cmd(self, m):
        t = self.now()
        self.ekf.predict(t)
        self.ekf.add_command(t, [m.linear.x, m.linear.y, m.linear.z, m.angular.z])

    def on_pose(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        sp = math.sqrt(max(m.pose.covariance[0], 1e-6))
        sy = math.sqrt(max(m.pose.covariance[35], 1e-8))
        ok = self.ekf.update_pose(self.now(), [p.x, p.y, p.z, yaw], sp, sy)
        self.n_pose += 1
        self.n_rej += 0 if ok else 1

    def on_tel(self, m):
        try:
            st = json.loads(m.data)
        except ValueError:
            return
        self.ekf.predict(self.now())
        tof = st.get("tof")
        if isinstance(tof, (int, float)) and 15 <= tof <= 400 and st.get("flying"):
            self.ekf.update_height(tof / 100.0)
        if isinstance(st.get("yaw"), (int, float)):
            self.ekf.update_imu_yaw(-math.radians(st["yaw"]))  # Tello: clockwise-positive degrees

    def publish(self):
        e = self.ekf
        if not e.initialised:
            return
        e.predict(self.now())
        s, P = e.s, e.P
        stamp = self.get_clock().now().to_msg()
        m = PoseWithCovarianceStamped()
        m.header.stamp, m.header.frame_id = stamp, "map"
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = float(s[IX]), float(s[IY]), float(s[IZ])
        qx, qy, qz, qw = quat_yaw(s[IYAW])
        o = m.pose.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        cov = np.zeros((6, 6))
        cov[:3, :3] = P[:3, :3]
        cov[5, 5] = P[IYAW, IYAW]
        cov[3, 3] = cov[4, 4] = 1e-4
        m.pose.covariance = cov.ravel().tolist()
        self.pub_pose.publish(m)
        v = TwistStamped()
        v.header = m.header
        v.twist.linear.x, v.twist.linear.y, v.twist.linear.z = float(s[IVX]), float(s[IVY]), float(s[IVZ])
        self.pub_vel.publish(v)
        t = TransformStamped()
        t.header = m.header
        t.child_frame_id = self.child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, s[:3])
        t.transform.rotation = m.pose.pose.orientation
        self.tf.sendTransform(t)
        if self.now() - self.last_path > 0.2:
            ps = PoseStamped()
            ps.header, ps.pose = m.header, m.pose.pose
            self.path.poses.append(ps)
            del self.path.poses[:-3000]
            self.path.header.stamp = stamp
            self.pub_path.publish(self.path)
            self.last_path = self.now()

    def report(self):
        if self.ekf.initialised:
            s = self.ekf.s
            self.get_logger().info(f"sigma_xy {self.ekf.sigma_xy() * 100:.0f} cm, drift ({s[IDX]:+.2f}, {s[IDY]:+.2f}) m/s, "
                                   f"IMU yaw offset {math.degrees(s[IB]):.0f} deg, poses {self.n_pose} ({self.n_rej} gated)")


def main():
    from tello_driver.ros_env import local_ros_env
    local_ros_env()
    rclpy.init()
    node = EkfNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
