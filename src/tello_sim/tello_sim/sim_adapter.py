"""Make the Gazebo drone look like the real Tello driver to the rest of the stack.

  /tello/cmd_vel_safe (Twist, body frame, m/s and rad/s)  --first-order lag, watchdog-->  /sim/tello/cmd_vel
  /sim/tello/image_raw + camera_info  --latency, jitter, frame drops-->  /tello/image_raw + camera_info
  /sim/ground_truth (Odometry)  -->  /tello/telemetry (JSON, Tello-like fields and units)
                                -->  TF map -> base_link_gt (ground truth, for RViz and evaluation)

Telemetry units mimic djitellopy's state dict. Velocity frame/sign conventions are ASSUMPTIONS
until task A6 measures them on the real drone: vgx forward, vgy right, vgz down, integer dm/s.
"""
import json
import math
import random
from collections import deque

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class SimAdapter(Node):
    def __init__(self):
        super().__init__("sim_adapter")
        p = self.declare_parameter
        self.latency = p("latency", 0.20).value
        self.jitter = p("latency_jitter", 0.03).value
        self.drop_prob = p("drop_prob", 0.03).value
        self.tau = p("tau", 0.30).value
        self.max_speed = p("max_speed", 1.0).value
        self.max_yaw_rate = p("max_yaw_rate", 1.5).value
        self.watchdog = p("cmd_timeout", 0.5).value
        self.yaw_offset = math.radians(p("yaw_offset_deg", 37.0).value)  # Tello yaw reference != map
        self.camera_frame = p("camera_frame", "tello_camera_optical").value

        self.cmd_target = [0.0, 0.0, 0.0, 0.0]
        self.cmd_state = [0.0, 0.0, 0.0, 0.0]
        self.last_cmd_t = None
        self.pending = deque()  # (release_time_ns, kind, msg)
        self.last_info = None
        self.battery = 95.0
        self.start_z = None

        self.pub_cmd = self.create_publisher(Twist, "/sim/tello/cmd_vel", 10)
        self.pub_img = self.create_publisher(Image, "/tello/image_raw", 5)
        self.pub_info = self.create_publisher(CameraInfo, "/tello/camera_info", 5)
        self.pub_tel = self.create_publisher(String, "/tello/telemetry", 10)
        self.tf = TransformBroadcaster(self)

        self.create_subscription(Twist, "/tello/cmd_vel_safe", self.on_cmd, 10)
        self.create_subscription(Image, "/sim/tello/image_raw", self.on_image, 5)
        self.create_subscription(CameraInfo, "/sim/tello/camera_info", self.on_info, 5)
        self.create_subscription(Odometry, "/sim/ground_truth", self.on_odom, 20)

        self.create_timer(0.02, self.step_cmd)
        self.create_timer(0.005, self.release)
        self.create_timer(0.1, self.publish_telemetry)
        self.odom = None
        self.get_logger().info(f"sim_adapter: latency {self.latency * 1000:.0f} ms, drops {self.drop_prob:.0%}, tau {self.tau} s")

    # ---------------------------------------------------------------- commands
    def on_cmd(self, m):
        c = lambda v, lim: max(-lim, min(lim, v))  # noqa: E731
        self.cmd_target = [c(m.linear.x, self.max_speed), c(m.linear.y, self.max_speed),
                           c(m.linear.z, self.max_speed), c(m.angular.z, self.max_yaw_rate)]
        self.last_cmd_t = self.get_clock().now()

    def step_cmd(self):
        now = self.get_clock().now()
        if self.last_cmd_t is None or (now - self.last_cmd_t).nanoseconds * 1e-9 > self.watchdog:
            self.cmd_target = [0.0, 0.0, 0.0, 0.0]  # real Tello hovers when commands stop
        a = min(1.0, 0.02 / self.tau)
        self.cmd_state = [s + a * (t - s) for s, t in zip(self.cmd_state, self.cmd_target)]
        if self.odom is not None and self.odom.pose.pose.position.z < 0.04 and self.cmd_state[2] < 0:
            self.cmd_state[2] = 0.0  # floor
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.z = self.cmd_state
        self.pub_cmd.publish(tw)

    # ---------------------------------------------------------------- video
    def on_info(self, m):
        self.last_info = m

    def on_image(self, m):
        if random.random() < self.drop_prob:
            return
        delay = max(0.0, self.latency + random.uniform(-self.jitter, self.jitter))
        release = self.get_clock().now().nanoseconds + int(delay * 1e9)
        self.pending.append((release, m))

    def release(self):
        now = self.get_clock().now()
        while self.pending and self.pending[0][0] <= now.nanoseconds:
            _, m = self.pending.popleft()
            # like the real driver: stamped when the frame reaches the PC, not when it was captured
            m.header.stamp = now.to_msg()
            m.header.frame_id = self.camera_frame
            self.pub_img.publish(m)
            if self.last_info is not None:
                info = self.last_info
                info.header.stamp = m.header.stamp
                info.header.frame_id = self.camera_frame
                self.pub_info.publish(info)

    # ---------------------------------------------------------------- telemetry
    def on_odom(self, m):
        self.odom = m
        if self.start_z is None:
            self.start_z = m.pose.pose.position.z
        t = TransformStamped()
        t.header = m.header
        t.header.frame_id = "map"
        t.child_frame_id = "base_link_gt"
        t.transform.translation.x = m.pose.pose.position.x
        t.transform.translation.y = m.pose.pose.position.y
        t.transform.translation.z = m.pose.pose.position.z
        t.transform.rotation = m.pose.pose.orientation
        self.tf.sendTransform(t)

    def publish_telemetry(self):
        if self.odom is None:
            return
        o = self.odom
        yaw = yaw_from_quat(o.pose.pose.orientation)
        # Gazebo odometry twist is expressed in the child (body) frame
        vx, vy, vz = o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z
        z = o.pose.pose.position.z
        self.battery = max(0.0, self.battery - 0.1 / 60.0 * 10)  # ~10 %/min
        rel_yaw = math.degrees(math.atan2(math.sin(yaw - self.yaw_offset), math.cos(yaw - self.yaw_offset)))
        state = {
            "pitch": 0, "roll": 0, "yaw": int(round(rel_yaw)),
            "vgx": int(round(vx * 10 + random.gauss(0, 0.3))),
            "vgy": int(round(-vy * 10 + random.gauss(0, 0.3))),
            "vgz": int(round(-vz * 10 + random.gauss(0, 0.3))),
            "templ": 60, "temph": 63,
            "tof": int(round(max(10.0, z * 100 + random.gauss(0, 1.0)))),
            "h": int(round((z - self.start_z) * 100 / 10)) * 10,
            "bat": int(self.battery), "baro": round(z + random.gauss(0, 0.05), 2), "time": 0,
            "agx": 0.0, "agy": 0.0, "agz": -1000.0,
            "stamp": Time.from_msg(o.header.stamp).nanoseconds * 1e-9,
        }
        self.pub_tel.publish(String(data=json.dumps(state)))


def main():
    rclpy.init()
    node = SimAdapter()
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
