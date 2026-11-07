"""Scripted test flight for the simulator: take off, fly a square with yaw changes, spin, land.

Uses the GROUND TRUTH pose to steer. This is a test driver for validating TF and localization,
not our controller (that is task F2, which will use the EKF).
Publishes /tello/cmd_vel_safe exactly like the real stack would.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.node import Node


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class TestFlight(Node):
    def __init__(self):
        super().__init__("test_flight")
        self.alt = self.declare_parameter("altitude", 1.2).value
        self.side = self.declare_parameter("square_side", 0.8).value
        self.vmax = self.declare_parameter("max_speed", 0.35).value
        self.balloon = self.declare_parameter("balloon_xy", [3.1, 2.6]).value
        self.status = self.create_publisher(String, "/test_flight/status", 10)
        self.pub = self.create_publisher(Twist, "/tello/cmd_vel_safe", 10)
        self.create_subscription(Odometry, "/sim/ground_truth", self.on_odom, 20)
        self.pose = None
        self.wps = None
        self.i = 0
        self.hold_until = None
        self.done = False
        self.create_timer(0.05, self.step)

    def on_odom(self, m):
        p = m.pose.pose
        yaw = math.atan2(2 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y),
                         1 - 2 * (p.orientation.y ** 2 + p.orientation.z ** 2))
        self.pose = (p.position.x, p.position.y, p.position.z, yaw)
        if self.wps is None:
            x0, y0, _, y_0 = self.pose
            s, h = self.side, self.alt
            d = math.pi / 2
            self.wps = [
                (x0, y0, h, y_0, "take off"),
                (x0 + s, y0, h, y_0, "square 1/4"),
                (x0 + s, y0 + s, h, y_0 + d, "square 2/4 (yaw +90)"),
                (x0, y0 + s, h, y_0 + 2 * d, "square 3/4 (yaw 180)"),
                (x0, y0, h, y_0 + 3 * d, "square 4/4 (yaw 270)"),
                (x0, y0, h - 0.4, y_0, "lower + yaw back"),
                (x0, y0, h + 0.3, y_0 - d, "higher + yaw -90"),
                (x0, y0, h, math.atan2(self.balloon[1] - y0, self.balloon[0] - x0), "face the balloon"),
                (x0, y0, 0.05, y_0, "land"),
            ]
            self.get_logger().info(f"Test flight with {len(self.wps)} waypoints from ({x0:.2f}, {y0:.2f})")

    def step(self):
        if self.pose is None or self.wps is None:
            return
        cmd = Twist()
        if self.i >= len(self.wps):
            self.pub.publish(cmd)
            if not self.done:
                self.done = True
                self.get_logger().info("Test flight finished")
                self.status.publish(String(data="finished"))
                raise SystemExit
            return
        x, y, z, yaw = self.pose
        tx, ty, tz, tyaw, label = self.wps[self.i]
        if getattr(self, "_announced", None) != self.i:
            self._announced = self.i
            self.status.publish(String(data=label))
        ex, ey, ez, eyaw = tx - x, ty - y, tz - z, wrap(tyaw - yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        vx, vy = 1.2 * (c * ex + s * ey), 1.2 * (-s * ex + c * ey)
        n = math.hypot(vx, vy)
        if n > self.vmax:
            vx, vy = vx * self.vmax / n, vy * self.vmax / n
        cmd.linear.x, cmd.linear.y = vx, vy
        cmd.linear.z = max(-self.vmax, min(self.vmax, 1.5 * ez))
        cmd.angular.z = max(-0.8, min(0.8, 1.5 * eyaw))
        self.pub.publish(cmd)

        reached = math.sqrt(ex * ex + ey * ey + ez * ez) < 0.06 and abs(eyaw) < math.radians(4)
        now = self.get_clock().now().nanoseconds * 1e-9
        if reached and self.hold_until is None:
            self.hold_until = now + 1.5
        if self.hold_until is not None and now >= self.hold_until:
            self.get_logger().info(f"reached: {label}")
            self.i += 1
            self.hold_until = None


def main():
    rclpy.init()
    node = TestFlight()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
