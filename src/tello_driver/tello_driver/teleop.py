"""Keyboard teleop for the Tello in a terminal (same keys as scripts/first_flight.py).

    ros2 run tello_driver tello_teleop

  t take off   g land   arrows move   i/k up/down   j/l yaw   space stop
  x EMERGENCY (motors off)   q quit (lands first)
Each key press moves for `hold` seconds (a terminal cannot see key releases): keep the key
pressed to keep moving; the driver's watchdog stops the drone if nothing arrives.
"""
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import Trigger

HELP = __doc__.split("\n\n")[1]


class Teleop(Node):
    def __init__(self):
        super().__init__("tello_teleop")
        self.speed = self.declare_parameter("speed", 0.35).value      # m/s
        self.yaw_rate = self.declare_parameter("yaw_rate", 0.6).value  # rad/s
        self.hold = self.declare_parameter("hold", 0.35).value         # s
        self.pub = self.create_publisher(Twist, "/tello/cmd_vel_safe", 10)
        self.cli = {n: self.create_client(Trigger, f"/tello_driver/{n}") for n in ("takeoff", "land", "emergency")}
        self.cmd = Twist()
        self.until = 0.0

    def call(self, name):
        c = self.cli[name]
        if not c.wait_for_service(timeout_sec=1.0):
            print(f"\r\n/tello_driver/{name} not available: is the driver running?\r")
            return
        fut = c.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if fut.result() is not None:
            print(f"\r\n{name}: {fut.result().message}\r")

    def key(self, k):
        t = Twist()
        s, w = self.speed, self.yaw_rate
        moves = {"UP": ("x", s), "DOWN": ("x", -s), "LEFT": ("y", s), "RIGHT": ("y", -s),
                 "i": ("z", s), "k": ("z", -s)}
        if k in moves:
            ax, v = moves[k]
            setattr(t.linear, ax, v)
        elif k == "j":
            t.angular.z = w
        elif k == "l":
            t.angular.z = -w
        elif k == " ":
            pass
        elif k == "t":
            self.call("takeoff"); return
        elif k == "g":
            self.call("land"); return
        elif k == "x":
            self.call("emergency"); return
        else:
            return
        self.cmd, self.until = t, time.time() + self.hold

    def tick(self):
        self.pub.publish(self.cmd if time.time() < self.until else Twist())


ARROWS = {b"\x1b[A": "UP", b"\x1b[B": "DOWN", b"\x1b[C": "RIGHT", b"\x1b[D": "LEFT",
          b"\x1bOA": "UP", b"\x1bOB": "DOWN", b"\x1bOC": "RIGHT", b"\x1bOD": "LEFT"}
_pending = b""


def read_key(timeout):
    """One key, or None. Reads the raw terminal bytes with os.read: sys.stdin is buffered, so after
    sys.stdin.read(1) the rest of an arrow sequence (ESC [ A) sits in Python's buffer, select() sees
    nothing left, and the arrow is lost."""
    global _pending
    fd = sys.stdin.fileno()
    if not _pending:
        if not select.select([fd], [], [], timeout)[0]:
            return None
        _pending = os.read(fd, 64)
    if _pending.startswith(b"\x1b"):
        if len(_pending) < 3 and select.select([fd], [], [], 0.02)[0]:
            _pending += os.read(fd, 64)
        seq, _pending = _pending[:3], _pending[3:]
        return ARROWS.get(seq)
    c, _pending = _pending[:1], _pending[1:]
    return c.decode(errors="ignore").lower()


def main():
    # same local-only ROS 2 settings as real.launch.py, or the teleop cannot see the driver on the Tello Wi-Fi
    os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "LARGE_DATA")
    rclpy.init()
    node = Teleop()
    old = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            k = read_key(0.05)
            if k == "q":
                node.call("land")
                break
            if k:
                node.key(k)
            node.tick()
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        node.call("land")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
