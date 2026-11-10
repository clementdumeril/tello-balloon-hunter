"""Keyboard teleop for the Tello in a terminal (same keys as scripts/first_flight.py).

    ros2 run tello_driver tello_teleop

Each key press moves for `hold` seconds (a terminal cannot see key releases): keep the key
pressed to keep moving; the driver's watchdog stops the drone if nothing arrives.
A status line shows whether the driver is there, whether the drone is flying, and the last key.
"""
import json
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger

HELP = """
  t  take off          g  land               x  EMERGENCY (motors off)
  arrows  move (forward / back / left / right)
  i / k   up / down    j / l   turn left / right
  space   stop         q  land and quit      (Ctrl+C also lands)
Keep a key pressed to keep moving: each press moves for 0.35 s.
"""
CLEAR_LINE = "\r\x1b[2K"
KEY_NAMES = {"UP": "forward", "DOWN": "back", "LEFT": "left", "RIGHT": "right", "i": "up", "k": "down",
             "j": "turn left", "l": "turn right", " ": "stop"}


class Teleop(Node):
    def __init__(self):
        super().__init__("tello_teleop")
        self.speed = self.declare_parameter("speed", 0.35).value      # m/s
        self.yaw_rate = self.declare_parameter("yaw_rate", 0.6).value  # rad/s
        self.hold = self.declare_parameter("hold", 0.35).value         # s
        self.pub = self.create_publisher(Twist, "/tello/cmd_vel_safe", 10)
        self.cli = {n: self.create_client(Trigger, f"/tello_driver/{n}") for n in ("takeoff", "land", "emergency")}
        self.create_subscription(String, "/tello/telemetry", self.on_tel, 10)
        self.cmd = Twist()
        self.until = 0.0
        self.tel = {}
        self.tel_t = 0.0
        self.last_key = "-"
        self.last_status = ""

    def on_tel(self, m):
        try:
            self.tel = json.loads(m.data)
            self.tel_t = time.time()
        except ValueError:
            pass

    def say(self, text):
        """Print a message on its own line (the status line is redrawn below it)."""
        print(f"{CLEAR_LINE}{text}\r", flush=True)
        self.last_status = ""

    def status(self):
        """One line, rewritten in place: is the driver there, is the drone flying, last command."""
        if self.count_subscribers("/tello/cmd_vel_safe") == 0:
            line = "DRIVER NOT FOUND: start  ros2 launch tello_driver real.launch.py  first"
        elif time.time() - self.tel_t > 2.0:
            line = "driver found, waiting for telemetry..."
        else:
            fly = "FLYING" if self.tel.get("flying") else "on the ground (t = take off)"
            line = (f"{fly} | battery {self.tel.get('bat', '?')} % | height {self.tel.get('h', '?')} cm"
                    f" | last key: {self.last_key}")
        if line != self.last_status:
            print(CLEAR_LINE + line, end="", flush=True)
            self.last_status = line

    def call(self, name):
        c = self.cli[name]
        if not c.wait_for_service(timeout_sec=1.0):
            self.say(f"/tello_driver/{name} not available: is the driver running?")
            return
        fut = c.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if fut.result() is not None:
            self.say(f"{name}: {fut.result().message}")

    def key(self, k):
        if k in KEY_NAMES:
            self.last_key = KEY_NAMES[k]
            if k != " " and not self.tel.get("flying", False):
                self.last_key += " (ignored: not flying, press t)"
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
    # No ROS signal handler: by default Ctrl+C shuts ROS down at once, and the 'land' call that follows fails.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Teleop()
    old = termios.tcgetattr(sys.stdin)
    print(HELP)
    t_status = 0.0
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            k = read_key(0.05)
            if k == "q":
                node.call("land")
                break
            if k:
                node.key(k)
            node.tick()
            rclpy.spin_once(node, timeout_sec=0)
            if time.time() - t_status > 0.3:
                node.status()
                t_status = time.time()
    except KeyboardInterrupt:
        node.say("Ctrl+C: landing")
        node.pub.publish(Twist())
        node.call("land")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        try:
            node.pub.publish(Twist())
            node.destroy_node()
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
        print()


if __name__ == "__main__":
    main()
