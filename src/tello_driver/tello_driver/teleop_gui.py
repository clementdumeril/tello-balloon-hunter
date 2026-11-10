"""Teleop window: the drone moves WHILE a key is held and stops as soon as it is released.

    ros2 run tello_driver tello_teleop_gui

A terminal cannot see key releases (each press moved for a fixed time); a window can. Several keys can be
held together (e.g. forward + left + turn). Click the window first: when it loses focus, the drone stops.

  t take off   g land   x EMERGENCY   space stop   q / closing the window: land and quit
  arrows: forward/back/left/right   i/k (or z/s): up/down   j/l (or a/e): turn left/right
  1 / 2 / 3: speed 0.25 / 0.5 / 0.75 m/s (the driver caps rc at 60, about 0.76 m/s)
"""
import json
import math
import os
import signal
import time
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger

SPEEDS = {"1": 0.25, "2": 0.5, "3": 0.75}   # m/s
YAW_RATES = {"1": 0.5, "2": 0.9, "3": 1.3}  # rad/s
# key -> (axis, sign); axis: x forward, y left, z up, w yaw (counter-clockwise)
MOTION = {"Up": ("x", 1), "Down": ("x", -1), "Left": ("y", 1), "Right": ("y", -1),
          "i": ("z", 1), "k": ("z", -1), "z": ("z", 1), "s": ("z", -1),
          "j": ("w", 1), "l": ("w", -1), "a": ("w", 1), "e": ("w", -1)}
RELEASE_DEBOUNCE = 0.06  # s: X11 auto-repeat sends release+press pairs while a key is held
STOP_REQUESTED = False   # set by Ctrl+C in the terminal (Tk would swallow a KeyboardInterrupt in a callback)


def on_sigint(signum, frame):
    global STOP_REQUESTED
    STOP_REQUESTED = True


class TeleopGui(Node):
    def __init__(self, root):
        super().__init__("tello_teleop_gui")
        self.root = root
        self.pub = self.create_publisher(Twist, "/tello/cmd_vel_safe", 10)
        self.cli = {n: self.create_client(Trigger, f"/tello_driver/{n}") for n in ("takeoff", "land", "emergency")}
        self.create_subscription(String, "/tello/telemetry", self.on_tel, 10)
        self.tel, self.tel_t = {}, 0.0
        self.level = "1"
        self.held = {}        # key -> time of the last press
        self.released = {}    # key -> time of the last release
        self.pending = []     # (name, future)
        self.msg, self.msg_t = "", 0.0

        root.title("Tello teleop")
        root.configure(bg="#1e1e1e")
        font = ("DejaVu Sans", 13)
        self.status = tk.Label(root, font=("DejaVu Sans", 15, "bold"), fg="white", bg="#1e1e1e", anchor="w", width=60)
        self.status.pack(fill="x", padx=12, pady=(12, 4))
        self.cmd_lbl = tk.Label(root, font=font, fg="#7fd7ff", bg="#1e1e1e", anchor="w")
        self.cmd_lbl.pack(fill="x", padx=12)
        self.msg_lbl = tk.Label(root, font=font, fg="#ffd24d", bg="#1e1e1e", anchor="w")
        self.msg_lbl.pack(fill="x", padx=12, pady=4)
        help_txt = ("t take off    g land    x EMERGENCY    space stop    q land + quit\n"
                    "arrows: move    i/k: up/down    j/l: turn    1/2/3: speed\n"
                    "Hold the keys: the drone stops as soon as you release them.")
        tk.Label(root, text=help_txt, font=("DejaVu Sans Mono", 11), fg="#bbbbbb", bg="#1e1e1e",
                 justify="left").pack(fill="x", padx=12, pady=(4, 12))
        root.bind("<KeyPress>", self.on_press)
        root.bind("<KeyRelease>", self.on_release)
        root.bind("<FocusOut>", lambda e: self.stop_all("window lost focus: stopped (click it again)"))
        root.protocol("WM_DELETE_WINDOW", self.quit)
        root.after(50, self.tick)

    # ---------------------------------------------------------------- ROS side
    def on_tel(self, m):
        try:
            self.tel, self.tel_t = json.loads(m.data), time.time()
        except ValueError:
            pass

    def in_air(self):
        h = self.tel.get("h", 0)
        return bool(self.tel.get("flying")) or (isinstance(h, (int, float)) and h >= 20)

    def call(self, name):
        c = self.cli[name]
        if not c.service_is_ready():
            self.say(f"/tello_driver/{name} not available: is the driver running?")
            return None
        fut = c.call_async(Trigger.Request())
        self.pending.append((name, fut))
        return fut

    def say(self, text):
        self.msg, self.msg_t = text, time.time()

    # ---------------------------------------------------------------- keyboard
    def key_name(self, e):
        return e.keysym if e.keysym in ("Up", "Down", "Left", "Right", "space") else e.keysym.lower()

    def on_press(self, e):
        k = self.key_name(e)
        now = time.time()
        if k in MOTION:
            self.held[k] = now
            if not self.in_air():
                self.say("on the ground: press t to take off")
        elif k == "space":
            self.stop_all("stop")
        elif k in SPEEDS:
            self.level = k
            self.say(f"speed {SPEEDS[k]} m/s")
        elif k == "t":
            self.call("takeoff")
        elif k == "g":
            self.stop_all()
            self.call("land")
        elif k == "x":
            self.stop_all()
            self.call("emergency")
        elif k == "q":
            self.quit()

    def on_release(self, e):
        k = self.key_name(e)
        if k in self.held:
            self.released[k] = time.time()

    def stop_all(self, text=None):
        self.held.clear()
        self.released.clear()
        self.pub.publish(Twist())
        if text:
            self.say(text)

    def command(self):
        now = time.time()
        for k, t_rel in list(self.released.items()):
            # a real release: no new press since, and the debounce delay has passed
            if k in self.held and t_rel >= self.held[k] and now - t_rel > RELEASE_DEBOUNCE:
                del self.held[k]
                del self.released[k]
        v, w = SPEEDS[self.level], YAW_RATES[self.level]
        t = Twist()
        for k in self.held:
            ax, sgn = MOTION[k]
            if ax == "x":
                t.linear.x += sgn * v
            elif ax == "y":
                t.linear.y += sgn * v
            elif ax == "z":
                t.linear.z += sgn * v
            else:
                t.angular.z += sgn * w
        return t

    # ---------------------------------------------------------------- loop
    def tick(self):
        if STOP_REQUESTED:
            self.say("Ctrl+C: landing")
            self.quit()
            return
        rclpy.spin_once(self, timeout_sec=0)
        t = self.command()
        self.pub.publish(t)
        for name, fut in list(self.pending):
            if fut.done():
                self.pending.remove((name, fut))
                r = fut.result()
                self.say(f"{name}: {r.message if r else 'no answer'}")
        if self.count_subscribers("/tello/cmd_vel_safe") == 0:
            self.status.config(text="DRIVER NOT FOUND: start  ros2 launch tello_driver real.launch.py", fg="#ff6060")
        elif time.time() - self.tel_t > 2.0:
            self.status.config(text="driver found, waiting for telemetry...", fg="#ffd24d")
        else:
            state = ("TAKING OFF / CLIMBING (wait)" if self.tel.get("busy") else
                     "FLYING" if self.in_air() else "ON THE GROUND (t = take off)")
            self.status.config(text=f"{state}   battery {self.tel.get('bat', '?')} %   height {self.tel.get('h', '?')} cm",
                               fg="#60ff90" if self.in_air() else "white")
        self.cmd_lbl.config(text=f"speed {SPEEDS[self.level]} m/s   command: fwd {t.linear.x:+.2f}  left {t.linear.y:+.2f}  "
                                 f"up {t.linear.z:+.2f} m/s   turn {math.degrees(t.angular.z):+.0f} deg/s")
        self.msg_lbl.config(text=self.msg if time.time() - self.msg_t < 4.0 else "")
        self.root.after(50, self.tick)

    def quit(self):
        self.stop_all()
        if self.in_air():
            fut = self.call("land")
            if fut is not None:
                rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        self.root.destroy()


def main():
    # same local-only ROS 2 settings as real.launch.py, or the teleop cannot see the driver on the Tello Wi-Fi
    os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "LARGE_DATA")
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl+C must still be able to land
    signal.signal(signal.SIGINT, on_sigint)
    root = tk.Tk()
    node = TeleopGui(root)
    try:
        root.mainloop()
    finally:
        try:
            node.pub.publish(Twist())
            node.destroy_node()
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
