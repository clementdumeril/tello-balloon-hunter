"""Fake Tello for tests without hardware (driver mock:=true): same interface as djitellopy's Tello.

With a marker map, FakeWorld (fake_world.py) simulates the identified dynamics and renders the camera view of the
markers, delivered VIDEO_DELAY late like the real video. Without one: a plain moving test pattern.
"""
import threading
import time

import numpy as np


class FakeFrameRead:
    """Without a marker map: a plain moving test pattern. With one: the virtual camera view of the markers,
    delivered VIDEO_DELAY late like the real video (the EKF assumes that latency)."""
    def __init__(self, world=None):
        self.world = world
        self._t = -1
        self._frame = None
        self._queue = []

    @property
    def frame(self):
        k = int(time.time() * 30)
        if k != self._t:
            self._t = k
            if self.world is not None:
                from tello_driver.fake_world import VIDEO_DELAY
                now = time.time()
                self._queue.append((now, self.world.render()))
                while len(self._queue) > 1 and self._queue[1][0] <= now - VIDEO_DELAY:
                    self._queue.pop(0)
                self._frame = self._queue[0][1]
            else:
                img = np.full((720, 960, 3), 90, np.uint8)
                img[:, :, 0] = (np.arange(960) * 255 // 960).astype(np.uint8)
                img[(k * 7) % 720, :, :] = 255
                self._frame = img
        return self._frame


class FakeTello:
    def __init__(self, markers_file=""):
        self.world = None
        if markers_file:
            from tello_driver.fake_world import FakeWorld
            self.world = FakeWorld(markers_file)
        self.is_flying = False

    def connect(self): pass
    def streamon(self): pass
    def streamoff(self): pass
    def end(self): pass
    def get_battery(self): return 90
    def get_frame_read(self): return FakeFrameRead(self.world)

    def takeoff(self):
        if self.world is not None:
            self.world.flying = True
            self.world.z_target = max(self.world.p[2], 0.85)  # the real Tello stops around 0.8-0.9 m
            self.wait_z()
        else:
            time.sleep(1.0)
        self.is_flying = True

    def move_up(self, cm):
        if self.world is not None:
            self.world.z_target = self.world.p[2] + cm / 100.0
            self.wait_z()

    def wait_z(self, timeout=8.0):
        t0 = time.time()
        while self.world.z_target is not None and time.time() - t0 < timeout:
            time.sleep(0.05)

    def get_height(self):
        return int(self.world.p[2] * 100) if self.world is not None else 80

    def land(self):
        if self.world is not None:
            self.world.z_target = 0.05
        time.sleep(1.0)
        self.is_flying = False
        if self.world is not None:
            self.world.flying = False

    def emergency(self):
        self.is_flying = False
        if self.world is not None:
            self.world.flying, self.world.z_target = False, 0.05

    def send_command_without_return(self, cmd):
        if cmd == "land":
            threading.Thread(target=self.land, daemon=True).start()

    def send_rc_control(self, a, b, c, d):
        if self.world is not None:
            self.world.rc = (a, b, c, d)

    def get_current_state(self):
        if self.world is not None:
            return self.world.state()
        return {"pitch": 0, "roll": 0, "yaw": 0, "vgx": 0, "vgy": 0, "vgz": 0, "templ": 50, "temph": 52,
                "tof": 80 if self.is_flying else 10, "h": 80 if self.is_flying else 0, "bat": 90,
                "baro": 80.0, "time": 0, "agx": 0.0, "agy": 0.0, "agz": -1000.0}
