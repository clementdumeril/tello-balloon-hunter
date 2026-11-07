#!/usr/bin/env python3
"""First flight and hardware characterization tool for the DJI Tello (no ROS needed).

Modes
  (default)          Keyboard flight with live video, on-screen telemetry and CSV logging.
  --latency-test     Drone stays on the ground. A window flashes black/white; point the
                     Tello camera at it and the script measures the video latency (task A5).
  --auto-hover N     Takeoff, hover N seconds with zero commands, land. Logs drift/noise (A6, F1).
  --mock             Fake Tello (synthetic video with a known latency) to test the script without a drone.

Keys (identical on AZERTY and QWERTY; the video window must be focused)
  T take off   G land   Arrows move   I / K up / down   J / L yaw   Space hover
  Esc land and quit   Backspace EMERGENCY motor stop (the drone falls!)

Outputs go to logs/<timestamp>_*: telemetry CSV (every state field + commands + frame stats)
and a summary JSON (latency, fps, frame drops, battery, flight time).
"""
import argparse
import csv
import json
import random
import statistics
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

WIN = "Tello"
FLASH_WIN = "Latency test: point the Tello camera here"
RC_HZ = 20
MIN_BATTERY_TAKEOFF = 25
AUTO_LAND_BATTERY = 12


# --------------------------------------------------------------------------- latency test
class FlashScreen:
    """Toggles between black and white at random intervals and remembers when."""

    def __init__(self, headless):
        self.headless = headless
        self.state = False
        self.toggles = deque(maxlen=200)  # (time, new_state)
        self.next_toggle = time.time() + 1.0
        self.toggles.append((time.time(), self.state))

    def update(self):
        now = time.time()
        if now >= self.next_toggle:
            self.state = not self.state
            self.toggles.append((now, self.state))
            self.next_toggle = now + random.uniform(0.8, 1.6)
        if not self.headless:
            img = np.full((500, 500), 255 if self.state else 0, np.uint8)
            cv2.imshow(FLASH_WIN, img)

    def state_at(self, t):
        s = self.toggles[0][1]
        for tt, st in self.toggles:
            if tt <= t:
                s = st
            else:
                break
        return s

    def last_toggle_to(self, state, before):
        for tt, st in reversed(self.toggles):
            if tt <= before and st == state:
                return tt
        return None


class LatencyMeter:
    """Detects brightness transitions in the image center and matches them to screen toggles."""

    def __init__(self, flash):
        self.flash = flash
        self.history = deque(maxlen=300)
        self.bright = None
        self.samples = []

    def update(self, frame_bgr, t_frame):
        h, w = frame_bgr.shape[:2]
        roi = frame_bgr[h // 3: 2 * h // 3, w // 3: 2 * w // 3]
        m = float(roi.mean())
        self.history.append(m)
        lo, hi = min(self.history), max(self.history)
        if hi - lo < 40:  # not enough contrast yet
            return
        th_hi, th_lo = lo + 0.6 * (hi - lo), lo + 0.4 * (hi - lo)
        new = True if m > th_hi else False if m < th_lo else self.bright
        if self.bright is not None and new is not None and new != self.bright:
            t0 = self.flash.last_toggle_to(new, t_frame)
            if t0 is not None and 0.0 < t_frame - t0 < 1.5:
                self.samples.append(t_frame - t0)
        self.bright = new

    def stats(self):
        s = self.samples[2:] if len(self.samples) > 4 else self.samples  # drop warm-up
        if not s:
            return None
        return {
            "n": len(s),
            "mean_ms": round(1000 * statistics.mean(s), 1),
            "median_ms": round(1000 * statistics.median(s), 1),
            "std_ms": round(1000 * statistics.pstdev(s), 1),
            "min_ms": round(1000 * min(s), 1),
            "max_ms": round(1000 * max(s), 1),
            "note": "Includes screen refresh (~10-30 ms) and decoding; subtract the screen part for the camera-to-PC latency.",
        }


# --------------------------------------------------------------------------- mock drone
class MockFrameRead:
    def __init__(self, mock):
        self.mock = mock
        self._idx = -1
        self._frame = None

    @property
    def frame(self):
        idx = int(time.time() * 30)
        if idx != self._idx:
            self._idx = idx
            if random.random() < 0.03:  # simulate a dropped frame
                return self._frame
            t_seen = time.time() - self.mock.latency - random.uniform(0, 0.03)
            on = self.mock.flash.state_at(t_seen) if self.mock.flash else (idx // 30) % 2 == 0
            img = np.full((720, 960, 3), 220 if on else 25, np.uint8)
            img = cv2.add(img, np.random.randint(0, 20, img.shape, dtype=np.uint8))
            cv2.putText(img, "MOCK TELLO", (330, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
            self._frame = img
        return self._frame


class MockTello:
    def __init__(self, latency):
        self.latency = latency
        self.flash = None
        self.is_flying = False
        self.battery = 87
        self.rc = (0, 0, 0, 0)

    def connect(self):
        print("[mock] connected")

    def get_battery(self):
        return self.battery

    def streamon(self):
        pass

    def streamoff(self):
        pass

    def get_frame_read(self):
        return MockFrameRead(self)

    def takeoff(self):
        time.sleep(1.0)
        self.is_flying = True

    def land(self):
        time.sleep(1.0)
        self.is_flying = False

    def emergency(self):
        self.is_flying = False

    def send_rc_control(self, a, b, c, d):
        self.rc = (a, b, c, d)

    def get_current_state(self):
        lr, fb, ud, yw = self.rc
        return {"pitch": 0, "roll": 0, "yaw": 0, "vgx": fb // 10, "vgy": lr // 10, "vgz": -ud // 10,
                "templ": 60, "temph": 62, "tof": 80 if self.is_flying else 10, "h": 80 if self.is_flying else 0,
                "bat": self.battery, "baro": 12.3, "time": 0, "agx": 0.0, "agy": 0.0, "agz": -1000.0}

    def end(self):
        print("[mock] end")


# --------------------------------------------------------------------------- keyboard
class Keyboard:
    """Real key up/down state (so the drone stops the moment a key is released)."""

    def __init__(self):
        from pynput import keyboard
        self.kb = keyboard
        self.down = set()
        self.events = deque()
        self.lock = threading.Lock()
        self.listener = keyboard.Listener(on_press=self._press, on_release=self._release)
        self.listener.start()

    def _name(self, key):
        if hasattr(key, "char") and key.char:
            return key.char.lower()
        return str(key).replace("Key.", "")

    def _press(self, key):
        n = self._name(key)
        with self.lock:
            if n not in self.down:
                self.events.append(n)
            self.down.add(n)

    def _release(self, key):
        with self.lock:
            self.down.discard(self._name(key))

    def snapshot(self):
        with self.lock:
            ev = list(self.events)
            self.events.clear()
            return set(self.down), ev

    def stop(self):
        self.listener.stop()


def check_tello_network():
    """Fail fast if this PC is not on the Tello Wi-Fi (the Tello gives addresses in 192.168.10.x)."""
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("192.168.10.1", 8889))  # no packet is sent: this only asks which local address would be used
        local_ip = s.getsockname()[0]
        s.close()
    except OSError:
        local_ip = "?"
    if not local_ip.startswith("192.168.10."):
        print(f"This PC is not on the Tello Wi-Fi: its address towards the drone is {local_ip}, expected 192.168.10.x.")
        print("Switch the drone on, connect this PC to the TELLO-xxxxxx Wi-Fi network, then run the script again.")
        sys.exit(1)
    print(f"On the Tello Wi-Fi ({local_ip}).")


def window_focused(title):
    """True if our OpenCV window is the foreground window (Windows). Elsewhere, assume yes."""
    if sys.platform != "win32":
        return True
    import ctypes
    hwnd = ctypes.windll.user32.GetForegroundWindow()
    buf = ctypes.create_unicode_buffer(256)
    ctypes.windll.user32.GetWindowTextW(hwnd, buf, 256)
    return buf.value == title


# --------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speed", type=int, default=30, help="rc command magnitude 10-100 (default 30)")
    ap.add_argument("--latency-test", action="store_true", help="measure video latency, no flight")
    ap.add_argument("--auto-hover", type=float, metavar="SECONDS", help="takeoff, hover, land automatically")
    ap.add_argument("--mock", action="store_true", help="fake drone, for testing without hardware")
    ap.add_argument("--mock-latency", type=float, default=0.18, help="latency of the fake video in seconds")
    ap.add_argument("--headless", action="store_true", help="no windows, no keyboard (tests)")
    ap.add_argument("--duration", type=float, help="stop after this many seconds")
    ap.add_argument("--swap-rb", action="store_true", help="swap red/blue if colors look wrong")
    ap.add_argument("--logdir", default=str(Path(__file__).resolve().parent.parent / "logs"))
    args = ap.parse_args()
    speed = max(10, min(100, args.speed))

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    mode = "latency" if args.latency_test else "hover" if args.auto_hover else "flight"
    logdir = Path(args.logdir)
    logdir.mkdir(parents=True, exist_ok=True)
    csv_path = logdir / f"{stamp}_{mode}_telemetry.csv"
    summary_path = logdir / f"{stamp}_{mode}_summary.json"

    # --- drone
    if args.mock:
        tello = MockTello(args.mock_latency)
    else:
        check_tello_network()
        from djitellopy import Tello
        tello = Tello()
    print("Connecting to the Tello...")
    try:
        tello.connect()
    except Exception as e:  # noqa: BLE001
        print(f"\nNo answer from the Tello: {e}")
        print("Check: drone switched on (LED blinking yellow), no phone app connected to it,\n"
              "and Windows Firewall allows inbound UDP 8889/8890/11111 (see scripts/README.md).")
        sys.exit(1)
    battery_start = tello.get_battery()
    print(f"Battery: {battery_start}%")
    can_fly = battery_start >= MIN_BATTERY_TAKEOFF and not args.latency_test
    if battery_start < MIN_BATTERY_TAKEOFF:
        print(f"Battery below {MIN_BATTERY_TAKEOFF}%: takeoff disabled.")

    tello.streamon()
    reader = tello.get_frame_read()

    flash = FlashScreen(args.headless) if args.latency_test else None
    if args.mock:
        tello.flash = flash
    meter = LatencyMeter(flash) if flash else None
    keys = None if args.headless else Keyboard()

    # --- background takeoff/land so the video never freezes
    busy = threading.Event()

    def run_async(fn, label):
        if busy.is_set():
            return
        def worker():
            busy.set()
            try:
                print(f"> {label}")
                fn()
            except Exception as e:  # noqa: BLE001
                print(f"! {label} failed: {e}")
            finally:
                busy.clear()
        threading.Thread(target=worker, daemon=True).start()

    # --- state
    t0 = time.time()
    last_frame, last_sample = None, None
    frame_times = deque(maxlen=2000)
    all_intervals = []
    n_frames = 0
    last_rc_t = 0.0
    rc = (0, 0, 0, 0)
    takeoff_t, flight_time = None, 0.0
    hover_phase = "pending" if args.auto_hover else None
    msg, msg_t = "", 0.0
    state = {}
    writer, csv_file = None, open(csv_path, "w", newline="", encoding="utf-8")

    def say(m):
        nonlocal msg, msg_t
        msg, msg_t = m, time.time()
        print(m)

    if not args.headless:
        cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            now = time.time()
            if args.duration and now - t0 > args.duration:
                break
            if flash:
                flash.update()

            # ---- video
            frame = reader.frame
            if frame is not None and frame is not last_frame:
                sample = frame[::48, ::48].tobytes()
                if sample != last_sample:
                    last_frame, last_sample = frame, sample
                    if frame_times:
                        all_intervals.append(now - frame_times[-1])
                    frame_times.append(now)
                    n_frames += 1
                    img = frame if args.mock else cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    if args.swap_rb:
                        img = img[:, :, ::-1].copy()
                    if meter:
                        meter.update(img, now)
                    if not args.headless:
                        draw_osd(img, state, rc, tello.is_flying, fps(frame_times), meter, now - frame_times[-1],
                                 msg if now - msg_t < 3 else "", mode)
                        cv2.imshow(WIN, img)
            if not args.headless:
                cv2.waitKey(1)

            # ---- keyboard
            down, events = keys.snapshot() if keys else (set(), [])
            focused = args.headless or window_focused(WIN)
            if not focused:
                down = set()
            for e in events:
                if not focused:  # keys typed in other windows are ignored
                    continue
                if e == "backspace":
                    tello.emergency()
                    say("EMERGENCY STOP: motors off")
                elif e == "esc":
                    raise KeyboardInterrupt
                elif e == "t":
                    if not can_fly:
                        say("Takeoff disabled (battery or latency-test mode)")
                    elif not tello.is_flying:
                        run_async(tello.takeoff, "takeoff")
                elif e == "g" and tello.is_flying:
                    run_async(tello.land, "land")
                elif e == "space":
                    say("Hover")

            # ---- auto hover
            if hover_phase == "pending" and not can_fly:
                print("Auto hover cancelled: takeoff not allowed")
                break
            if hover_phase == "pending" and not busy.is_set():
                run_async(tello.takeoff, "auto takeoff")
                hover_phase = "taking_off"
            elif hover_phase == "taking_off" and tello.is_flying and not busy.is_set():
                hover_phase, hover_start = "hovering", now
                say(f"Hovering {args.auto_hover:.0f} s")
            elif hover_phase == "hovering" and now - hover_start > args.auto_hover:
                run_async(tello.land, "auto land")
                hover_phase = "done"
            elif hover_phase == "done" and not tello.is_flying and not busy.is_set():
                break

            # ---- commands + telemetry at RC_HZ
            if now - last_rc_t >= 1.0 / RC_HZ:
                last_rc_t = now
                lr = speed * ((("right" in down) - ("left" in down)))
                fb = speed * ((("up" in down) - ("down" in down)))
                ud = speed * ((("i" in down) - ("k" in down)))
                yw = speed * ((("l" in down) - ("j" in down)))
                rc = (lr, fb, ud, yw) if hover_phase is None else (0, 0, 0, 0)
                if "space" in down:
                    rc = (0, 0, 0, 0)
                try:
                    tello.send_rc_control(*rc)
                except Exception as e:  # noqa: BLE001
                    say(f"rc failed: {e}")
                try:
                    state = dict(tello.get_current_state() or {})
                except Exception:  # noqa: BLE001
                    state = {}

                bat = state.get("bat")
                if tello.is_flying and isinstance(bat, (int, float)) and bat <= AUTO_LAND_BATTERY and not busy.is_set():
                    say(f"Battery {bat}%: auto land")
                    run_async(tello.land, "low-battery land")

                if tello.is_flying and takeoff_t is None:
                    takeoff_t = now
                if not tello.is_flying and takeoff_t is not None:
                    flight_time += now - takeoff_t
                    takeoff_t = None

                row = {"t_wall": f"{now:.4f}", "t": f"{now - t0:.4f}", "flying": int(tello.is_flying),
                       "rc_lr": rc[0], "rc_fb": rc[1], "rc_ud": rc[2], "rc_yaw": rc[3],
                       "frames": n_frames, "since_last_frame_ms": round(1000 * (now - frame_times[-1]), 1) if frame_times else "",
                       "flash_state": int(flash.state) if flash else ""}
                row.update({f"s_{k}": v for k, v in state.items()})
                if writer is None and state:
                    writer = csv.DictWriter(csv_file, fieldnames=list(row.keys()), extrasaction="ignore")
                    writer.writeheader()
                if writer:
                    writer.writerow(row)

            time.sleep(0.002)

    except KeyboardInterrupt:
        print("Quit requested")
    finally:
        if tello.is_flying:
            print("Landing...")
            try:
                tello.send_rc_control(0, 0, 0, 0)
                tello.land()
            except Exception as e:  # noqa: BLE001
                print(f"! land failed: {e}")
        if takeoff_t is not None:
            flight_time += time.time() - takeoff_t
        try:
            battery_end = tello.get_battery()
        except Exception:  # noqa: BLE001
            battery_end = None
        for fn in (tello.streamoff, tello.end):
            try:
                fn()
            except Exception:  # noqa: BLE001
                pass
        if keys:
            keys.stop()
        cv2.destroyAllWindows()
        csv_file.close()

        summary = {
            "mode": mode,
            "mock": args.mock,
            "duration_s": round(time.time() - t0, 1),
            "frames": n_frames,
            "video_fps_mean": round(n_frames / max(1e-6, time.time() - t0), 1),
            "frame_drops": frame_drop_stats(all_intervals),
            "latency": meter.stats() if meter else None,
            "battery_start": battery_start,
            "battery_end": battery_end,
            "flight_time_s": round(flight_time, 1),
            "telemetry_csv": str(csv_path),
        }
        summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(json.dumps(summary, indent=2))
        print(f"Summary saved to {summary_path}")


def fps(times):
    recent = [t for t in times if t > times[-1] - 2.0]
    return (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 2 else 0.0


def frame_drop_stats(intervals):
    if len(intervals) < 10:
        return None
    med = statistics.median(intervals)
    gaps = [i for i in intervals if i > 1.8 * med]
    missing = sum(round(i / med) - 1 for i in gaps)
    return {"median_interval_ms": round(1000 * med, 1), "gaps": len(gaps), "estimated_missing_frames": missing,
            "drop_rate": round(missing / (len(intervals) + missing), 3), "max_gap_ms": round(1000 * max(intervals), 1)}


def draw_osd(img, s, rc, flying, f, meter, since_frame, msg, mode):
    def put(txt, y, color=(255, 255, 255)):
        cv2.putText(img, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(img, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)

    g = lambda k: s.get(k, "-")  # noqa: E731
    put(f"[{mode}] {'FLYING' if flying else 'on ground'}   bat {g('bat')}%   temp {g('temph')}C   video {f:4.1f} fps",
        25, (0, 255, 0) if flying else (255, 255, 255))
    put(f"h {g('h')} cm   tof {g('tof')} cm   baro {g('baro')}", 50)
    put(f"vgx {g('vgx')}  vgy {g('vgy')}  vgz {g('vgz')}   yaw {g('yaw')}  pitch {g('pitch')}  roll {g('roll')}", 75)
    put(f"rc lr {rc[0]} fb {rc[1]} ud {rc[2]} yaw {rc[3]}", 100)
    if meter:
        st = meter.stats()
        put(f"latency: {st['median_ms']} ms median over {st['n']}" if st else "latency: waiting for flashes...", 125, (0, 255, 255))
    if since_frame > 1.0:
        put(f"VIDEO STALLED {since_frame:.1f} s (Wi-Fi?)", 150, (0, 0, 255))
    if msg:
        put(msg, 175, (0, 200, 255))
    h = img.shape[0]
    put("T takeoff  G land  arrows move  I/K up/down  J/L yaw  Space hover  Esc quit  Backspace EMERGENCY", h - 15, (200, 200, 200))


if __name__ == "__main__":
    main()
