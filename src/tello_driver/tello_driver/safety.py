"""What the driver does to every command before it goes to the Tello (no ROS: tested offline as is).

    rc (from the mission / teleop)
      -> position_hold   no horizontal command: hold where the drone stops; no vertical one: hold that height
      -> ceiling         never above max_height (on the height predicted 1 s ahead: ~1 s to stop)
      -> band_guard      not out of the zone's height band (the obstacle grid is only valid there)
      -> zone_guard      slow down / brake / push back before the obstacles of the room scan
      -> wall_guard      a marker too big in the image = wall closer than min_wall: no forward, back off
    -> rc sent

rc = Tello joystick values (left/right, forward/back, up/down, yaw), integers, right/forward/up/clockwise > 0.
The pose comes from the EKF (/ekf/pose, velocity, drift): the driver passes it with on_pose / on_ekf_*.

Position hold (v2, Nov 11): the Tello's own optical-flow hold does not work in this room (vgx = vgy = 0 while it
drifted 0.4 m in 8 s: plain floor). The old PI (kp 0.6) swung 30-50 cm after each key release. Now: position
predicted L_u ahead (the command acts 0.3 s later), desired velocity kp * error, model inversion
u = (v_d - drift + kv (v_d - v)) / K with the EKF velocity and drift, small integral for the model errors. Gains
tuned on the fake drone (drift 8/5 cm/s): kp 0.8 / kv 0.4 / ki 0.15 holds within 2.4 cm (old PI: 13 cm) and
overshoots 7-9 cm after a key release (old: 24 cm). Height: same loop on the EKF height.
"""
import math
import time

from tello_driver.zone import clearance, wrap


class CommandFilter:
    # gains that can be changed in flight (ros2 param set /tello_driver <name> <value>) -> attribute
    LIVE_PARAMS = {"hold_kp": "hold_kp", "hold_kv": "hold_kv", "hold_ki": "hold_ki", "hold_kp_z": "hold_kpz",
                   "hold_vmax": "hold_vmax", "hold_drift_ff": "hold_drift_ff", "position_hold": "hold_enabled",
                   "height_hold": "hold_z"}

    def __init__(self, zone=None, lin_gain=0.38 / 30, yaw_gain=math.radians(18) / 30, max_rc=80, max_height=2.6,
                 model_K=1.15, model_tau=0.55, model_Lu=0.30, model_Kz=0.9, min_wall=0.7, wall_band=0.15,
                 blind_speed=0.15, hold_enabled=True, hold_kp=0.8, hold_kv=0.4, hold_ki=0.15, hold_vmax=0.4,
                 hold_z=True, hold_kpz=1.0, hold_drift_ff=True, warn=print, clock=time.time):
        self.zone = zone
        self.lin_gain, self.yaw_gain, self.max_rc, self.max_height = lin_gain, yaw_gain, int(max_rc), max_height
        self.model_K, self.model_tau, self.model_Lu, self.model_Kz = model_K, model_tau, model_Lu, model_Kz
        self.min_wall, self.wall_band = min_wall, wall_band   # forward speed shrinks over wall_band above min_wall
        self.blind_speed = blind_speed                        # m/s, horizontal speed allowed without a pose
        self.hold_enabled, self.hold_z, self.hold_drift_ff = hold_enabled, hold_z, hold_drift_ff
        self.hold_kp, self.hold_kv, self.hold_ki, self.hold_vmax, self.hold_kpz = hold_kp, hold_kv, hold_ki, hold_vmax, hold_kpz
        self._warn_fn, self.clock = warn, clock
        self._warned = {}
        # pose (EKF), trusted while recent and certain
        self.pose_xy, self.pose_z, self.pose_yaw, self.pose_t = None, 0.0, 0.0, 0.0
        self.pose_sigma, self.pose_sigma_z = 1.0, 1.0
        self.xy_hist, self.h_hist = [], []
        self.ekf_v, self.ekf_v_t, self.ekf_d = (0.0, 0.0, 0.0), 0.0, (0.0, 0.0)
        self.near_d, self.near_t = 9.0, 0.0                   # closest marker in view (its size in the image)
        # hold state
        self.hold_target, self.hold_zt, self.hold_i, self.hold_t = None, None, [0.0, 0.0, 0.0], 0.0
        self.guard_note, self.guard_note_t = "", 0.0

    # ---------------------------------------------------------------- inputs
    def on_pose(self, x, y, z, yaw, sigma_xy, sigma_z):
        # the estimate jumped (markers back after dead reckoning, filter restart) or is uncertain: a hold target
        # taken from the old estimate is meaningless (stress test seed 142: dragged the drone 63 cm to it)
        if self.pose_xy is not None and (math.hypot(x - self.pose_xy[0], y - self.pose_xy[1]) > 0.15 or sigma_xy > 0.06):
            self.hold_target = None
        if abs(z - self.pose_z) > 0.15 or sigma_z > 0.08:
            self.hold_zt = None
        now = self.clock()
        self.pose_xy, self.pose_z, self.pose_yaw, self.pose_t = (x, y), z, yaw, now
        self.pose_sigma, self.pose_sigma_z = sigma_xy, sigma_z
        self.xy_hist = [(t, p) for t, p in self.xy_hist if now - t < 0.6] + [(now, (x, y))]

    def on_ekf_velocity(self, vx, vy, vz):
        self.ekf_v, self.ekf_v_t = (vx, vy, vz), self.clock()

    def on_ekf_drift(self, dx, dy):
        self.ekf_d = (dx, dy)

    def on_nearest_marker(self, d):
        self.near_d, self.near_t = float(d), self.clock()

    def set_param(self, name, value):
        """A live parameter change; False if the name is not one of LIVE_PARAMS."""
        if name not in self.LIVE_PARAMS:
            return False
        setattr(self, self.LIVE_PARAMS[name], value)
        return True

    # ---------------------------------------------------------------- helpers
    def pose_valid(self, max_age, max_sigma):
        """A pose recent enough and certain enough (the EKF keeps publishing without markers, with a growing
        uncertainty: beyond max_sigma it is not trusted)."""
        return self.pose_xy is not None and self.clock() - self.pose_t < max_age and self.pose_sigma < max_sigma

    def velocity_xy(self):
        """Velocity from the last 0.6 s of poses (fallback when the EKF velocity is stale)."""
        if len(self.xy_hist) < 2:
            return 0.0, 0.0
        (t0, (x0, y0)), (t1, (x1, y1)) = self.xy_hist[0], self.xy_hist[-1]
        dt = t1 - t0
        return ((x1 - x0) / dt, (y1 - y0) / dt) if dt > 0.2 else (0.0, 0.0)

    def height_estimate(self, tof_cm):
        """Height above the floor (m): the larger of the distance sensor and the EKF height (conservative for the
        ceiling: over the bed the sensor reads ~0.85 m less). The EKF height is used up to 0.5 m uncertainty: its
        variance grows while the sensor is gated out over furniture (stress test seed 203 reached 2.67 m when it
        was dropped). None if neither is valid."""
        c = []
        if isinstance(tof_cm, (int, float)) and 15 <= tof_cm <= 400:
            c.append(tof_cm / 100.0)
        if self.pose_xy is not None and self.clock() - self.pose_t < 0.5 and self.pose_sigma_z < 0.5:
            c.append(self.pose_z)
        return max(c) if c else None

    def height_and_rate(self, tof_cm, flying):
        """(height, vertical speed over the last 0.6 s), (None, 0) when not flying / no measurement."""
        hgt = self.height_estimate(tof_cm) if flying else None
        if hgt is None:
            return None, 0.0
        now = self.clock()
        self.h_hist = [(t, h) for t, h in self.h_hist if now - t < 0.6] + [(now, hgt)]
        t_old, h_old = self.h_hist[0]
        return hgt, ((hgt - h_old) / (now - t_old) if now - t_old > 0.2 else 0.0)

    def clip(self, v):
        return int(max(-self.max_rc, min(self.max_rc, round(v))))

    def note(self, s):
        """Which guard changed the command (telemetry 'guard', for flight_report / debugging)."""
        self.guard_note, self.guard_note_t = s, self.clock()

    def guard_now(self):
        return self.guard_note if self.clock() - self.guard_note_t < 0.5 else ""

    def warn(self, key, msg, period=1.0):
        """Log at most once per period per kind of message."""
        now = self.clock()
        if now - self._warned.get(key, -1e9) >= period:
            self._warned[key] = now
            self._warn_fn(msg)

    # ---------------------------------------------------------------- the chain
    def filter(self, rc, hgt, vz, flying, busy=False, climbing=False):
        """The command really sent for the requested rc (see the module docstring for the order)."""
        rc = self.position_hold(rc, flying, busy, climbing)
        rc = self.ceiling(rc, hgt, vz)
        rc = self.band_guard(rc, flying, climbing)
        rc = self.zone_guard(rc, flying, climbing)
        return self.wall_guard(rc, flying)

    def hold_axis(self, k, target, pos, vel, drift, K, kp, dt):
        """One axis of the hold: world velocity command (m/s)."""
        e = target - (pos + vel * self.model_Lu)  # where the drone will be when this command acts
        vd = max(-self.hold_vmax, min(self.hold_vmax, kp * e))
        if abs(e) < 0.3:
            self.hold_i[k] = max(-0.1, min(0.1, self.hold_i[k] + self.hold_ki * e * dt))
        u = (vd - drift + self.hold_kv * (vd - vel)) / K + self.hold_i[k]
        return max(-0.6, min(0.6, u))

    def position_hold(self, rc, flying, busy=False, climbing=False):
        """No horizontal/yaw command -> hold where the drone stops; no vertical command -> hold that height.
        A command on an axis releases the hold of that axis; no fresh, certain pose -> no hold (integral kept)."""
        now = self.clock()
        dt = min(0.1, now - self.hold_t) if self.hold_t else 0.05
        self.hold_t = now
        if not (self.hold_enabled and flying) or busy:
            self.hold_target = self.hold_zt = None
            return rc
        lr, fb, ud, yw = rc
        moving_xy = bool(lr or fb or yw)
        if moving_xy:
            self.hold_target = None  # the pilot / the mission is moving the drone
            self.hold_i[0] = self.hold_i[1] = 0.0
        if ud or climbing:
            self.hold_zt = None
            self.hold_i[2] = 0.0
        if not self.pose_valid(0.4, 0.12):
            return rc
        x, y = self.pose_xy
        z = self.pose_z
        if now - self.ekf_v_t < 0.3:
            vx, vy, vz = self.ekf_v
        else:
            (vx, vy), vz = self.velocity_xy(), 0.0
        dx, dy = self.ekf_d if (self.hold_drift_ff and now - self.ekf_v_t < 0.3) else (0.0, 0.0)
        if not moving_xy and (self.hold_target is not None or self.pose_sigma < 0.06):
            if self.hold_target is None:  # a new target only on a certain pose (markers in view)
                # hold where it stops on its own (it keeps going ~ (L_u + tau) * v), not where the keys were released
                s = self.model_Lu + self.model_tau
                self.hold_target = (x + s * vx, y + s * vy, self.pose_yaw)
                self.hold_i[0] = self.hold_i[1] = 0.0
            tx, ty, tyaw = self.hold_target
            wx = self.hold_axis(0, tx, x, vx, dx, self.model_K, self.hold_kp, dt)
            wy = self.hold_axis(1, ty, y, vy, dy, self.model_K, self.hold_kp, dt)
            c, s = math.cos(self.pose_yaw), math.sin(self.pose_yaw)
            bx, by = c * wx + s * wy, -s * wx + c * wy
            wz = max(-0.3, min(0.3, 0.8 * wrap(tyaw - self.pose_yaw)))
            lr, fb, yw = self.clip(-by / self.lin_gain), self.clip(bx / self.lin_gain), self.clip(-wz / self.yaw_gain)
        if self.hold_z and not ud and not climbing:
            if self.hold_zt is None:
                self.hold_zt = z + 0.5 * vz
                self.hold_i[2] = 0.0
            uz = self.hold_axis(2, self.hold_zt, z, vz, 0.0, self.model_Kz, self.hold_kpz, dt)
            ud = self.clip(max(-0.3, min(0.3, uz)) / self.lin_gain)
        return (lr, fb, ud, yw)

    def ceiling(self, rc, hgt, vz):
        """Never above max_height, whatever program sends the commands. The drone needs ~1 s to stop (tau ~0.8 s
        + delay): the guard works on the height predicted 1 s ahead, not on the current one."""
        if hgt is None:
            return rc
        h_pred = hgt + max(vz, 0.0) * 1.0
        limit = self.max_height - 0.15
        lr, fb, ud, yw = rc
        if hgt > self.max_height:
            ud = -30
            self.note("ceiling")
            self.warn("ceiling", f"height {hgt:.2f} m > {self.max_height:.2f} m: descending")
        elif h_pred > limit:
            ud = min(ud, -20)  # brake now, or it overshoots
            self.note("ceiling: brake")
            self.warn("ceiling", f"height {hgt:.2f} m rising {vz:+.2f} m/s: braking before the {self.max_height:.2f} m limit")
        else:
            ud = min(ud, int((limit - h_pred) / self.lin_gain))  # slower and slower towards the limit
        return (lr, fb, ud, yw)

    def band_guard(self, rc, flying, climbing=False):
        """The obstacle grid is only valid in the zone's height band (band mode 1.70-2.00 m): the pilot cannot
        take the drone more than 10 cm out of it, and 25 cm out it is pushed back (stress test seed 27 went down
        to 0.17 m, among the furniture). Only on a certain EKF height (uncertain again after the takeoff jump: a
        wrong one pushed the drone to 2.6 m in the sim); markers not required (blind descents went below 1.2 m)."""
        z = self.zone
        if (not z or not flying or climbing or self.pose_xy is None
                or self.clock() - self.pose_t > 0.5 or self.pose_sigma_z > 0.10):
            return rc
        lr, fb, ud, yw = rc
        h = self.pose_z
        if h < z["z_min"] - 0.25:
            ud = max(ud, 12)
            self.note("band: up")
        elif h < z["z_min"] - 0.10 and ud < 0:
            ud = 0
            self.note("band: low")
        if h > z["z_max"] + 0.25:
            ud = min(ud, -12)
            self.note("band: down")
        elif h > z["z_max"] + 0.10 and ud > 0:
            ud = 0
            self.note("band: high")
        return (lr, fb, ud, yw)

    def zone_guard(self, rc, flying, climbing=False):
        """Keep the drone inside the flight zone, on the position predicted 1 s ahead (the drone needs ~1 s to
        stop): outward commands shrink near the edge, brake before it, push back in beyond it."""
        z = self.zone
        if not z or not flying or climbing:
            return rc
        if not self.pose_valid(0.5, 0.25):
            # position unknown: the zone cannot be enforced. Nov 10 (sim) the drone flew blind through the wall, so
            # moves were blocked; but then the pilot could not get out (flight 225048: 'it will not go back, I
            # have to turn'). Now: slow moves only (0.15 m/s, ~1 s to cross the 15 cm the pilot can react in).
            lim = int(round(self.blind_speed / self.lin_gain))
            if abs(rc[0]) > lim or abs(rc[1]) > lim:
                self.note("blind: slow")
                self.warn("blind", f"no marker pose: horizontal moves slowed to {self.blind_speed:.2f} m/s (turn to "
                          "see the markers)", 2.0)
            return (max(-lim, min(lim, rc[0])), max(-lim, min(lim, rc[1])), rc[2], rc[3])
        x, y = self.pose_xy
        vx, vy = self.velocity_xy()
        lr, fb, ud, yw = rc
        c, s = math.cos(self.pose_yaw), math.sin(self.pose_yaw)
        bx, by = fb * self.lin_gain, -lr * self.lin_gain          # body velocity command
        wx, wy = c * bx - s * by, s * bx + c * by                 # world
        if "grid" in z:
            wx, wy = self.grid_guard(x, y, vx, vy, wx, wy)
        else:
            wx, wy = self.box_guard(x, y, vx, vy, wx, wy)
        bx, by = c * wx + s * wy, -s * wx + c * wy
        return (self.clip(-by / self.lin_gain), self.clip(bx / self.lin_gain), ud, yw)

    def grid_guard(self, x, y, vx, vy, wx, wy):
        """Free-space guard: distance to the nearest scanned obstacle at the current and the predicted (1 s)
        position. Closer than the margin: the velocity towards the obstacle is removed and the drone is pushed
        away; approaching it: that velocity shrinks to zero at the margin."""
        mg = self.zone["margin"]
        d, (gx, gy) = clearance(self.zone, x, y)
        # where the drone will be in 1 s if this command is kept (identified model: delay L_u, then first-order
        # response): a planned deceleration is not taken for a run at the wall
        K, tau, Lu, H = self.model_K, self.model_tau, self.model_Lu, 1.0
        f = tau * (1 - math.exp(-(H - Lu) / tau))
        px = x + vx * Lu + K * wx * (H - Lu) + (vx - K * wx) * f
        py = y + vy * Lu + K * wy * (H - Lu) + (vy - K * wy) * f
        dp, (gpx, gpy) = clearance(self.zone, px, py)
        if dp < d:
            d_use, (ux, uy) = dp, (gpx, gpy)
        else:
            d_use, (ux, uy) = d, (gx, gy)
        if ux == 0.0 and uy == 0.0:
            return wx, wy
        toward = -(wx * ux + wy * uy)  # > 0: moving towards the obstacle
        if d < mg:
            if toward > -0.25:
                wx, wy = wx + (toward + 0.25) * ux, wy + (toward + 0.25) * uy
            self.note("zone: push away")
            self.warn("zone", f"flight zone: {d:.2f} m from an obstacle at ({x:+.2f}, {y:+.2f}): pushing away")
        elif d_use < mg + 0.1:
            if toward > -0.15:
                wx, wy = wx + (toward + 0.15) * ux, wy + (toward + 0.15) * uy
            self.note("zone: brake")
            self.warn("zone", f"flight zone: braking ({d_use:.2f} m from an obstacle)")
        else:
            allowed = (d_use - mg) * 1.0
            if toward > allowed:
                wx, wy = wx + (toward - allowed) * ux, wy + (toward - allowed) * uy
                self.note("zone: slow")
        return wx, wy

    def box_guard(self, x, y, vx, vy, wx, wy):
        """Without the scan grid: the mission rectangle only (same braking on the position predicted 1 s ahead)."""
        z, m, hit = self.zone, 0.15, []
        for name, pos, vel, lo, hi in (("x", x, vx, z["x_min"], z["x_max"]), ("y", y, vy, z["y_min"], z["y_max"])):
            pred = pos + vel * 1.0
            w = wx if name == "x" else wy
            if pos > hi:
                w = min(w, -0.25); hit.append(f"{name}>{hi:.2f}")
            elif pred > hi - m:
                w = min(w, -0.15); hit.append(f"{name}->{hi:.2f}")
            else:
                w = min(w, (hi - m - pred) * 1.0)
            if pos < lo:
                w = max(w, 0.25); hit.append(f"{name}<{lo:.2f}")
            elif pred < lo + m:
                w = max(w, 0.15); hit.append(f"{name}->{lo:.2f}")
            else:
                w = max(w, (lo + m - pred) * 1.0)
            if name == "x":
                wx = w
            else:
                wy = w
        if hit:
            self.warn("zone", f"flight zone: braking/pushing back ({', '.join(hit)}) at ({x:+.2f}, {y:+.2f})")
        return wx, wy

    def wall_guard(self, rc, flying):
        """A marker in view closer than min_wall: no forward motion, back off. Between that and +wall_band the
        forward speed shrinks. Independent of the map and of the pose (camera = forward)."""
        if not flying or self.clock() - self.near_t > 0.4:
            return rc
        lr, fb, ud, yw = rc
        d = self.near_d
        if d < self.min_wall:
            fb = min(fb, -12)
            self.note("wall: back off")
            self.warn("wall", f"wall {d:.2f} m ahead (marker size): backing off")
        elif d < self.min_wall + self.wall_band and fb > 0:
            fb = int(fb * (d - self.min_wall) / self.wall_band)
            self.note("wall: slow")
        return (lr, fb, ud, yw)
