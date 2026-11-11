"""Minimum-jerk trajectories (quintic polynomials) and their safety check against the room's free space.

A rest-to-rest quintic from p0 to p1 in T seconds:
    p(t) = p0 + (p1 - p0) s(t/T),   s(r) = 10 r^3 - 15 r^4 + 6 r^5
peak speed 15/8 d/T, peak acceleration 10/sqrt(3) d/T^2, smooth jerk: what a quick, clean move looks like.
T is the shortest duration that respects v_max and a_max. A trajectory is accepted only if every point along it
stays in the obstacle-free space of the room scan (clearance >= margin) and inside the height limits.
"""
import math

import numpy as np


def s_profile(r):
    r = min(max(r, 0.0), 1.0)
    return (10 * r ** 3 - 15 * r ** 4 + 6 * r ** 5,
            30 * r ** 2 - 60 * r ** 3 + 30 * r ** 4,
            60 * r - 180 * r ** 2 + 120 * r ** 3)


class Quintic:
    def __init__(self, p0, p1, v_max=0.8, a_max=1.0, t_min=0.6):
        self.p0, self.p1 = np.asarray(p0, float), np.asarray(p1, float)
        d = float(np.linalg.norm(self.p1 - self.p0))
        self.T = max(t_min, 15.0 / 8.0 * d / v_max, math.sqrt(10.0 / math.sqrt(3.0) * d / a_max)) if d > 1e-3 else t_min
        self.d = d

    def at(self, t):
        """(position, velocity, acceleration) at time t (clamped to [0, T])."""
        s, ds, dds = s_profile(t / self.T)
        dp = self.p1 - self.p0
        return self.p0 + dp * s, dp * ds / self.T, dp * dds / self.T ** 2

    def samples(self, n=40):
        return [self.at(self.T * k / n)[0] for k in range(n + 1)]


def check(traj, zone, clearance_fn, z_lim=(1.2, 2.1)):
    """None if the trajectory is safe, else the reason."""
    for p in traj.samples():
        if not z_lim[0] - 1e-3 <= p[2] <= z_lim[1] + 1e-3:
            return f"height {p[2]:.2f} m outside [{z_lim[0]}, {z_lim[1]}]"
        if zone and "grid" in zone:
            d, _ = clearance_fn(zone, p[0], p[1])
            if d < zone["margin"]:
                return f"passes {d:.2f} m from an obstacle at ({p[0]:+.2f}, {p[1]:+.2f})"
    return None
