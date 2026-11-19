"""Figures + numbers of the 'PnP step by step' section (chapter 2). Pinhole camera of config/tello_camera.yaml,
without distortion for clarity. Camera frame: x right, y down, z forward (OpenCV optical frame)."""
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
fx, fy, cx, cy = 910.47, 913.55, 466.65, 363.89
L = 0.1889; h = L / 2
sq = np.array([[-h, -h, 0], [h, -h, 0], [h, h, 0], [-h, h, 0]])  # TL, TR, BR, BL (y down)

def proj(P):
    return np.c_[fx * P[:, 0] / P[:, 2] + cx, fy * P[:, 1] / P[:, 2] + cy]

def roty(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def corners(d, yaw_deg, off=(0, 0)):
    return proj((roty(np.radians(yaw_deg)) @ sq.T).T + np.array([off[0], off[1], d]))

print("facing, 1 m:", np.round(corners(1.0, 0), 1).tolist(), " width", round(corners(1.0, 0)[1, 0] - corners(1.0, 0)[0, 0], 1))
print("facing, 2 m:", np.round(corners(2.0, 0), 1).tolist(), " width", round(corners(2.0, 0)[1, 0] - corners(2.0, 0)[0, 0], 1))
for a in (20, -20):
    c = corners(2.0, a)
    print(f"tilt {a:+d}, 2 m:", np.round(c, 1).tolist(), " left edge", round(c[3, 1] - c[0, 1], 2), " right edge", round(c[2, 1] - c[1, 1], 2), " width", round(c[1, 0] - c[0, 0], 1))
cp, cm = corners(2.0, 20), corners(2.0, -20)
# the mirror ambiguity, as OpenCV sees it: an off-axis marker (0.5 m to the right): pose +a vs the IPPE twin
import cv2
K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])  # IPPE_SQUARE order, marker frame (y up)
img = corners(2.0, 20, off=(0.5, 0.0)).astype(np.float64)
rng = np.random.default_rng(3)
n, rvecs, tvecs, errs = cv2.solvePnPGeneric(obj.astype(np.float64), img + rng.normal(0, 0.3, img.shape), K, np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE)
sols = []
for k in range(n):
    R, _ = cv2.Rodrigues(rvecs[k]); t = tvecs[k].ravel()
    cam_in_marker = -R.T @ t
    yaw = np.degrees(np.arctan2(R[0, 2], R[2, 2]))
    sols.append((float(np.ravel(errs)[k]), cam_in_marker, yaw))
    print(f"IPPE sol {k}: reproj {float(np.ravel(errs)[k]):.2f} px, marker yaw {yaw:+.1f} deg, camera in marker frame", np.round(cam_in_marker, 3))
print("distance between the two camera positions: %.2f m" % np.linalg.norm(sols[0][1] - sols[1][1]))
# precision of the angle vs spread of the points
for spread_px in (86, 455):
    print(f"spread {spread_px} px: angle error ~ {np.degrees(0.5 / spread_px):.2f} deg for 0.5 px")

# ---- figure 1: perspective (facing at 1 m / 2 m, tilted +-20 at 2 m)
fig, ax = plt.subplots(1, 2, figsize=(10, 3.8))
for c, lab, st in [(corners(1.0, 0), "facing, 1 m", "-"), (corners(2.0, 0), "facing, 2 m", "--")]:
    q = np.vstack([c, c[:1]]); ax[0].plot(q[:, 0], q[:, 1], st, lw=2, label=lab)
ax[0].plot(cx, cy, "k+"); ax[0].set_title("Distance = size in pixels"); ax[0].legend(fontsize=8)
for c, lab, col in [(cp, r"turned $+20^\circ$", "C3"), (cm, r"turned $-20^\circ$", "C2")]:
    q = np.vstack([c, c[:1]]); ax[1].plot(q[:, 0], q[:, 1], color=col, lw=2, label=lab)
q = np.vstack([corners(2.0, 0), corners(2.0, 0)[:1]]); ax[1].plot(q[:, 0], q[:, 1], "k:", lw=1, label="facing")
ax[1].set_title("Angle = shape (2 m):\nthe two trapezoids are mirror images"); ax[1].legend(fontsize=8)
for a in ax:
    a.set_aspect("equal"); a.invert_yaxis(); a.set_xlabel("u (px)"); a.set_ylabel("v (px)"); a.grid(alpha=0.3)
plt.tight_layout(); plt.savefig("pnp_perspective.pdf")

# ---- figure 2: top view of the two IPPE solutions (marker frame: x right, z out of the marker towards the camera)
fig, ax = plt.subplots(figsize=(6.5, 4.6))
ax.plot([-h, h], [0, 0], "k-", lw=5, label="marker (19 cm)")
for (e, c, yaw), col in zip(sols, ("C0", "C1")):
    ax.plot(c[0], c[2], "o", color=col, ms=9, label=f"camera, solution reproj {e:.2f} px")
    ax.plot([c[0], 0], [c[2], 0], ":", color=col)
ax.set_xlabel("x along the wall (m)"); ax.set_ylabel("distance from the wall (m)")
ax.set_title("One marker: two camera positions explain\nthe same 4 corners (top view)"); ax.legend(fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.15), ncol=1)
ax.set_xlim(-1.0, 1.0); ax.set_ylim(-0.1, 2.3)
ax.set_aspect("equal"); ax.grid(alpha=0.3); plt.tight_layout(); plt.savefig("pnp_mirror.pdf")
