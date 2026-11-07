"""Marker map: load config/markers.yaml and compute marker poses and corner positions in `map`.

Shared by the Gazebo world generator and the real/sim localizer, so both use exactly the same geometry.

Marker frame (OpenCV convention): origin at the marker center, x right, y up, z out of the marker.
A marker with yaw_deg = psi faces the direction (cos psi, sin psi, 0) and stands upright.
"""
import math
from dataclasses import dataclass

import numpy as np
import yaml


@dataclass
class Marker:
    id: int
    size: float
    t: np.ndarray  # (3,) center in map
    R: np.ndarray  # (3,3) columns = marker x, y, z axes expressed in map

    def corners_map(self):
        """4x3 corners in map, in OpenCV detection order: top-left, top-right, bottom-right, bottom-left."""
        h = self.size / 2.0
        local = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])
        return (self.R @ local.T).T + self.t


def rotation_from_yaw(yaw_rad):
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    z = np.array([c, s, 0.0])     # normal, out of the marker
    y = np.array([0.0, 0.0, 1.0])  # up
    x = np.cross(y, z)              # right, seen from the front
    return np.column_stack([x, y, z])


def load_marker_map(path):
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    default = float(cfg.get("marker_size", 0.15))
    markers = {}
    for m in cfg["markers"]:
        mk = Marker(
            id=int(m["id"]),
            size=float(m.get("size", default)),
            t=np.array([m["x"], m["y"], m["z"]], dtype=float),
            R=rotation_from_yaw(math.radians(float(m["yaw_deg"]))),
        )
        if mk.id in markers:
            raise ValueError(f"duplicate marker id {mk.id} in {path}")
        markers[mk.id] = mk
    return cfg, markers


def rpy_from_matrix(R):
    """ZYX Euler angles (roll, pitch, yaw) such that R = Rz(yaw) Ry(pitch) Rx(roll)."""
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def quat_from_matrix(R):
    """Quaternion (x, y, z, w) from a rotation matrix."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return x, y, z, w


def matrix_from_quat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def aruco_dictionary(name):
    import cv2
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def marker_image(dictionary_name, marker_id, px_per_module=100):
    """Marker image with a one-module white quiet zone. Works with OpenCV 4.6 and >= 4.7."""
    import cv2
    d = aruco_dictionary(dictionary_name)
    bits = d.markerSize if hasattr(d, "markerSize") else int(round(math.sqrt(d.bytesList.shape[1] * 8)))
    side = (bits + 2) * px_per_module
    if hasattr(cv2.aruco, "generateImageMarker"):
        img = cv2.aruco.generateImageMarker(d, marker_id, side)
    else:
        img = cv2.aruco.drawMarker(d, marker_id, side)
    img = cv2.copyMakeBorder(img, px_per_module, px_per_module, px_per_module, px_per_module,
                             cv2.BORDER_CONSTANT, value=255)
    return img, bits
