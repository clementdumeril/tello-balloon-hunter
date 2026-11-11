"""ArUco localizer: one joint solvePnP over the corners of ALL visible markers -> drone pose in `map`.

Subscribes: /tello/image_raw, /tello/camera_info
Publishes:  /aruco/pose (PoseWithCovarianceStamped, frame map, pose of base_link)
            TF map -> base_link_aruco (visualization only: base_link itself belongs to the EKF)
            /aruco/debug_image (annotated, throttled)
            /aruco/path (nav_msgs/Path, trail of the poses) and /aruco/sight_lines (camera -> seen markers), for RViz
Works with OpenCV 4.6 (Ubuntu 24.04 system package) and >= 4.7.
"""
import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker as VizMarker

from aruco_localization.marker_map import (aruco_dictionary, load_marker_map, matrix_from_quat,
                                           quat_from_matrix)


class Detector:
    def __init__(self, dict_name):
        self.dict = aruco_dictionary(dict_name)
        if hasattr(cv2.aruco, "ArucoDetector"):
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.det = cv2.aruco.ArucoDetector(self.dict, params)
            self.params = None
        else:
            self.params = cv2.aruco.DetectorParameters_create()
            self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.det = None

    def detect(self, gray):
        if self.det is not None:
            corners, ids, _ = self.det.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dict, parameters=self.params)
        return corners, ids


class ArucoLocalizer(Node):
    def __init__(self):
        super().__init__("aruco_localizer")
        p = self.declare_parameter
        self.cfg, self.markers = load_marker_map(p("markers_file", "").value)
        # reprojection error limit: strict for one marker (flips), looser for several (a 1 cm map error is already
        # ~6 px at 1.6 m; a wrong multi-marker pose is tens of px off)
        self.max_reproj = p("max_reprojection_px", 3.0).value
        self.max_reproj_multi = p("max_reprojection_px_multi", 8.0).value
        self.debug_every = p("debug_every_n", 5).value
        self.debug_scale = p("debug_scale", 1.0).value  # < 1 shrinks the debug image (lighter for RViz)
        self.debug_dir = p("debug_dir", "").value
        # TF child frame for the ArUco pose. base_link_aruco by default (base_link belongs to the EKF);
        # the real-drone launch sets base_link until the EKF exists, so RViz shows the camera on the drone.
        self.child_frame = p("child_frame", "base_link_aruco").value
        self.detector = Detector(self.cfg.get("dictionary", "DICT_4X4_50"))
        self.bridge = CvBridge()
        self.K = self.D = None
        self.T_base_cam = None
        self.n = 0
        self.ambiguous = 0
        self.saved = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, "/aruco/pose", 10)
        self.pub_dbg = self.create_publisher(Image, "/aruco/debug_image", 2)
        # proximity sensor: distance to the closest marker in view, from its size in the image only (no map,
        # no pose): f * size / longest edge. The marker is on the wall in front: the driver stops before it.
        from std_msgs.msg import Float32
        self.Float32 = Float32
        self.pub_near = self.create_publisher(Float32, "/aruco/nearest_marker", 5)
        self.pub_path = self.create_publisher(PathMsg, "/aruco/path", 2)
        self.pub_sight = self.create_publisher(VizMarker, "/aruco/sight_lines", 5)
        self.path = PathMsg()
        self.path.header.frame_id = "map"
        self.path_max = p("path_length", 2000).value
        self.last_path_pub = 0.0
        self.create_subscription(CameraInfo, "/tello/camera_info", self.on_info, 5)
        # depth 1: always process the most recent frame; never queue up old ones (that only adds latency)
        self.create_subscription(Image, "/tello/image_raw", self.on_image, 1)
        if self.debug_dir:
            Path(self.debug_dir).mkdir(parents=True, exist_ok=True)

    def on_info(self, m):
        self.K = np.array(m.k, dtype=float).reshape(3, 3)
        self.D = np.array(m.d, dtype=float) if len(m.d) else np.zeros(5)

    def camera_mount(self):
        if self.T_base_cam is None:
            try:
                t = self.tf_buffer.lookup_transform("base_link", "tello_camera_optical", Time())
            except Exception:  # noqa: BLE001
                return None
            q = t.transform.rotation
            T = np.eye(4)
            T[:3, :3] = matrix_from_quat(q.x, q.y, q.z, q.w)
            T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
            self.T_base_cam = T
        return self.T_base_cam

    def on_image(self, msg):
        if self.K is None or self.camera_mount() is None:
            return
        self.n += 1
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids = self.detector.detect(gray)

        obj, imgp, used = [], [], []
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                mk = self.markers.get(int(i))
                if mk is None:
                    continue
                obj.append(mk.corners_map())
                imgp.append(c.reshape(4, 2))
                used.append(int(i))
        ok, T_map_base, err, dist = False, None, float("nan"), float("nan")
        if used:
            fx = self.K[0, 0]
            near = min(fx * self.markers[i].size / max(np.max(np.linalg.norm(c - np.roll(c, 1, 0), axis=1)), 1.0)
                       for i, c in zip(used, imgp))
            self.pub_near.publish(self.Float32(data=float(near)))
            obj_pts = np.concatenate(obj).astype(np.float64)
            img_pts = np.concatenate(imgp).astype(np.float64)
            if len(used) == 1:
                # one marker: two mirror poses fit; take IPPE's best one, and only if it is clearly the best
                # (SQPnP may converge to the mirror pose: 1.4 m errors seen with the fake drone)
                ok, rvec, tvec = self.single_marker_pose(imgp[0], self.markers[used[0]])
                if not ok:
                    self.ambiguous += 1
            else:
                flag = cv2.SOLVEPNP_SQPNP if hasattr(cv2, "SOLVEPNP_SQPNP") else cv2.SOLVEPNP_ITERATIVE
                ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.K, self.D, flags=flag)
            if ok:
                rvec, tvec = cv2.solvePnPRefineLM(obj_pts, img_pts, self.K, self.D, rvec, tvec)
                proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, self.K, self.D)
                err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img_pts) ** 2, axis=1))))
                R_cm, _ = cv2.Rodrigues(rvec)  # map -> camera
                T_map_cam = np.eye(4)
                T_map_cam[:3, :3] = R_cm.T
                T_map_cam[:3, 3] = (-R_cm.T @ tvec).ravel()
                T_map_base = T_map_cam @ np.linalg.inv(self.T_base_cam)
                dist = float(np.mean(np.linalg.norm(obj_pts - T_map_cam[:3, 3], axis=1)))
                lim = self.max_reproj if len(used) == 1 else self.max_reproj_multi
                ok = err <= lim and self.plausible(T_map_base[:3, 3], len(used))
        if ok:
            self._last_cam = (self.get_clock().now().nanoseconds * 1e-9, T_map_cam[:3, 3].copy())
            self.publish(msg.header.stamp, T_map_base, err, dist, len(used))
            self.publish_viz(msg.header.stamp, T_map_cam[:3, 3], used)
        if self.debug_every and self.n % self.debug_every == 0:
            self.publish_debug(msg, img, corners, ids, ok, err, used)

    def single_marker_pose(self, corners, mk, min_ratio=3.0, max_tilt_deg=30.0):
        """Pose from ONE marker, as map -> camera (rvec, tvec); ok=False if it cannot be disambiguated.

        IPPE gives two mirror poses. Choice, in order:
          1. drop poses where the camera is tilted > 30 deg (the Tello camera is level, the drone tilts < 15 deg);
          2. one clearly better reprojection (ratio >= 3) -> it;
          3. both poses (almost) at the same place -> the best one (head-on view: rejecting it lost the drone
             in front of marker 10, Nov 10);
          4. the one closest to the last pose (< 1 s old, < 0.4 m away);
          5. otherwise reject."""
        h = mk.size / 2.0
        local = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
        try:
            n, rvecs, tvecs, errs = cv2.solvePnPGeneric(local, corners.astype(np.float64), self.K, self.D,
                                                        flags=cv2.SOLVEPNP_IPPE_SQUARE)
        except cv2.error:
            return False, None, None
        if n == 0:
            return False, None, None
        e = np.ravel(errs) if errs is not None else np.zeros(n)
        cands = []
        for k in range(n):
            R_mc, _ = cv2.Rodrigues(rvecs[k])          # marker -> camera
            R_cmap = R_mc @ mk.R.T                      # map -> camera
            t = np.asarray(tvecs[k], float).reshape(3, 1) - R_cmap @ mk.t.reshape(3, 1)
            cam = (-R_cmap.T @ t).ravel()               # camera position in map
            down = R_cmap.T[:, 1]                       # camera 'down' axis in map
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, -down[2]))))
            if tilt <= max_tilt_deg:
                cands.append((e[k], cam, R_cmap, t))
        if not cands:
            return False, None, None
        cands.sort(key=lambda c: c[0])
        pick = cands[0]
        if len(cands) >= 2:
            ratio = cands[1][0] / max(cands[0][0], 1e-6)
            same = np.linalg.norm(cands[0][1] - cands[1][1]) < 0.15
            if ratio < min_ratio and not same:
                last = getattr(self, "_last_cam", None)
                now = self.get_clock().now().nanoseconds * 1e-9
                if last is None or now - last[0] > 1.0:
                    return False, None, None
                pick = min(cands[:2], key=lambda c: np.linalg.norm(c[1] - last[1]))
                if np.linalg.norm(pick[1] - last[1]) > 0.4:
                    return False, None, None
        rvec, _ = cv2.Rodrigues(pick[2])
        return True, rvec, pick[3]

    def plausible(self, pos, n_markers, max_jump=0.5, window=0.5):
        """Reject a single-marker pose that jumps far from the previous one (a flip the ratio test missed).
        Poses from 2+ markers are trusted and always accepted (they re-anchor the gate)."""
        now = self.get_clock().now().nanoseconds * 1e-9
        last = getattr(self, "_last_ok", None)
        if n_markers == 1 and last is not None and now - last[0] < window and np.linalg.norm(pos - last[1]) > max_jump:
            self.ambiguous += 1
            return False
        self._last_ok = (now, np.array(pos, float))
        return True

    def publish(self, stamp, T, err, dist, n_markers):
        fx = self.K[0, 0]
        # crude covariance model: pixel error -> metric error at the marker distance, better with more markers
        sigma_p = max(0.01, 1.5 * max(err, 0.3) * dist / fx / math.sqrt(n_markers))
        sigma_yaw = max(math.radians(0.5), sigma_p / max(dist, 0.3))
        q = quat_from_matrix(T[:3, :3])
        m = PoseWithCovarianceStamped()
        m.header.stamp = stamp
        m.header.frame_id = "map"
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = map(float, T[:3, 3])
        m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w = q
        cov = np.zeros(36)
        for k in range(3):
            cov[k * 7] = sigma_p ** 2
        for k in range(3, 6):
            cov[k * 7] = sigma_yaw ** 2
        m.pose.covariance = cov.tolist()
        self.pub_pose.publish(m)
        self.last_pose = m.pose.pose

        t = TransformStamped()
        t.header = m.header
        t.child_frame_id = self.child_frame
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, T[:3, 3])
        t.transform.rotation = m.pose.pose.orientation
        self.tf_pub.sendTransform(t)

    def publish_viz(self, stamp, cam_pos, used):
        """RViz helpers: trail of the drone poses, and one line from the camera to each marker it used."""
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = "map"
        ps.pose = self.last_pose
        self.path.poses.append(ps)
        del self.path.poses[:-self.path_max]
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_path_pub > 0.2:  # 5 Hz is plenty for a trail
            self.path.header.stamp = stamp
            self.pub_path.publish(self.path)
            self.last_path_pub = now
        m = VizMarker()
        m.header.stamp = stamp
        m.header.frame_id = "map"
        m.ns, m.id, m.type, m.action = "sight", 0, VizMarker.LINE_LIST, VizMarker.ADD
        m.scale.x = 0.01
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 1.0, 0.2, 0.9
        m.pose.orientation.w = 1.0
        m.lifetime.nanosec = 500_000_000  # disappears half a second after the markers are lost
        for i in used:
            c = self.markers[i].t
            m.points += [Point(x=float(cam_pos[0]), y=float(cam_pos[1]), z=float(cam_pos[2])),
                         Point(x=float(c[0]), y=float(c[1]), z=float(c[2]))]
        self.pub_sight.publish(m)

    def publish_debug(self, msg, img, corners, ids, ok, err, used):
        dbg = img.copy()
        if ids is not None and len(ids):
            cv2.aruco.drawDetectedMarkers(dbg, corners, ids)
        txt = f"{len(used)} markers  reproj {err:.2f}px  {'OK' if ok else 'REJECTED'}" if used else "no marker"
        cv2.putText(dbg, txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if ok else (0, 0, 255), 2)
        if self.debug_scale != 1.0:
            dbg = cv2.resize(dbg, None, fx=self.debug_scale, fy=self.debug_scale, interpolation=cv2.INTER_AREA)
        out = self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8")
        out.header = msg.header
        self.pub_dbg.publish(out)
        if self.debug_dir and self.saved < 40:
            cv2.imwrite(str(Path(self.debug_dir) / f"debug_{self.n:05d}.jpg"), dbg)
            self.saved += 1


def main():
    rclpy.init()
    node = ArucoLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
