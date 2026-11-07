"""ArUco localizer: one joint solvePnP over the corners of ALL visible markers -> drone pose in `map`.

Subscribes: /tello/image_raw, /tello/camera_info
Publishes:  /aruco/pose (PoseWithCovarianceStamped, frame map, pose of base_link)
            TF map -> base_link_aruco (visualization only: base_link itself belongs to the EKF)
            /aruco/debug_image (annotated, throttled)
Works with OpenCV 4.6 (Ubuntu 24.04 system package) and >= 4.7.
"""
import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

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
        self.max_reproj = p("max_reprojection_px", 3.0).value
        self.debug_every = p("debug_every_n", 5).value
        self.debug_dir = p("debug_dir", "").value
        # TF child frame for the ArUco pose. base_link_aruco by default (base_link belongs to the EKF);
        # the real-drone launch sets base_link until the EKF exists, so RViz shows the camera on the drone.
        self.child_frame = p("child_frame", "base_link_aruco").value
        self.detector = Detector(self.cfg.get("dictionary", "DICT_4X4_50"))
        self.bridge = CvBridge()
        self.K = self.D = None
        self.T_base_cam = None
        self.n = 0
        self.saved = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, "/aruco/pose", 10)
        self.pub_dbg = self.create_publisher(Image, "/aruco/debug_image", 2)
        self.create_subscription(CameraInfo, "/tello/camera_info", self.on_info, 5)
        self.create_subscription(Image, "/tello/image_raw", self.on_image, 5)
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
            obj_pts = np.concatenate(obj).astype(np.float64)
            img_pts = np.concatenate(imgp).astype(np.float64)
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
                ok = err <= self.max_reproj
        if ok:
            self.publish(msg.header.stamp, T_map_base, err, dist, len(used))
        if self.debug_every and self.n % self.debug_every == 0:
            self.publish_debug(msg, img, corners, ids, ok, err, used)

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

        t = TransformStamped()
        t.header = m.header
        t.child_frame_id = self.child_frame
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, T[:3, 3])
        t.transform.rotation = m.pose.pose.orientation
        self.tf_pub.sendTransform(t)

    def publish_debug(self, msg, img, corners, ids, ok, err, used):
        dbg = img.copy()
        if ids is not None and len(ids):
            cv2.aruco.drawDetectedMarkers(dbg, corners, ids)
        txt = f"{len(used)} markers  reproj {err:.2f}px  {'OK' if ok else 'REJECTED'}" if used else "no marker"
        cv2.putText(dbg, txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if ok else (0, 0, 255), 2)
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
