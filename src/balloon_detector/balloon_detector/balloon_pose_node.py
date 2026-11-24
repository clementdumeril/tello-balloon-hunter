#!/usr/bin/env python3
"""ROS 2 balloon pose node: turns 2D detections into a 3D balloon position.

Takes the pixel bounding boxes published by detector_node, estimates the
balloon's distance from its apparent size, back-projects the box center into
a 3D point in the camera frame, and publishes it as a pose and a TF frame.

Method (pinhole camera model):
    d_px = (box_width + box_height) / 2           apparent diameter, pixels
    Z    = f * D_real / d_px                      depth along optical axis
    X    = x_n * Z,  Y = y_n * Z                  (x_n, y_n) = normalized
                                                  image coords of the center
This is the same as Z = f * sqrt(A_real / A_px) with A_real = pi * (D/2)^2,
i.e. it assumes the balloon is a sphere of known diameter.

Subscribes:
    /detections    (vision_msgs/Detection2DArray)  from detector_node
    /camera_info   (sensor_msgs/CameraInfo)        from usb_cam

Publishes:
    /balloon_pose  (geometry_msgs/PoseStamped)     in the camera optical frame
    TF             camera frame -> 'balloon'       (if publish_tf is true)

Camera optical frame convention: x right, y down, z forward (out of lens).
Orientation is identity, since a balloon has no meaningful orientation.

Parameters:
    balloon_diameter_m  (float, 0.30)  REAL diameter of your inflated balloon;
                                       measure it, the distance scales with it
    target_class        (str,   '')    only use detections with this class_id
                                       ('' = any class)
    camera_frame        (str,   '')    frame for the pose ('' = use the frame
                                       in the detection's header)
    child_frame         (str,   'balloon')  TF child frame name
    publish_tf          (bool,  True)
    border_margin_px    (float, 5.0)   ignore boxes this close to the image
                                       edge (a clipped balloon looks too small)
    min_diameter_px     (float, 8.0)   ignore very small boxes (too noisy)
    smoothing_alpha     (float, 0.5)   1.0 = no smoothing, lower = smoother
    track_timeout_s     (float, 1.0)   reset smoothing after this long without
                                       a valid detection
    undistort           (bool,  True)  correct lens distortion on the center
                                       (uses D from camera_info if present)
    fallback_hfov_deg   (float, 0.0)   rough stand-in for calibration: if
                                       camera_info has no intrinsics, estimate
                                       the focal length from this horizontal
                                       field of view. 0 = disabled.
                                       Real calibration is strongly preferred.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformBroadcaster
from vision_msgs.msg import Detection2DArray


class BalloonPoseNode(Node):
    def __init__(self):
        super().__init__('balloon_pose_node')

        # ---- parameters ----
        self.declare_parameter('balloon_diameter_m', 0.30) # TODO: Update with true measured balloon diameter in meters
        self.declare_parameter('target_class', '') # TODO: Define target class as "sports ball"
        self.declare_parameter('camera_frame', '')
        self.declare_parameter('child_frame', 'balloon')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('border_margin_px', 5.0)
        self.declare_parameter('min_diameter_px', 8.0)
        self.declare_parameter('smoothing_alpha', 0.5)
        self.declare_parameter('track_timeout_s', 1.0)
        self.declare_parameter('undistort', True)
        self.declare_parameter('fallback_hfov_deg', 0.0)

        get = lambda name: self.get_parameter(name).value
        self.diameter = float(get('balloon_diameter_m'))
        self.target_class = get('target_class')
        self.camera_frame = get('camera_frame')
        self.child_frame = get('child_frame')
        self.publish_tf = bool(get('publish_tf'))
        self.border_margin_px = float(get('border_margin_px'))
        self.min_diameter_px = float(get('min_diameter_px'))
        self.alpha = min(max(float(get('smoothing_alpha')), 0.01), 1.0)
        self.timeout = float(get('track_timeout_s'))
        self.undistort = bool(get('undistort'))
        self.fallback_hfov_deg = float(get('fallback_hfov_deg'))

        if self.diameter <= 0.0:
            raise ValueError('balloon_diameter_m must be greater than 0')

        # ---- camera intrinsics (filled in from camera_info) ----
        self.fx = self.fy = self.cx = self.cy = None
        self.img_w = self.img_h = 0
        self.dist = []
        self.dist_model = ''
        self._source = None

        # ---- smoothing state ----
        self.filt = None
        self.last_t = None

        # ---- ROS interfaces ----
        self.create_subscription(CameraInfo, 'camera_info', self.on_camera_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, 'detections',
                                 self.on_detections, 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'balloon_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.get_logger().info(
            f'Balloon diameter {self.diameter:.3f} m; waiting for camera_info '
            'and detections...')

    # ------------------------------------------------------------------
    # camera intrinsics
    # ------------------------------------------------------------------
    def on_camera_info(self, msg: CameraInfo):
        self.img_w, self.img_h = int(msg.width), int(msg.height)
        self.dist = list(msg.d)
        self.dist_model = msg.distortion_model

        fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        if fx > 0.0 and fy > 0.0:
            self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
            source = 'camera_info (calibrated)'
        elif self.fallback_hfov_deg > 0.0 and self.img_w > 0:
            f = (self.img_w / 2.0) / math.tan(math.radians(self.fallback_hfov_deg) / 2.0)
            self.fx = self.fy = f
            self.cx, self.cy = self.img_w / 2.0, self.img_h / 2.0
            self.dist = []  # no distortion model for the fallback
            source = f'FALLBACK from {self.fallback_hfov_deg:.0f} deg hfov (rough)'
        else:
            self.fx = None
            source = None

        if source != self._source:
            self._source = source
            if source:
                self.get_logger().info(
                    f'Intrinsics from {source}: fx={self.fx:.1f} fy={self.fy:.1f} '
                    f'cx={self.cx:.1f} cy={self.cy:.1f} ({self.img_w}x{self.img_h})')

    def _normalized(self, u, v):
        """Pixel -> normalized image coordinates, undistorted if possible."""
        xd = (u - self.cx) / self.fx
        yd = (v - self.cy) / self.fy

        d = self.dist
        if (not self.undistort or len(d) < 4 or not any(d)
                or self.dist_model != 'plumb_bob'):
            return xd, yd

        k1, k2, p1, p2 = d[0], d[1], d[2], d[3]
        k3 = d[4] if len(d) > 4 else 0.0
        x, y = xd, yd
        for _ in range(5):  # iterative inverse of the plumb_bob model
            r2 = x * x + y * y
            radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
            dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
            dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
            x = (xd - dx) / radial
            y = (yd - dy) / radial
        return x, y

    # ------------------------------------------------------------------
    # detections
    # ------------------------------------------------------------------
    def _select(self, msg: Detection2DArray):
        """Return (score, u, v, diameter_px) of the best usable detection."""
        best = None
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            if self.target_class and hyp.class_id != self.target_class:
                continue

            u = det.bbox.center.position.x
            v = det.bbox.center.position.y
            w, h = det.bbox.size_x, det.bbox.size_y
            d_px = 0.5 * (w + h)
            if d_px < self.min_diameter_px:
                continue

            if self.img_w > 0 and self.img_h > 0:
                m = self.border_margin_px
                if (u - w / 2.0 < m or v - h / 2.0 < m
                        or u + w / 2.0 > self.img_w - m
                        or v + h / 2.0 > self.img_h - m):
                    continue  # touches the image edge: size is unreliable

            if best is None or hyp.score > best[0]:
                best = (hyp.score, u, v, d_px)
        return best

    def _smooth(self, pos, t):
        if (self.filt is None or self.last_t is None
                or not (0.0 <= t - self.last_t <= self.timeout)):
            self.filt = pos
        else:
            a = self.alpha
            self.filt = tuple(a * n + (1.0 - a) * o for n, o in zip(pos, self.filt))
        self.last_t = t
        return self.filt

    def on_detections(self, msg: Detection2DArray):
        if self.fx is None:
            self.get_logger().warn(
                'No usable camera intrinsics yet. Calibrate the camera, or set '
                'fallback_hfov_deg for a rough test.', throttle_duration_sec=5.0)
            return

        best = self._select(msg)
        if best is None:
            return
        score, u, v, d_px = best

        # Depth from apparent size, then back-project the center pixel.
        f = 0.5 * (self.fx + self.fy)
        z = f * self.diameter / d_px
        xn, yn = self._normalized(u, v)
        pos = (xn * z, yn * z, z)

        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        x, y, z = self._smooth(pos, t)

        frame = self.camera_frame or msg.header.frame_id
        if not frame:
            self.get_logger().error(
                'No frame_id: set the camera_frame parameter.',
                throttle_duration_sec=5.0)
            return

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = frame
            tf.child_frame_id = self.child_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = z
            tf.transform.rotation.w = 1.0
            self.tf_broadcaster.send_transform(tf)

        self.get_logger().info(
            f'balloon at x={x:+.2f} y={y:+.2f} z={z:.2f} m '
            f'(diameter {d_px:.0f} px, score {score:.2f})',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = BalloonPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
