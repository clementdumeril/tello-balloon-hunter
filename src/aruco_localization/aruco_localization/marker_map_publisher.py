"""Publish the static frames: map -> aruco_<id> (from markers.yaml) and base_link -> tello_camera_optical.

Also publishes the markers as a MarkerArray (/aruco/map_markers) for RViz.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker as VizMarker
from visualization_msgs.msg import MarkerArray

from aruco_localization.marker_map import load_marker_map, quat_from_matrix


def camera_mount_rotation():
    """Rotation of the optical frame (z forward, x right, y down) in the body frame (x fwd, y left, z up)."""
    return np.column_stack([[0, -1, 0], [0, 0, -1], [1, 0, 0]]).astype(float)


def make_tf(stamp, parent, child, t, R):
    m = TransformStamped()
    m.header.stamp = stamp
    m.header.frame_id = parent
    m.child_frame_id = child
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, t)
    q = quat_from_matrix(R)
    m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = q
    return m


class MarkerMapPublisher(Node):
    def __init__(self):
        super().__init__("marker_map_publisher")
        path = self.declare_parameter("markers_file", "").value
        cam_xyz = self.declare_parameter("camera_offset", [0.05, 0.0, 0.0]).value
        self.cfg, self.markers = load_marker_map(path)
        stamp = self.get_clock().now().to_msg()

        tfs = [make_tf(stamp, "map", f"aruco_{m.id}", m.t, m.R) for m in self.markers.values()]
        tfs.append(make_tf(stamp, "base_link", "tello_camera_optical", cam_xyz, camera_mount_rotation()))
        self.static = StaticTransformBroadcaster(self)
        self.static.sendTransform(tfs)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, "/aruco/map_markers", qos)
        arr = MarkerArray()
        for m in self.markers.values():
            v = VizMarker()
            v.header.frame_id = f"aruco_{m.id}"
            v.header.stamp = stamp
            v.ns, v.id, v.type, v.action = "aruco", m.id, VizMarker.CUBE, VizMarker.ADD
            v.scale.x, v.scale.y, v.scale.z = m.size, m.size, 0.005
            v.color.r = v.color.g = v.color.b = 0.1
            v.color.a = 1.0
            v.pose.orientation.w = 1.0
            arr.markers.append(v)
            t = VizMarker()
            t.header = v.header
            t.ns, t.id, t.type, t.action = "aruco_id", m.id, VizMarker.TEXT_VIEW_FACING, VizMarker.ADD
            t.pose.position.z = 0.05
            t.pose.position.y = m.size * 0.8
            t.pose.orientation.w = 1.0
            t.scale.z = 0.06
            t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 0.8, 0.0, 1.0
            t.text = str(m.id)
            arr.markers.append(t)
        self.pub.publish(arr)
        self.get_logger().info(f"Published {len(self.markers)} marker frames from {path}")


def main():
    rclpy.init()
    node = MarkerMapPublisher()
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
