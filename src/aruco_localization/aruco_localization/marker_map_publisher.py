"""Publish the static frames: map -> aruco_<id> (from markers.yaml) and base_link -> tello_camera_optical.

Also publishes, for RViz: the markers (/aruco/map_markers) and a drone body + camera field of view
attached to base_link (/tello/body_markers).
"""
import math

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point, TransformStamped
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
        self.pub.publish(self.map_markers(stamp))
        # Drone body + camera field of view, attached to base_link: they appear (and follow the drone)
        # as soon as the localizer publishes map -> base_link. Separate topic so that the marker display
        # never shows a "no transform" warning before the first pose.
        self.pub_body = self.create_publisher(MarkerArray, "/tello/body_markers", qos)
        # stamp 0 = "use the latest transform": the drone frame moves, an old stamp would be looked up in the past
        self.body = self.body_markers(TimeMsg())
        self.pub_body.publish(self.body)
        self.create_timer(1.0, lambda: self.pub_body.publish(self.body))
        # the scanned room (tools/room_mesh_to_map.py), shown/hidden with its checkbox in RViz
        mesh = self.declare_parameter("room_mesh", "").value
        if mesh:
            self.pub_room = self.create_publisher(VizMarker, "/room/mesh", qos)
            m = viz(TimeMsg(), "map", "room", 0, VizMarker.MESH_RESOURCE, (1, 1, 1), (0.0, 0.0, 0.0, 0.0))
            m.mesh_resource = "file://" + str(mesh)
            m.mesh_use_embedded_materials = True  # the scan's own texture
            self.pub_room.publish(m)
            self.get_logger().info(f"Room mesh: {mesh}")
        self.get_logger().info(f"Published {len(self.markers)} marker frames from {path}")

    def map_markers(self, stamp):
        arr = MarkerArray()
        for m in self.markers.values():
            frame = f"aruco_{m.id}"
            # white sheet slightly behind, black square in front: looks like the printed marker, visible on a dark background
            arr.markers.append(viz(stamp, frame, "aruco_sheet", m.id, VizMarker.CUBE, (1.5 * m.size, 1.5 * m.size, 0.004),
                                   (0.95, 0.95, 0.95, 1.0), z=-0.004))
            arr.markers.append(viz(stamp, frame, "aruco", m.id, VizMarker.CUBE, (m.size, m.size, 0.006), (0.05, 0.05, 0.05, 1.0)))
            t = viz(stamp, frame, "aruco_id", m.id, VizMarker.TEXT_VIEW_FACING, (0, 0, 0.14), (1.0, 0.85, 0.0, 1.0),
                    y=m.size * 1.1, z=0.03)
            t.text = str(m.id)
            arr.markers.append(t)
        # map origin: a small sphere so the reference point is obvious
        arr.markers.append(viz(stamp, "map", "origin", 0, VizMarker.SPHERE, (0.08, 0.08, 0.08), (1.0, 1.0, 1.0, 1.0)))
        return arr

    def body_markers(self, stamp):
        """A Tello-like body (enlarged x2 so it is visible from across the room) and the camera field of view."""
        arr = MarkerArray()
        add = arr.markers.append
        add(viz(stamp, "base_link", "body", 0, VizMarker.CUBE, (0.20, 0.18, 0.08), (0.85, 0.85, 0.85, 1.0)))
        for k, (x, y) in enumerate([(0.13, 0.13), (0.13, -0.13), (-0.13, 0.13), (-0.13, -0.13)]):
            front = x > 0
            add(viz(stamp, "base_link", "props", k, VizMarker.CYLINDER, (0.14, 0.14, 0.01),
                    (1.0, 0.2, 0.2, 0.9) if front else (0.2, 0.2, 0.2, 0.9), x=x, y=y, z=0.045))
        arrow = viz(stamp, "base_link", "heading", 0, VizMarker.ARROW, (0.35, 0.03, 0.03), (1.0, 0.2, 0.2, 1.0))
        add(arrow)
        # camera field of view (55.6 x 43 deg), 1 m deep, in the optical frame (z forward, x right, y down)
        d = 1.0
        hx, hy = d * math.tan(math.radians(27.8)), d * math.tan(math.radians(21.5))
        c = [(-hx, -hy, d), (hx, -hy, d), (hx, hy, d), (-hx, hy, d)]
        fov = viz(stamp, "tello_camera_optical", "camera_fov", 0, VizMarker.LINE_LIST, (0.012, 0, 0), (0.0, 0.9, 1.0, 1.0))
        for i in range(4):
            fov.points += [point(0, 0, 0), point(*c[i]), point(*c[i]), point(*c[(i + 1) % 4])]
        add(fov)
        return arr


def point(x, y, z):
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def viz(stamp, frame, ns, mid, mtype, scale, rgba, x=0.0, y=0.0, z=0.0):
    v = VizMarker()
    v.header.frame_id = frame
    v.header.stamp = stamp
    v.ns, v.id, v.type, v.action = ns, mid, mtype, VizMarker.ADD
    v.frame_locked = True  # re-transformed every frame: follows the drone when the frame moves
    v.scale.x, v.scale.y, v.scale.z = (float(a) for a in scale)
    v.color.r, v.color.g, v.color.b, v.color.a = (float(a) for a in rgba)
    v.pose.position.x, v.pose.position.y, v.pose.position.z = float(x), float(y), float(z)
    v.pose.orientation.w = 1.0
    return v


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
