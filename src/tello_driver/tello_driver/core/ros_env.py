"""Local-only ROS 2 settings shared by every program of the project (must run before rclpy.init)."""
import os
from pathlib import Path


def local_ros_env():
    """All nodes run on this PC. Keep discovery and data local (localhost + shared memory) so that switching
    Windows to the Tello Wi-Fi does not cut the nodes off, and use config/fastdds_local.xml: UDP on 127.0.0.1
    only, packets <= 1400 bytes (WSL mirrored networking drops fragmented UDP on the loopback; with Tailscale
    up the discovery messages grew past 1500 bytes and no node could see the others, Nov 10)."""
    os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "LARGE_DATA")
    if "FASTRTPS_DEFAULT_PROFILES_FILE" not in os.environ:
        xml = fastdds_profile()
        if xml:
            os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = xml


def fastdds_profile():
    try:
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory("tello_driver")) / "config" / "fastdds_local.xml"
        return str(p) if p.exists() else ""
    except Exception:  # noqa: BLE001
        return ""
