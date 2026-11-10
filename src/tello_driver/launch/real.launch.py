"""Real drone: Tello driver + marker frames + ArUco localizer + RViz.

    ros2 launch tello_driver real.launch.py                      # real Tello, RViz
    ros2 launch tello_driver real.launch.py mock:=true           # fake drone (no hardware), to test the setup
    ros2 launch tello_driver real.launch.py camera_info_file:=$HOME/tello_ws/src/tello-balloon-hunter/config/tello_camera.yaml

Then, in a second terminal:  ros2 run tello_driver tello_teleop
"""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

REPO = os.environ.get("TELLO_REPO", str(Path.home() / "tello_ws/src/tello-balloon-hunter"))
LOCAL_ROS_ENV = {"ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST", "FASTDDS_BUILTIN_TRANSPORTS": "LARGE_DATA"}


def setup(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    # Remove shared-memory segments left by ROS processes that were killed (Ctrl+C twice, crash, kill -9):
    # with the local-only settings, stale segments can silently block RViz from receiving anything.
    import subprocess
    try:
        subprocess.run(["fastdds", "shm", "clean"], timeout=15, capture_output=True)
    except Exception:  # noqa: BLE001
        pass
    share = get_package_share_directory("tello_driver")
    calib = lc("camera_info_file")
    if calib == "auto":
        default = Path(REPO) / "config" / "tello_camera.yaml"
        calib = str(default) if default.exists() else ""
    nodes = [
        Node(package="tello_driver", executable="tello_driver", output="screen",
             parameters=[{"mock": lc("mock") == "true", "camera_info_file": calib}]),
        Node(package="aruco_localization", executable="marker_map_publisher", output="screen",
             parameters=[{"markers_file": lc("markers")}]),
        Node(package="aruco_localization", executable="aruco_localizer", output="screen",
             parameters=[{"markers_file": lc("markers"), "child_frame": "base_link",
                          "debug_every_n": 3, "debug_scale": 0.5}]),
    ]
    if lc("record") == "true":
        nodes.append(Node(package="tello_driver", executable="session_recorder", output="screen"))
    if lc("rviz") == "true":
        nodes.append(Node(package="rviz2", executable="rviz2", output="log",
                          arguments=["-d", rviz_config_centred_on_markers(os.path.join(share, "config", "real.rviz"), lc("markers"))]))
    return nodes


def rviz_config_centred_on_markers(template, markers_file):
    """Copy of the RViz config whose camera looks at the centre of the real marker map."""
    import yaml
    try:
        ms = yaml.safe_load(open(markers_file, encoding="utf-8"))["markers"]
        cx = sum(m["x"] for m in ms) / len(ms)
        cy = sum(m["y"] for m in ms) / len(ms)
        cz = sum(m["z"] for m in ms) / len(ms)
        span = max(max(m["x"] for m in ms) - min(m["x"] for m in ms), max(m["y"] for m in ms) - min(m["y"] for m in ms))
    except Exception:  # noqa: BLE001
        return template
    text = open(template, encoding="utf-8").read()
    text = text.replace("Focal Point: {X: 1.5, Y: 1.5, Z: 0.8}", f"Focal Point: {{X: {cx:.2f}, Y: {cy:.2f}, Z: {cz:.2f}}}")
    text = text.replace("Distance: 6", f"Distance: {max(4.0, 1.8 * span):.1f}")
    out = Path("/tmp") / "tello_real_centred.rviz"
    out.write_text(text, encoding="utf-8")
    return str(out)


def generate_launch_description():
    return LaunchDescription([
        # All nodes run on this PC. Keep ROS 2 discovery and data local (localhost + shared memory) so that
        # switching Windows to the Tello Wi-Fi (no multicast, no internet) does not cut the nodes off.
        *[SetEnvironmentVariable(k, v) for k, v in LOCAL_ROS_ENV.items()],
        DeclareLaunchArgument("mock", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("record", default_value="true", description="save images, poses, telemetry in ~/tello_ws/sessions"),
        DeclareLaunchArgument("markers", default_value=os.path.join(REPO, "config", "markers.yaml")),
        DeclareLaunchArgument("camera_info_file", default_value="auto",
                              description="calibration YAML; auto = config/tello_camera.yaml if it exists"),
        OpaqueFunction(function=setup),
    ])
