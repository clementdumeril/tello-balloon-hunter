"""Real drone: Tello driver + marker frames + ArUco localizer + RViz.

    ros2 launch tello_driver real.launch.py                      # real Tello, RViz
    ros2 launch tello_driver real.launch.py mock:=true           # fake drone (no hardware), to test the setup
    ros2 launch tello_driver real.launch.py camera_info_file:=$HOME/tello_ws/src/tello-balloon-hunter/config/tello_camera.yaml

Then, in a second terminal:  ros2 run tello_driver tello_teleop
"""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

REPO = os.environ.get("TELLO_REPO", str(Path.home() / "tello_ws/src/tello-balloon-hunter"))


def setup(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
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
             parameters=[{"markers_file": lc("markers"), "child_frame": "base_link", "debug_every_n": 2}]),
    ]
    if lc("rviz") == "true":
        nodes.append(Node(package="rviz2", executable="rviz2", output="log",
                          arguments=["-d", os.path.join(share, "config", "real.rviz")]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mock", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("markers", default_value=os.path.join(REPO, "config", "markers.yaml")),
        DeclareLaunchArgument("camera_info_file", default_value="auto",
                              description="calibration YAML; auto = config/tello_camera.yaml if it exists"),
        OpaqueFunction(function=setup),
    ])
