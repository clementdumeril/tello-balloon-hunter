"""Simulated room + full localization chain.

    ros2 launch tello_sim sim.launch.py                       # headless, low-res cameras, test flight + evaluation
    ros2 launch tello_sim sim.launch.py gui:=true gpu:=true   # on the big PC's screen, full-res cameras, RViz

Same topics as the real drone: /tello/image_raw, /tello/camera_info, /tello/telemetry, /tello/cmd_vel_safe.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO = os.environ.get("TELLO_REPO", str(Path.home() / "tello_ws/src/tello-balloon-hunter"))


def gazebo_env(headless):
    """Sourcing ROS Jazzy puts its own gz-tools/vendor libraries first, which hides `gz sim`.
    Run the system Gazebo Harmonic with the system paths instead."""
    env = dict(os.environ)
    env["GZ_CONFIG_PATH"] = "/usr/share/gz"
    env["LD_LIBRARY_PATH"] = ":".join(p for p in env.get("LD_LIBRARY_PATH", "").split(":")
                                      if p and not p.startswith("/opt/ros"))
    if headless:  # over SSH the WSLg display does not answer; force EGL headless rendering
        env.pop("DISPLAY", None)
        env.pop("WAYLAND_DISPLAY", None)
    return env


def setup(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    gui, gpu = lc("gui") == "true", lc("gpu") == "true"
    markers, latency = lc("markers"), float(lc("latency"))
    out = Path(lc("out_dir"))
    world_dir = out / "world"
    share = get_package_share_directory("tello_sim")

    res = [] if gpu else ["--tello-width", "480", "--tello-height", "360", "--tello-rate", "10",
                          "--logi-width", "480", "--logi-height", "270", "--logi-rate", "5"]
    gen = ExecuteProcess(cmd=["python3", "-m", "tello_sim.generate_world", "--markers", markers,
                              "--out", str(world_dir), *res], output="screen")
    gz_cmd = ["/usr/bin/gz", "sim", "-r", str(world_dir / "room.sdf")]
    if not gui:
        gz_cmd.insert(2, "-s")
    gz = ExecuteProcess(cmd=gz_cmd, output="log", env=gazebo_env(headless=not gui))

    sim_time = {"use_sim_time": True}
    nodes = [
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge", output="log",
             parameters=[{"config_file": os.path.join(share, "config", "bridge.yaml")}, sim_time]),
        Node(package="tello_sim", executable="sim_adapter", output="screen",
             parameters=[{"latency": latency}, sim_time]),
        Node(package="aruco_localization", executable="marker_map_publisher", output="screen",
             parameters=[{"markers_file": markers}, sim_time]),
        Node(package="aruco_localization", executable="aruco_localizer", output="screen",
             parameters=[{"markers_file": markers, "debug_dir": str(out / "debug")}, sim_time]),
        Node(package="tello_sim", executable="eval_localization", output="screen",
             parameters=[{"latency": latency, "out_dir": str(out)}, sim_time]),
    ]
    if lc("rviz") == "true":
        nodes.append(Node(package="rviz2", executable="rviz2", output="log",
                          arguments=["-d", os.path.join(share, "config", "sim.rviz")], parameters=[sim_time]))
    actions = [gen, RegisterEventHandler(OnProcessExit(target_action=gen, on_exit=[gz, *nodes]))]
    if lc("test_flight") == "true":
        flight = Node(package="tello_sim", executable="test_flight", output="screen", parameters=[sim_time])
        actions.append(RegisterEventHandler(OnProcessExit(target_action=gen, on_exit=[flight])))
        actions.append(RegisterEventHandler(OnProcessExit(target_action=flight,
                                                          on_exit=[EmitEvent(event=Shutdown(reason="test flight done"))])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("PYTHONNOUSERSITE", "1"),  # avoid user-site numpy 2 breaking system OpenCV
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("gpu", default_value="false", description="full-res cameras (needs GPU rendering)"),
        DeclareLaunchArgument("rviz", default_value=LaunchConfiguration("gui")),
        DeclareLaunchArgument("test_flight", default_value="true"),
        DeclareLaunchArgument("latency", default_value="0.20"),
        DeclareLaunchArgument("markers", default_value=os.path.join(REPO, "config", "markers.yaml")),
        DeclareLaunchArgument("out_dir", default_value=str(Path.home() / "tello_ws/sim_out")),
        OpaqueFunction(function=setup),
    ])
