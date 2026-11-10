# Local-only ROS 2 for the laptop (WSL): source it in ~/.bashrc so command-line tools (ros2 topic ...)
# see the same nodes as real.launch.py, also when Windows is on the Tello Wi-Fi.
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
