# Local-only ROS 2 for the laptop (WSL): source it in ~/.bashrc so command-line tools (ros2 topic ...)
# see the same nodes as real.launch.py, also when Windows is on the Tello Wi-Fi.
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
# Fast DDS on the loopback only, packets <= 1400 bytes (WSL mirrored networking drops fragmented UDP on
# the loopback; with Tailscale up, discovery messages grew past 1500 bytes and the nodes could not see each other)
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/tello_ws/install/tello_driver/share/tello_driver/config/fastdds_local.xml"
