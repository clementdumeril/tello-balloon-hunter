#!/usr/bin/env bash
# One-shot setup of the laptop's Ubuntu 24.04 (WSL2) for the REAL drone: ROS 2 Jazzy, RViz, camera
# calibration, djitellopy, and the project workspace in the Linux disk (~/tello_ws, NOT in OneDrive).
#
#   bash setup_ros_jazzy.sh          (asks for your Ubuntu password once, for sudo)
#
# Safe to run again: every step checks what is already there.
set -euo pipefail

REPO_URL="https://github.com/clementdumeril/tello-balloon-hunter.git"
WS="$HOME/tello_ws"

say() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }

. /etc/os-release
if [ "${VERSION_CODENAME:-}" != "noble" ]; then
  echo "This script expects Ubuntu 24.04 (noble); found ${PRETTY_NAME:-unknown}." >&2
  exit 1
fi

say "1/6 ROS 2 apt repository"
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y software-properties-common curl
  sudo add-apt-repository -y universe
  ver=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  curl -fL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.${VERSION_CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
fi

say "2/6 ROS 2 Jazzy desktop + tools (the long step: 10-20 min)"
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-desktop ros-dev-tools \
  ros-jazzy-cv-bridge ros-jazzy-camera-calibration ros-jazzy-camera-calibration-parsers \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-usb-cam \
  python3-opencv python3-pil python3-yaml python3-pip git

say "3/6 djitellopy + PyAV (no extra numpy: keeps the system OpenCV working)"
# --no-deps on purpose: a pip numpy 2 in ~/.local breaks the system OpenCV / cv_bridge.
python3 -m pip install --user --break-system-packages --no-deps --upgrade djitellopy av

say "4/6 Project workspace in the Linux disk: $WS"
mkdir -p "$WS/src"
if [ ! -d "$WS/src/tello-balloon-hunter/.git" ]; then
  git clone "$REPO_URL" "$WS/src/tello-balloon-hunter"
else
  git -C "$WS/src/tello-balloon-hunter" pull --ff-only
fi

say "5/6 Build"
# ROS setup files use unset variables: relax `set -u` while sourcing them
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
cd "$WS"
colcon build --symlink-install --packages-select aruco_localization tello_driver

say "6/6 Shell setup"
grep -q "tello_ws/install/setup.bash" ~/.bashrc || cat >> ~/.bashrc <<'EOF'

# --- tello-balloon-hunter ---
source /opt/ros/jazzy/setup.bash
[ -f ~/tello_ws/install/setup.bash ] && source ~/tello_ws/install/setup.bash
EOF

python3 - <<'EOF'
import cv2, numpy, djitellopy, av
print(f"OK: OpenCV {cv2.__version__} (aruco: {hasattr(cv2, 'aruco')}), numpy {numpy.__version__}, djitellopy, PyAV {av.__version__}")
EOF
echo
echo "Done. Open a NEW Ubuntu terminal, then:  ros2 launch tello_driver real.launch.py"
