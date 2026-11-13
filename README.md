# Tello Balloon Hunter

**EECS C106A / 206A, Fall 2026: final project.**
A DJI Tello localizes itself in a room covered with ArUco markers (our own EKF), flies to targets on its own and,
in the end, finds and pops balloons.

**How it works: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** (nodes, topics, frames, the safety chain, the EKF,
the missions, every file) and **[docs/ARUCO.md](docs/ARUCO.md)** (how a camera image becomes the drone position).

## Fly

Everything runs on one laptop: Windows connects to the drone's Wi-Fi, ROS 2 Jazzy runs in WSL2 (Ubuntu 24.04).

**Quickest: double-click `tello.bat`** (drone on first): it connects the USB Wi-Fi dongle to the Tello, starts
the launch below with RViz, and opens a second terminal ready for the commands. `tello.bat mock` = fake drone.

By hand:

```bash
# terminal 1 (WSL): driver + localizer + EKF + RViz. Band mode (1.70-2.00 m) is the default.
ros2 launch tello_driver real.launch.py
# add mock:=true for the fake drone (no hardware), rviz:=false to save CPU during missions

# terminal 2: fly by hand...
ros2 run tello_driver tello_teleop          # t take off, arrows, i/k, j/l, g land, x emergency
# ...or a mission: go in front of marker 49, then 35 (ENTER takes off, any key lands)
ros2 run tello_driver tello_goto 49 35

# after a flight
python3 tools/flight_report.py --band 1.7 2.0
```

The drone takes off (the Tello stops at ~0.8 m), the driver climbs to 1.85 m where the markers are in view, and
from then on every command goes through the safety chain (position hold, ceiling, height band, obstacles of the room
scan, wall ahead).

## Repository

```
src/tello_driver/        ROS 2 package: driver, safety chain, EKF, missions, teleop, fake drone
src/aruco_localization/  ROS 2 package: marker map, ArUco localizer
config/                  markers.yaml (marker map), tello_camera.yaml, flight zones from the room scan
tools/                   offline tools: marker map, calibration, room scan, flight report, takeoff pad
scripts/wsl/             ROS 2 setup in WSL
docs/                    ARCHITECTURE.md, printable markers
```

Install (once): `bash scripts/wsl/setup_ros_jazzy.sh`, then
`cd ~/tello_ws && colcon build --symlink-install && source install/setup.bash`.

## Key dates

| Date | Deliverable |
|---|---|
| **Dec 10–11** | Showcase: demo + 10-minute presentation |
| **Dec 18, 11:59 pm** | Report website + peer evaluations |
