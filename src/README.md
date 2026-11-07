# ROS 2 packages (planned)

Created by task A1. See [docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md#ros-2-nodes-and-topics) for nodes and topics.

| Package | Content | Workstream |
|---|---|---|
| `tello_driver` | Tello driver (djitellopy), safety layer, teleop | A |
| `aruco_localization` | Marker map publisher, ArUco localizer, marker mapper, fixed camera extrinsics | B, C |
| `estimation` | Drone EKF, balloon EKF, pure-Python EKF library + tests | E |
| `perception` | Balloon detector (YOLO / HSV backends) | G |
| `control` | Position controller, visual servoing | F, H |
| `mission` | Mission state machine | H |
| `tello_sim` | Gazebo Harmonic room from `config/markers.yaml`, Tello-like adapter, test flight, evaluation ([README](tello_sim/README.md)) | D |
