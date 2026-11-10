# Laptop setup: ROS 2 Jazzy in WSL2 for the real drone

Goal: fly the real Tello from the laptop **with ROS 2 and live RViz** (ArUco markers, drone pose, camera view).
Ubuntu lives in `C:\Users\<you>\AppData\Local` and the ROS workspace in the Linux disk (`~/tello_ws`), **never in OneDrive**.

Time: about 1 h, mostly waiting for downloads.

## 1. Windows side (administrator, once)

Open **PowerShell as administrator** (Start → type *PowerShell* → right click → *Run as administrator*):

```powershell
wsl --install -d Ubuntu-24.04
```

**Restart the PC.** Ubuntu opens by itself after the restart: choose a Linux user name and password, and remember the password, because `sudo` asks for it.

Still in an **administrator** PowerShell, let the drone's packets reach Ubuntu:

```powershell
# WSL shares the PC's network (needed: the Tello sends its video and state to the PC's address)
Set-Content -Path "$env:USERPROFILE\.wslconfig" -Value "[wsl2]`nnetworkingMode=mirrored"
# allow incoming packets (Tello video/state) into WSL
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
wsl --shutdown
```

## 2. Ubuntu side (once)

Open **Ubuntu** from the Start menu, then:

```bash
bash <(curl -fsSL https://raw.githubusercontent.com/clementdumeril/tello-balloon-hunter/main/scripts/wsl/setup_ros_jazzy.sh)
```

The script installs ROS 2 Jazzy, RViz, the camera calibration tool, djitellopy, then clones and builds the project in `~/tello_ws`. It ends with `OK: OpenCV … djitellopy, PyAV …`. Close Ubuntu and open it again.

Test without the drone:

```bash
ros2 launch tello_driver real.launch.py mock:=true
```

RViz opens on the room (the markers of `config/markers.yaml`, numbered) with a **virtual drone**: a fake camera
renders the real marker map, the localizer computes the pose from those images, and the drone (grey body, cyan
camera field of view) appears 2 m in front of the main wall. In a second terminal, `ros2 run tello_driver tello_teleop`
then `t` (take off) and the arrows fly it: the orange trail is its trajectory, the green lines go to the markers it
currently sees. Same chain as the real drone, no hardware needed.

## 3. Every flight session

1. Switch the drone on and connect **Windows** to `TELLO-xxxxxx` (Ubuntu follows automatically).
2. Ubuntu terminal 1, **the first program to talk to the drone**:
   ```bash
   ros2 launch tello_driver real.launch.py
   ```
3. Ubuntu terminal 2:
   ```bash
   ros2 run tello_driver tello_teleop
   ```
   `t` take off · `g` land · arrows move · `i`/`k` up/down · `j`/`l` yaw · space stop · `x` EMERGENCY · `q` land and quit
4. In RViz: the drone (grey body, red front rotors, cyan camera field of view) moves among the markers whenever
   the camera sees at least one known marker; green lines = markers used right now, orange = trajectory.
   No green line = no marker in view: the drone stays at its last known pose (hold it 1.5-2.5 m from the wall,
   1.3-1.8 m high, so that 2-3 markers are in the picture).

## 4. Camera calibration (once per drone, before trusting positions)

Print `docs/print/checkerboard_A4.pdf` at 100 % and glue it flat on cardboard (9×6 inner corners, 25 mm squares). With the driver running, drone on the table:

```bash
ros2 run camera_calibration cameracalibrator --size 9x6 --square 0.025 --no-service-check \
  --ros-args -r image:=/tello/image_raw -p camera:=/tello
```

Move the board in front of the camera (near, far, tilted, in every corner) until X, Y, Size and Skew are green, then **CALIBRATE**, then **SAVE**. Copy the result into the repo:

```bash
cd /tmp && tar xzf calibrationdata.tar.gz ost.yaml && cp ost.yaml ~/tello_ws/src/tello-balloon-hunter/config/tello_camera.yaml
```

`real.launch.py` picks up `config/tello_camera.yaml` automatically.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `No answer from the Tello` | The drone answers only the first program since power-on: switch it OFF (LEDs off) and on, then start `real.launch.py` first |
| Driver connects but no video / no state | The mirrored mode or Hyper-V firewall step was skipped: redo step 1, then `wsl --shutdown` |
| RViz window does not open | Update WSL: `wsl --update` (Windows PowerShell), then `wsl --shutdown` |
| OpenCV import error mentioning numpy | A pip numpy 2 ended up in `~/.local`: `python3 -m pip uninstall --break-system-packages numpy` |
