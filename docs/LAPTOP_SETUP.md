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
3. Ubuntu terminal 2, the **teleop window** (recommended: the drone moves while a key is held, stops on release):
   ```bash
   ros2 run tello_driver tello_teleop_gui
   ```
   Click the window first (if it loses focus, the drone stops). `t` take off · `g` land · arrows move ·
   `i`/`k` up/down · `j`/`l` turn · `1`/`2`/`3` speed 0.25/0.5/0.75 m/s · space stop · `x` EMERGENCY · `q` land and quit.
   Terminal-only version (each press moves 0.35 s): `ros2 run tello_driver tello_teleop`.
4. In RViz: the drone (grey body, red front rotors, cyan camera field of view) moves among the markers whenever
   the camera sees at least one known marker; green lines = markers used right now, orange = trajectory.
   No green line = no marker in view: the drone stays at its last known pose (hold it 1.5-2.5 m from the wall,
   1.3-1.8 m high, so that 2-3 markers are in the picture).

**Room scan in RViz:** checkbox *Room (3D scan)* in the Displays panel (untick it if RViz gets slow). The launch
loads `~/tello_ws/room_scan/room_map.obj`, made from a Scaniverse OBJ + its marker alignment with
`python tools/room_mesh_to_map.py <scan.obj> <alignment.npz> <out_dir>` (ceiling removed so RViz sees inside).

**Height safety (driver):** after takeoff the drone climbs to about 1.8 m (where the markers are visible), measured with
its distance sensor and the ArUco pose. Ceiling guard for every program: `max_height` 2.6 m (launch argument), it
brakes ahead of the limit and descends if above it.

## Strike mission (planned trajectories: the balloon attack rehearsed on markers)

```bash
ros2 run tello_driver tello_strike                                  # patrol legs, then strikes on markers 6, 10, 5
ros2 run tello_driver tello_strike --ros-args -p mode:=strike -p targets:=10,5 -p v_max:=0.6
```

Minimum-jerk trajectories planned with v_max / a_max, the command limit and checked against the free space of the
room scan before flying; tracked with the identified model as feedforward, u = (v_d + tau a_d) / K taken
L_u = 0.3 s ahead, plus PI feedback on the EKF. Simulation (fake drone with the real dynamics, drift and 0.5 s
delay): patrol legs of 1.6 m in 3.8 s with 4-6 cm rms tracking error; 0.7 m strike dashes in ~2 s, no
overshoot. Results in `~/tello_ws/sessions/strike_<time>/results.csv` and `tracking.csv`.

Fast identification (larger, shorter pulses, in the free space):
`ros2 run tello_driver tello_step_test --ros-args -p amplitude:=0.6 -p duration:=0.8 -p axes:=y,x`

## Marker tour (first autonomous mission)

Same start as the identification flight (drone on the floor, 2 m from the marker wall, facing it, no teleop):

```bash
ros2 run tello_driver tello_marker_tour
```

ENTER takes off; the drone climbs, then visits every marker (stops in front of it, facing it, 2 s), changing
walls from the room centre, comes back to its start point and lands. About 2 min. Flight zone: 0.6 m from the
marker walls and inside the room scan, heights 1.2-2.2 m, at most 0.3 m/s. **Any key = stop and land.**
Checked on the fake drone (12/12 markers, 119 s). Recorded in `~/tello_ws/sessions/tour_<time>/`.

## Identification flight (the drone's model, for the position controller)

Drone facing the marker wall at about 2 m, 2-3 markers in the picture, nothing within 1 m, battery > 30 %.
**No teleop running** (the script refuses to start otherwise). Terminal 1: the launch as above. Terminal 2:

```bash
ros2 run tello_driver tello_step_test
```

ENTER takes off. About 1 min 30: on each axis (forward, left, up, turn) two doublets (+0.3 m/s for 1.2 s then
-0.3 m/s, so the drone comes back), with a re-centring on the start point in between. **Any key = stop and land.**
Automatic landing if the drone leaves a +-0.8 m box, if no marker is seen for 3 s, or on low battery.
It lands, then fits per axis $\tau \dot v = K u(t-L) - v$ and prints $K$ (real/commanded speed), $\tau$ and $L$.
Results in `~/tello_ws/sessions/step_<time>/` (`model.yaml`, `step_fit.png`). Re-fit a folder:
`ros2 run tello_driver tello_step_fit <folder>`. Checked on the fake drone (built-in $\tau$ = 0.8 s, $L$ = 0.1 s):
fit $K$ 0.97-1.09, $\tau$ 0.75-0.90 s, $L$ 0.10-0.16 s (0.1 s + camera/localizer).

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
