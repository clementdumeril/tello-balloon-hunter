# Hardware & Real-World Constraints

The design choices in this project are driven by what the hardware can really do. **Values marked _TBM_ (to be measured) must be filled in during week 1** (tasks A5, A6). The EKF and controller parameters depend on them.

## Course constraints

| Constraint | Consequence for us |
|---|---|
| Must be a real robotic task on real hardware; simulation is not allowed as the result unless approved | Simulation is only a development tool. Every claim in the report is backed by real flights. |
| Model-based course, not focused on learning; off-the-shelf chaining is not valued | Our own localizer, EKF, controller and state machine. YOLO is one component. |
| Drone projects must state where the drone is legally flown | Flights happen in a private off-campus room. **Demo logistics must be confirmed with the GSI (B8).** |
| $55 purchase budget, request form, **deadline Nov 20, 12 pm** | Shopping list in A8 |
| Group of 4–5; graduate expectations if any member is in 206A | Higher bar for evaluation and polish |
| Peer evaluations | Every member owns visible issues and PRs |

## DJI Tello (standard, not EDU)

| Property | Value | Impact |
|---|---|---|
| Video stream | 960×720, ~30 fps, H.264 over Wi-Fi | Compression artifacts, frame drops |
| Camera | Fixed, forward-facing, ~82.6° diagonal FOV (≈ 680 px focal at 960×720) | Markers only on walls; no ceiling or floor markers |
| Video latency | _TBM_ (typically 100–300 ms) | Delayed EKF updates (E4) |
| Link | Tello is a 2.4 GHz access point; the laptop joins it | Laptop loses internet → use a USB Wi-Fi dongle or ethernet |
| SDK | UDP: commands on 8889, state on 8890, video on 11111. `rc a b c d` velocity commands | Control rate ~10–20 Hz |
| Telemetry | vgx/vgy/vgz, ToF height, baro, pitch/roll/yaw, accelerations, battery | Units, resolution, frame and rate are _TBM_ (A6) |
| Stabilization | Downward vision positioning system (optical flow) | Needs a **textured floor** and good light |
| Battery | ~13 min spec, **~8–10 min usable** | Spare batteries, charging schedule |
| Payload | Very limited | Pin mount kept to a few grams (H1) |
| No mission pads | Standard Tello | No built-in ground truth → our own evaluation protocol (I2) |

## Logitech C922 (fixed camera)

| Property | Value |
|---|---|
| Modes | 1080p @ 30 fps or 720p @ 60 fps |
| FOV | ~78° diagonal (≈ 900 px focal at 1280×720) |
| Placement | High in a room corner, tilted down, covering the whole flight zone |
| Role | Marker map building (B6), balloon localization for the search phase (G6), optional drone tracking (C5) |

## Pixel budget (why some ideas work and others don't)

| Object | Camera | Distance | Apparent size | Verdict |
|---|---|---|---|---|
| 15 cm ArUco | Tello | 3 m | ~34 px | OK for detection; pose accuracy drops with distance |
| 15 cm ArUco | Tello | 1 m | ~100 px | Very good |
| 28 cm balloon | Tello | 2 m | ~95 px | Easy to detect |
| 28 cm balloon | C922 (720p) | 4 m | ~63 px | Easy to detect |
| Tello (~10 cm) | C922 (720p) | 4 m | **~23 px** | **Hard for YOLO.** Small, blurred, thin props, so drone detection by YOLO is not in the plan (only background subtraction as a stretch, C5). |

## Flight room

| Aspect | Constraint | Decision |
|---|---|---|
| Small room | Drone 0.5–3 m from the walls. At 0.5 m from a wall, the camera sees only ~70 cm of it | **Dense markers: spacing ≤ 60 cm**, mostly 15 cm with a few 20 cm. Fly mainly in the center of the room. |
| High ceiling | Helps mount the C922 high | Mount it high in a corner, tilted down |
| Furniture (bed…) | Less free volume, ToF jumps over furniture | Define a flight box; reject ToF outliers; the bed is the emergency landing zone |
| Floor | Optical flow needs texture | Put a textured mat or rug in the flight box |
| Windows | Backlight hurts marker detection | Curtains closed, tests at several times of day |
| Prop wash | Turbulence near walls; pushes the balloon away | Tether the balloon at 1–1.5 m; fast final approach |
| Wi-Fi | Crowded 2.4 GHz band in residential buildings | Laptop close to the flight box; log drops (A5) |
| Demo | The showcase is in the lab, not this room | Portable marker boards and a reproducible layout (B8) |

## To measure in week 1

| Quantity | How | Result | Task |
|---|---|---|---|
| Video latency (mean, std) | Tello films a stopwatch on the laptop screen; compare with the clock in the frame | _TBM_ | A5 |
| Frame drop rate | Count missing frames over 5 min at different distances | _TBM_ | A5 |
| Telemetry rate | Timestamps of state packets | _TBM_ | A6 |
| Velocity units, resolution, frame | Move the drone by hand along known axes; check the reported values | _TBM_ | A6 |
| ToF noise | Std at rest at 3 heights | _TBM_ | A6 |
| Yaw reference and drift | Yaw at power-on vs. map; drift over 5 min | _TBM_ | A6 |
| Hover drift without correction | Hover 60 s, measure displacement | _TBM_ | F1 |
| Step response to `rc` | Step inputs per axis → k, τ | _TBM_ | F1 |
| Usable flight time | Full battery to auto-land | _TBM_ | A6 |
