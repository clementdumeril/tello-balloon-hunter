# First flight & hardware characterization (no ROS)

`first_flight.py` flies the Tello from the keyboard with live video and on-screen telemetry. It logs every state field to CSV, and it measures what the EKF needs: video latency (A5), telemetry behavior (A6) and hover noise (F1). It runs on Windows, macOS and Linux without ROS.

## Easiest way (Windows)

**Double-click `tello_tests.bat`** at the root of the repo. It goes to the right folder by itself and shows a menu:

| Choice | Test |
|---|---|
| 1 | Connection check (motors off). **Run it first after every drone power-on** |
| 2 | Video latency |
| 3 | Telemetry by hand, then manual flight |
| 4 | Automatic hover 30 s |
| 5 | Fake drone, to try the keys without hardware |

You can also run one test directly from any terminal: `tello_tests.bat 2`.

## Setup (once)

```bash
python -m venv ~/venvs/tello
~/venvs/tello/Scripts/python -m pip install -r scripts/requirements.txt   # Windows
# ~/venvs/tello/bin/python -m pip install -r scripts/requirements.txt     # macOS / Linux
```

Then connect the PC to the drone's **TELLO-xxxxxx** Wi-Fi (the PC loses internet while connected).

## Before every flight

- [ ] Prop guards on, battery > 25% (the script refuses takeoff below that)
- [ ] Room lit, **textured floor** under the drone (the Tello drifts over plain or shiny floors)
- [ ] Nobody within 1 m, a soft landing zone nearby (bed, sofa)
- [ ] The video window is **focused**: keys typed in other windows are ignored

| Key | Action |
|---|---|
| `T` / `G` | take off / land |
| Arrows | forward, back, left, right (released = stop) |
| `I` / `K` | up / down |
| `J` / `L` | yaw left / right |
| `Space` | hover |
| `Esc` | land and quit |
| `Backspace` | **EMERGENCY: motors off, the drone falls.** Only if something goes wrong. |

The keys are the same on AZERTY and QWERTY. Default speed is 30/100 (`--speed`).

## The four runs to do first

1. **Test without a drone:**
   `python scripts/first_flight.py --mock`
2. **Latency test (A5)**, drone on the table, motors off:
   `python scripts/first_flight.py --latency-test`
   Point the Tello camera at the flashing window from about 30 cm and wait 60 s. The median latency appears on screen and in the summary JSON. Repeat at 1 m, 3 m and 5 m from the PC to see the Wi-Fi effect.
3. **Telemetry by hand (A6)**, motors off: `python scripts/first_flight.py`
   Move the drone by hand along its forward axis, then sideways, then up, and watch `vgx`/`vgy`/`vgz`, `h`/`tof` and `yaw`. From the CSV, figure out the units, resolution and frame of each field (velocities may only update in flight, which is worth checking).
4. **Automatic hover (A6, F1):**
   `python scripts/first_flight.py --auto-hover 30`
   Takeoff, 30 s with zero commands, land. Gives ToF and velocity noise and the battery drain.

Results go to `logs/` (not committed): a `*_telemetry.csv` with every Tello state field (`s_*` columns), the commands sent and frame stats, plus a `*_summary.json` with latency, fps, frame drops, battery and flight time. **Copy the numbers into [docs/HARDWARE_AND_CONSTRAINTS.md](../docs/HARDWARE_AND_CONSTRAINTS.md#to-measure-in-week-1)** and attach the JSON to the issue.

## Troubleshooting

| Message | Cause | Fix |
|---|---|---|
| `This PC is not on the Tello Wi-Fi` | The PC is on another network (eduroam, home Wi-Fi) | Switch the drone on, wait for the LED to blink yellow, connect the PC to `TELLO-xxxxxx` |
| `No answer from the Tello` (on the Tello Wi-Fi) | Drone off or in a bad state | Restart the drone |
| same | The Tello phone app is connected to the drone | Close the app: only one controller at a time |
| `No answer from the Tello`, although it answered before | **The Tello only answers the first program (address + port) that sent `command` since it was switched on**, until it restarts | Restart the drone, then start `first_flight.py` first. Running another tool (phone app, another script) in between locks the drone onto it |
| same | Windows Firewall drops the drone's replies (the Tello network is a *Public* network) | In an **administrator** PowerShell: `New-NetFirewallRule -DisplayName "Tello UDP" -Direction Inbound -Protocol UDP -LocalPort 8889,8890,11111 -Action Allow -Profile Any` |

Quick check once on the Tello Wi-Fi: `ping 192.168.10.1` must answer.

**Seen on Nov 7:** after a first `command` from a random port, the Tello sent *every* later reply to that port (54753), whatever port the commands came from, and kept executing the commands (state and video kept flowing). `scripts/tello_udp_check.py` now detects this case and shows which port the replies go to. A real power cycle (hold the button until the LEDs go off) clears it.

**Heat:** the Tello heats up fast when it is switched on but not flying (86–89 °C measured after ~10 min on the ground). It shuts itself down near 90 °C. Switch it on just before testing, and switch it off between tests.

## Notes

- The latency measured with the flash includes the screen refresh (~10–30 ms) and the decoding time. It is the latency the rest of our pipeline really sees.
- The safety layer also auto-lands at 12% battery, and the OSD shows `VIDEO STALLED` if no frame arrives for more than 1 s.
- If the colors look wrong (blue skin), add `--swap-rb`.
