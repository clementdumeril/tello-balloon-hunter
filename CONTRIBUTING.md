# How we work

## The loop

1. **Find a task.** Open [STATUS.md](STATUS.md) and look under *Up for grabs*, or filter the [issues](../../issues) by your workstream label (`ws:A` … `ws:I`). Prefer issues labeled `critical-path`.
2. **Claim it.** Assign yourself on the issue. Don't take more than 2 open tasks at once. If the task is blocked, add the `blocked` label and comment why.
3. **Branch.** Name it `<task-id>-short-name`, for example `C3-aruco-localizer`.
4. **Work.** Commit small and often. Put measurements, plots and decisions in the issue comments, so the report writes itself.
5. **Pull request.** Use the template, write `Closes #<issue>` in the description, and add a short demo (a plot, a GIF, a rosbag name).
6. **Review.** Get one teammate's approval, then merge into `main`. The issue closes automatically and [STATUS.md](STATUS.md) updates itself.

## Rules

- `main` must always build (`colcon build`) and launch.
- **A task is done when its "Definition of done" checklist is fully ticked**, not when code exists.
- Hardware tests: at least two people present, prop guards on, emergency-land key ready.
- **No large files in git**: rosbags, datasets, model weights and videos go in the team drive. Commit only links and small configs (calibrations, `markers.yaml`, EKF parameters).
- Take photos and videos at every stage, for the check-ins and the website.

## Labels

| Label | Meaning |
|---|---|
| `ws:A` … `ws:I` | Workstream (see [docs/PLAN.md](docs/PLAN.md)) |
| `critical-path` | Delays the demo if late |
| `stretch` | Only after the levels 1 and 2 scope is solid |
| `hardware` | Needs the drone, cameras or purchases |
| `blocked` | Waiting on something; explain in a comment |

## Weekly sync (15 min)

1. Each person: done / next / blocked (look at STATUS.md together).
2. Check the critical path and the next deadline.
3. Book flight-test slots for the week.

## Environment

- Ubuntu 24.04 + **ROS 2 Jazzy** + **Gazebo Harmonic** on both machines (native, or WSL2 on Windows)
- Simulation: big PC (`ros2 launch tello_sim sim.launch.py`, see `src/tello_sim/README.md`) · real drone: the laptop
- Code moves between machines only through Git; real flight data moves as rosbags
- Python ≥ 3.10, OpenCV with `aruco` (`opencv-contrib-python`), `djitellopy`, `ultralytics`, `numpy`, `scipy`
