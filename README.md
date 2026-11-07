# Tello Balloon Hunter

**EECS C106A / 206A, Fall 2026: Final Project**

A DJI Tello localizes itself in a room covered with ArUco markers (TF tree + our own EKF), finds a balloon with its camera and a fixed Logitech C922, flies to it and pops it.

> **New to the repo? Start here:**
> 1. Look at **[STATUS.md](STATUS.md)** to see what is done, what is in progress and what is free to take.
> 2. Pick an open, unassigned issue in **[Issues](../../issues)** and assign yourself.
> 3. Read **[CONTRIBUTING.md](CONTRIBUTING.md)** for the workflow (branch, PR, `Closes #N`).

<!-- STATUS:START -->
### 📊 Progress: 1/55 tasks (2%) · 0 in progress · 9 up for grabs

`█░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░`

| Milestone | Due | Progress |
|---|---|---|
| [M1: Infrastructure](https://github.com/clementdumeril/tello-balloon-hunter/milestone/1)  |  2026-11-15  |  `█░░░░░░░░░░░░░░░░░░░` 6% |
| [M2: TF chain validated (hand-held)](https://github.com/clementdumeril/tello-balloon-hunter/milestone/2)  |  2026-11-22  |  `░░░░░░░░░░░░░░░░░░░░` 0% |
| [M3: Estimation & autonomous flight](https://github.com/clementdumeril/tello-balloon-hunter/milestone/3)  |  2026-11-29  |  `░░░░░░░░░░░░░░░░░░░░` 0% |
| [M4: Balloon perception](https://github.com/clementdumeril/tello-balloon-hunter/milestone/4)  |  2026-12-02  |  `░░░░░░░░░░░░░░░░░░░░` 0% |
| [M5: Interception (feature freeze)](https://github.com/clementdumeril/tello-balloon-hunter/milestone/5)  |  2026-12-06  |  `░░░░░░░░░░░░░░░░░░░░` 0% |
| [M6: Demo & report](https://github.com/clementdumeril/tello-balloon-hunter/milestone/6)  |  2026-12-18  |  `░░░░░░░░░░░░░░░░░░░░` 0% |

➡️ **Details, owners and free tasks: [STATUS.md](STATUS.md)**
<!-- STATUS:END -->

---

## The project in one picture

```mermaid
flowchart LR
    subgraph Room["Flight room: ArUco markers on every wall"]
        T["DJI Tello<br/>forward camera + telemetry"]
        L["Logitech C922<br/>fixed, mounted high"]
        B(("Balloon<br/>tethered"))
    end

    T -- "Wi-Fi video + state" --> LOC["ArUco localizer<br/>(joint solvePnP)"]
    T -- "velocity, ToF, yaw" --> EKF["Drone EKF"]
    LOC --> EKF
    EKF -- "TF map → base_link" --> CTRL["Position controller"]
    T -- "video" --> DET["Balloon detector<br/>(YOLO / HSV)"]
    L -- "video" --> DETL["Balloon detector<br/>(fixed cam)"]
    DET --> BEKF["Balloon EKF"]
    DETL --> BEKF
    BEKF -- "TF map → balloon" --> MIS["Mission state machine<br/>search → approach → strike"]
    MIS --> CTRL
    CTRL -- "rc commands" --> T
```

## Sensing, planning and actuation (course requirement)

| Requirement | How we meet it |
|---|---|
| **Sensing** | Tello forward camera, Tello telemetry (optical-flow velocity, ToF height, IMU yaw), fixed Logitech C922 |
| **Planning** | Mission state machine, waypoint navigation, approach planning toward the balloon frame |
| **Actuation** | Closed-loop position control and visual servoing of the Tello through `rc` velocity commands |
| **Hardware** | Real Tello flying in a real room. Simulation is a development tool only, never the final result. |

## Key dates

| Date | Deliverable |
|---|---|
| Week 12 (Nov 9–13) | Check-In 1 with a GSI (short presentation) |
| **Nov 20, 12 pm** | **Material purchase deadline ($55 budget)** |
| Week 14 | Check-In 2 with a GSI |
| **Dec 10–11** | **Showcase: demo + 10-minute presentation** |
| **Dec 18, 11:59 pm** | **Report website + peer evaluations** |

No extensions on any project deadline. Full schedule: [docs/PLAN.md](docs/PLAN.md#timeline).

## Documentation

| Document | What's inside |
|---|---|
| [ROS graph + simulated flight](https://claude.ai/artifact/L9Mz7RZQoL94jwAVUmNYeQ) | Interactive ROS 2 graph (nodes, topics, TF), code of every node, video of the simulated test flight (ask Clément for access) |
| [STATUS.md](STATUS.md) | Live progress per milestone and workstream, who is doing what (auto-generated) |
| [docs/PLAN.md](docs/PLAN.md) | Full work plan: workstreams, every task with its definition of done, timeline, critical path, roles, risks |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | TF tree, ROS 2 nodes and topics, EKF design (state, models), controller, mission state machine |
| [docs/HARDWARE_AND_CONSTRAINTS.md](docs/HARDWARE_AND_CONSTRAINTS.md) | Real hardware limits (Tello, C922), room constraints, what we must measure in week 1 |
| [docs/LAPTOP_SETUP.md](docs/LAPTOP_SETUP.md) | Laptop: ROS 2 Jazzy in WSL2 for the real drone, live RViz, camera calibration |
| [CONTRIBUTING.md](CONTRIBUTING.md) | How we work: issues, branches, PRs, data, weekly sync |
| [scripts/](scripts/README.md) | **First flight tool** (no ROS): keyboard flight, latency test, telemetry logging |

## Team

| Name | Role | GitHub |
|---|---|---|
| _TBD_ | Drone & control | |
| _TBD_ | Localization & mapping | |
| _TBD_ | State estimation | |
| _TBD_ | Perception | |
| _TBD_ | Mission, integration & report | |

## Repository layout (planned)

```
src/                 ROS 2 packages (see src/README.md)
docs/                plan, architecture, hardware notes
config/              camera calibrations, markers.yaml, EKF parameters
scripts/             first_flight.py: keyboard flight + latency/telemetry tests (no ROS)
tools/               status generator
```
