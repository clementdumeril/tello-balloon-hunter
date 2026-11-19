"""tello_driver: everything that flies the Tello.

    core/            runs on EVERY flight (started by launch/real.launch.py), whatever the mission
    missions/        flight programs (one ros2 run each), all built on missions/base.py
    identification/  measures the drone's response and fits the model used by the EKF and the missions
    sim/             fake drone + virtual room for tests without hardware (real.launch.py mock:=true)
"""
