"""Go in front of markers, one after the other, back to the room centre between them.

    ros2 launch tello_driver real.launch.py band:=true     (terminal 1, drone on the floor anywhere in the room)
    ros2 run tello_driver tello_goto 49 35                  (terminal 2, NO teleop running; you stay next to it)

Takeoff, climb to the band, find the markers (sweep, then a turn on the spot if none is in view), fly to the room
centre, then for each marker: turn to face it, fly in front of it (~0.9 m, braking profile), stop 1 s, back to
the centre. Then back above the takeoff pad (config/home.yaml) and landing.

SAFETY: ANY KEY = stop and land. Ctrl+C = land. Automatic landing if the drone gets too close to an obstacle of the
room scan, if no marker is seen for 12 s, or on low battery. Options (--ros-args -p ...): max_speed (0.5 m/s),
approach (0.9 m), hold (1 s in front of each marker), band (true).
"""
import os
import sys

from tello_driver.marker_tour import MarkerTour
from tello_driver.step_test import run


class GotoMarkers(MarkerTour):
    NODE_NAME = "tello_goto"
    PREFIX = "goto"
    BAND_DEFAULT = True

    def __init__(self):
        super().__init__()
        if not self.sequence:
            raise SystemExit("usage: ros2 run tello_driver tello_goto 49 35   (marker ids, in order)")
        self.survey = False
        self.fence_tol = 1.0  # takeoff anywhere: the obstacle grid (safety) is the real limit, not the rectangle


def main():
    ids = [a for a in sys.argv[1:] if a.lstrip("-").isdigit()]
    if ids:
        os.environ["GOTO_SEQ"] = ",".join(ids)
    run(GotoMarkers)


if __name__ == "__main__":
    main()
