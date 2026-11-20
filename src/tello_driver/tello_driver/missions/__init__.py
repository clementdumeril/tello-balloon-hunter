"""Flight programs. Each is a Mission subclass started with base.run(cls).

    base.py        Mission: inputs, takeoff/start point, 20 Hz loop with safety checks, search, return home, logs
    tour.py        goto(): fly in front of a marker; tello_marker_tour (all markers), tello_goto 49 35 (a list)
    patrol.py      laps of the flight zone, type a marker id to go to it
    strike.py      fast model-based trajectories (quintic + feedforward) to a target: the base of the balloon strike
    hunt.py        the demo: patrol, the external camera sees the red balloon, dash through it, come back
    trajectory.py  the quintic trajectory used by strike.py and hunt.py
"""
