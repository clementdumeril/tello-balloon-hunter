"""Runs on every flight.

    driver.py            the only node that talks to the Tello (Wi-Fi): video, telemetry, takeoff/land, rc commands;
                         every command goes through safety.py first; camera/link failsafe (no video -> hover/land)
    safety.py            the guard chain applied to every command (position hold, ceiling, height band, zone, walls)
    zone.py              the obstacle-free flight zone (room scan grid) and the clearance queries
    home.py              the takeoff pad position (config/home.yaml), for the return before landing
    ekf_core.py          the 10-state EKF (pure numpy, no ROS: also used offline by tools/ekf_replay.py)
    ekf_node.py          its ROS wrapper: /aruco/pose + /tello/rc_sent in, /ekf/pose out
    model.py             the identified dynamics (gains, time constants, delays) shared by the EKF, missions and sim
    video_reader.py      optional low-latency video decoder (driver low_latency_video:=true)
    ros_env.py           local-only DDS settings (loopback, small packets) for every node
    session_recorder.py  records each flight (video, poses, commands) in ~/tello_ws/sessions
    teleop.py            keyboard flight; its read_key() is also the "any key = land" of the missions
"""
