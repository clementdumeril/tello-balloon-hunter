"""Identification flight: velocity doublets of several amplitudes and durations on each axis, then fitted.

    ros2 launch tello_driver real.launch.py                                  (terminal 1)
    ros2 run tello_driver tello_step_test                                    identification flight (~3.5 min)
    ros2 run tello_driver tello_step_test --ros-args -p schedule:=valid      validation flight (~2 min)

Each doublet: hover 2 s, +A for T, -A for T (the drone comes back), hover 3 s; the tested axis is open loop, the
others hold the position; the drone re-centres (middle of the flight zone and of the height band) before each one.
Several amplitudes AND durations (Nov 13: 2 identical doublets could not separate the delay L from the time
constant tau), both signs. Up/down: short doublets (A*T <= 12 cm, the height-band guard must not cut them) but strong
(rc >= 20: the Tello did not react at all to 8-12 rc points up/down, Nov 13); before: the band guard cut them
(it did on Nov 13 and spoiled the vertical fit). The command REALLY sent by the driver (after its guards) is
recorded in sent.csv and used by the fit. After landing: identification -> tello_step_fit (model.yaml);
validation -> the latest identified model is simulated on this flight (validation.txt / .png).

SAFETY: ANY KEY = stop and land. Ctrl+C = land. Automatic landing if the drone leaves a box around its start
point (default +-0.8 m horizontally, 0.4-2.4 m high), if no marker is seen for 3 s, or on low battery.
It refuses to start if another node (e.g. the teleop) also sends velocity commands.
"""
from tello_driver.missions.base import Lost, Mission, run

# (amplitude, half-duration) of the doublets, per axis kind: m/s or rad/s, s. Signs alternate.
SCHEDULES = {
    "ident": {"xy": [(0.15, 1.8), (0.25, 1.2), (0.35, 0.8), (0.25, 1.8), (0.35, 1.2), (0.15, 0.8)],
              "z": [(0.30, 0.4), (0.25, 0.5), (0.30, 0.3), (0.25, 0.4)],  # rc >= 20: the Tello ignored 8-12 (Nov 13)
              "yaw": [(0.3, 1.2), (0.5, 1.0), (0.7, 0.6), (0.5, 0.6)]},
    "valid": {"xy": [(0.20, 1.0), (0.30, 1.5), (0.30, 0.6), (0.20, 1.5)],
              "z": [(0.28, 0.4), (0.30, 0.35)],
              "yaw": [(0.4, 0.8), (0.6, 1.0)]},
}


class StepTest(Mission):
    NODE_NAME = "tello_step_test"
    PREFIX = "step"
    MOVE_TO_ZONE_CENTRE = True    # the pulses need room all around
    POSE_TOPIC = "/aruco/pose"    # the raw ArUco pose: the EKF uses the model being identified
    MIN_BATTERY = 50              # 3.5 min of flight and a full climb (at 32 % the Tello only climbed to 1.28 m)

    def __init__(self):
        super().__init__()
        self.axes = [a.strip() for a in self.declare_parameter("axes", "x,y,z,yaw").value.split(",") if a.strip()]

    def session_prefix(self):
        # called by Mission.__init__ (before the end of ours): the schedule decides the folder name
        self.schedule = self.declare_parameter("schedule", "ident").value  # ident | valid (see SCHEDULES)
        if self.schedule not in SCHEDULES:
            raise SystemExit(f"schedule must be one of {list(SCHEDULES)}")
        return "stepval" if self.schedule == "valid" else "step"

    def doublet(self, segment, axis, sign, amp, dur):
        """Hover (position held) - pulse +A for dur - pulse -A for dur - hover. During the pulses the tested axis is
        open loop and the other axes keep holding the position (u logged = asked; sent.csv = really sent)."""
        key = {"x": "cx", "y": "cy", "z": "cz", "yaw": "cw"}[axis]

        def cmd(u):
            c = self.hold_cmd()
            if u is not None:
                c[key] = u
            self.send(**c, segment=segment, axis=axis, u=c[key])
        for u, d in ((None, 2.0), (sign * amp, dur), (-sign * amp, dur), (None, 3.0)):
            self.run(d, lambda u=u: cmd(u), restart_on_loss=True)

    def mission(self):
        self.run_tests()

    def run_tests(self):
        names = {"x": "forward/back", "y": "left/right", "z": "up/down", "yaw": "turn"}
        sched = SCHEDULES[self.schedule]
        plan = [(axis, a, d) for axis in self.axes for a, d in sched["xy" if axis in ("x", "y") else axis]]
        unit = {"x": "m/s", "y": "m/s", "z": "m/s", "yaw": "rad/s"}
        self.say(f"{self.schedule} flight: {len(plan)} doublets")
        seg = 0
        for n, (axis, amp, dur) in enumerate(plan):
            sign = 1 if sum(1 for q in plan[:n] if q[0] == axis) % 2 == 0 else -1
            for attempt in range(3):
                self.say(f"[{n + 1}/{len(plan)}] {names[axis]}: {sign * amp:+.2f} {unit[axis]} for {dur:.1f} s, then back"
                         + (f" (retry {attempt})" if attempt else ""))
                self.recentre()
                try:
                    self.doublet(seg, axis, sign, amp, dur)
                    seg += 1
                    break
                except Lost:
                    self.failed_segments.append(seg)
                    seg += 1
                    self.say("  markers lost during the test: searching, then repeating it")
                    self.search_until_found(self.p0[3])
        self.return_home()
        self.say("Done: landing.")

    def after_landing(self):
        """Identification flight -> fit the model (model.yaml); validation flight -> simulate the last model on it."""
        from tello_driver.identification import step_fit
        if self.schedule == "valid":
            ident = step_fit.latest_identification(before=self.out)
            if ident is None:
                print("No identification flight to validate: run tello_step_test first")
            else:
                step_fit.validate(ident, self.out)
        else:
            step_fit.fit(self.out)
            print(f"Model: {self.out / 'model.yaml'}   plots: {self.out / 'step_fit.png'}, {self.out / 'cost_map.png'}")


def main():
    run(StepTest)


if __name__ == "__main__":
    main()
