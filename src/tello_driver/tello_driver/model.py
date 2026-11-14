"""The Tello's identified response: the ONE place where the model lives (EKF, driver safety chain, missions, fake drone).

Per axis, from the velocity command u we send to the velocity v the drone reaches (its own flight controller
included):  tau dv/dt = K u(t - L) - v,  L = L_U (command side) + L_V (video side, command -> marker pose).

Identification flight step_20261113_180041 (tello_step_test v2, 6 doublets per horizontal axis), validated on a
separate flight (stepval_20261113_181035): fit 81 % forward / 67 % left / 87 % yaw, against 48 / 54 / 30 % for the
previous model (K 1.15, tau 0.55, L 0.50, identified Nov 11 with a marker map 6 % too large and 2 doublets).
  forward x: K 0.66, L + tau 0.73 s      left y: K 0.83, L + tau 0.85 s      yaw: K 1.00, L 0.20 s, tau ~0
L and tau trade off (only their sum is well determined), so one tau for both horizontal axes and L_U chosen to keep
the measured L + tau. The vertical axis could not be identified yet (the Tello ignored the small up/down doublets):
the previous values are kept for it.
"""
K_X = 0.66        # forward: real speed / commanded speed
K_Y = 0.83        # left
TAU_XY = 0.25     # s, horizontal time constant
L_V = 0.20        # s, image taken -> marker pose (video + detection)
L_U = 0.35        # s, command -> motion (L_U + L_V + TAU_XY = 0.80 s, measured 0.73-0.85 s)
K_Z, TAU_Z = 0.9, 0.45   # vertical: previous values (not re-identified yet)
K_W = 1.0         # yaw rate
K_MEAN = (K_X + K_Y) / 2
