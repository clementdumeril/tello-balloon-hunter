"""System identification: how the Tello answers a velocity command (gain K, time constant tau, delay L per axis).

    step_test.py  identification / validation flight (velocity doublets), a Mission flown on the raw ArUco pose
    step_fit.py   fits the first-order-plus-delay model on that flight -> model.yaml (copied into core/model.py)
"""
