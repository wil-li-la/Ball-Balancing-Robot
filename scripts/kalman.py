# The state estimator: measured pixel error of the ball in, filtered error and its velocity out.

import numpy as np

# MODEL CONSTANTS, the same of linear_quadratic_control.m
G_ACC = 9.81                            # m/s^2
PX_PER_M = 50.0 / 0.02                  # the ball radius is 50 px in the frame and 0.02 m in reality
C = -(3/5) * G_ACC * PX_PER_M           # px/s^2 per unit of slope, from the moment of inertia of a hollow shell

# KALMAN CONSTANTS
SIGMA_A = 100.0                         # px/s^2, std of the acceleration the model does not explain (friction, servo lag, pushes)
R_MEAS = 1.0                            # px^2, variance of the measured error
P0_POS = R_MEAS                         # px^2, at reacquisition the position is as good as the measurement
P0_VEL = 100.0**2                       # (px/s)^2, at reacquisition the velocity is unknown

H = np.array([[1.0, 0.0]])              # only the error is actually measured, not its velocity

# INNOVATION GATE: a measurement further than this from the prediction is not the ball moving (the detection jumped to
# a hand or a reflection, the ball was touched) and is skipped, up to GATE_MAX frames in a row: after that it is taken,
# the ball really is somewhere else. 40 px at 30 fps is 3 m/s, far above anything a balanced ball does
GATE_PX = 40.0
GATE_MAX = 5
rejected = 0                            # measurements skipped in a row

# KALMAN INITIALIZATION
# x_hat[i] = [e, e_dot] along axis i. The two axes have the same model and the same noise,
# and the covariance does not depend on the measurements, so a single P serves both
x_hat = np.zeros((2, 2))                # state estimate
P = np.diag([P0_POS, P0_VEL])           # covariance of the estimate


def reset(err):
    # reacquisition: the position is the measurement, the velocity is unknown
    global x_hat, P, rejected
    x_hat = np.column_stack([err, np.zeros(2)])
    P = np.diag([P0_POS, P0_VEL])
    rejected = 0


def update(err, u, dt):
    # err is the measured error, u the slope commanded at the previous frame (even if it might be different from the actual slope due to clipping...)
    global x_hat, P, rejected
    F = np.array([[1.0, dt],
                  [0.0, 1.0]])
    G = np.array([C*dt**2/2, C*dt])
    Q = SIGMA_A**2 * np.array([[dt**4/4, dt**3/2],
                               [dt**3/2, dt**2  ]])

    # predict
    x_pred = x_hat @ F.T + np.outer(u, G)
    P_pred = F @ P @ F.T + Q

    # update
    y = err - x_pred[:, 0]
    if np.linalg.norm(y) > GATE_PX and rejected < GATE_MAX:     # not the ball: keep the prediction
        rejected += 1
        x_hat, P = x_pred, P_pred
        return x_hat[:, 0].copy(), x_hat[:, 1].copy()
    rejected = 0
    S = (H @ P_pred @ H.T)[0, 0] + R_MEAS
    K = P_pred @ H.T[:, 0] / S
    x_hat = x_pred + np.outer(y, K)
    P = (np.eye(2) - np.outer(K, H[0])) @ P_pred

    return x_hat[:, 0].copy(), x_hat[:, 1].copy()     # filtered error, its velocity


def predict(err, vel, u, horizon):
    # where the ball will be `horizon` seconds after the frame, under the slope u that is acting on it: the loop acts on
    # what the camera saw 60 to 100 ms ago (exposure, transfer, detection, servo pulse period, servo response), and a
    # ping pong ball crosses 2 to 3 cm in that time. The controllers are given this instead of the measured error,
    # so they command the plate for where the ball will be when the command takes effect
    acc = C * np.asarray(u, dtype=float)
    return err + vel * horizon + 0.5 * acc * horizon**2, vel + acc * horizon
