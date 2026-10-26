# What is done to the servo angles between the controller and the pulses, as pure functions, so that the simulated
# robot (reinforcement_learning/sim_robot.py) does exactly what hardware.py does. No Pi imports here.

import numpy as np

# SLEW LIMIT: the SG90 follows a step of a few degrees within a frame, so a command that jumps 15 deg between two
# frames (a derivative term on a noisy measurement) is taken whole and shakes the whole robot. Every call may move each
# servo by at most SLEW_DEG_PER_S x (time since the last call), so a jump is spread over a few frames and the plate
# moves at a speed the ball can follow. 150 deg/s = 5 deg per frame at 30 fps: the whole travel in 6 frames
SLEW_DEG_PER_S = 150.0
DT_MAX = 0.1                              # [s] a longer pause than this (a reacquisition) does not buy a larger step


def slew(q_wanted, q_last, dt, rate=SLEW_DEG_PER_S):
    # the angles to send now: q_wanted, or as far towards it from q_last as the rate allows in dt seconds
    q_wanted = np.asarray(q_wanted, dtype=float)
    if q_last is None:
        return q_wanted
    step = rate * min(dt, DT_MAX)
    return np.asarray(q_last, dtype=float) + np.clip(q_wanted - q_last, -step, step)
