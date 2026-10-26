#!/usr/bin/env python3
# Follow the ball by hand: wherever the ball is seen, the plate is tilted the way the controllers would want it to
# roll, towards the center, with a slope proportional to the distance and nothing else (no integral, no derivative).
# Hold the ball and move it around the plate: the side of the plate the ball is on must go UP, and the dashboard's
# yellow arrow (u) must point from the ball to the center. Anything else means the camera rotation or a servo is wrong.
#
#   python3 follow.py [--camera sonix] [--gain 0.0012] [--umax 0.12]

import argparse
import time

import numpy as np

import hardware
import kinematics
import stream
import vision

p = argparse.ArgumentParser()
p.add_argument("--camera", default="sonix")
p.add_argument("--gain", type=float, default=0.0012, help="slope per pixel of error")
p.add_argument("--umax", type=float, default=0.12)
p.add_argument("--detect", choices=["color", "background"], default="color")
p.add_argument("--stream", default="127.0.0.1:5006")
a = p.parse_args()

cam = hardware.start_uvc(a.camera, vision.W, vision.H)
hardware.home()
link = stream.Stream(a.stream, vision.W, vision.H, getattr(cam, "cal", None), vision.PX_PER_MM)
center = np.array(vision.center, dtype=float)
u = np.zeros(2)
try:
    while True:
        frame = cam.capture_array()
        t = time.monotonic()
        ball, _ = vision.detect(frame, a.detect)
        q_sent = q_actual = None
        if ball is not None:
            err = center - np.array(ball[:2])
            u = a.gain * err
            n = np.linalg.norm(u)
            if n > a.umax:
                u = u / n * a.umax
        else:
            u = u * 0.9                                     # no ball: ease back to level
        q = kinematics.solve(u, kinematics.H_NOM)
        if q is not None:
            q_deg = np.clip(q, hardware.Q_MIN, hardware.Q_MAX)
            q_actual = hardware.set_angle([q_deg[i] + hardware.OFFSETS[i] for i in range(3)]) - hardware.OFFSETS
            q_sent = q_deg
        link.send(t, ball, u if ball is not None else None, q_sent, q_actual, cam.t_capture)
except KeyboardInterrupt:
    pass
finally:
    hardware.home()
    time.sleep(0.5)
    hardware.shut_down(cam)
