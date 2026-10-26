#!/usr/bin/env python3
# Level the plate from a balancing run: the slope the controller had to hold on average to keep the ball at the center
# is the slope the plate has when the servos are at home. Folding it into hardware.OFFSETS makes home level, so the
# integral term starts from zero instead of having to find it after every reacquisition.
#
#   python3 level.py runs/run_....jsonl          # prints the mean slope held and the OFFSETS that would cancel it

import json
import sys

import numpy as np

import hardware
import kinematics

msgs = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
held = [m for m in msgs if m["ball"] is not None and m["u"] is not None]
# only frames with the ball near the center count: there the plate is close to level and the slope is the bias
near = [m for m in held if np.hypot(m["ball"][0] - m["size"][0] / 2, m["ball"][1] - m["size"][1] / 2) < 25 * 2.5]
if len(near) < 90:
    raise SystemExit(f"only {len(near)} frames with the ball within 25 mm of the center, not enough")
u_mean = np.mean([m["u"] for m in near], axis=0)
q0 = kinematics.solve(np.zeros(2), kinematics.H_NOM)
q1 = kinematics.solve(u_mean, kinematics.H_NOM)
shift = q1 - q0
new = [round(hardware.OFFSETS[i] + shift[i]) for i in range(3)]
print(f"{len(near)} frames near the center: mean slope held u = [{u_mean[0]:+.3f}, {u_mean[1]:+.3f}] ({np.degrees(np.hypot(*u_mean)):.1f} deg)")
print(f"servo angles for that slope differ from level by {np.round(shift, 2).tolist()} deg")
print(f"OFFSETS {hardware.OFFSETS} -> {new}")
