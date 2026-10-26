#!/usr/bin/env python3
"""Ramp the three servos smoothly to a pose and hold it.
usage: servo_goto.py home | 90 | q1 q2 q3 | release     (angles in deg, offsets NOT applied to explicit angles)
Run from any dir; imports hardware.py from the repo."""
import sys, time
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
import hardware as hw
pins = hw.SERVO_PINS
cur = [hw.pi.get_servo_pulsewidth(p) for p in pins]
cur = [None if pw == 0 else (pw - 500) / 2000 * 180 for pw in cur]
arg = sys.argv[1:]
if arg == ["release"]:
    for p in pins: hw.pi.set_servo_pulsewidth(p, 0)
    print("released, no pulses"); sys.exit()
if arg == ["home"]:
    target = [hw.Q_HOME + hw.OFFSETS[i] for i in range(3)]
elif len(arg) == 1:
    target = [float(arg[0])] * 3
else:
    target = [float(a) for a in arg]
if any(c is None for c in cur):
    hw.set_angle(target)                       # no known position: go directly
else:
    steps = 30
    for s in range(1, steps + 1):
        hw.set_angle([cur[i] + (target[i] - cur[i]) * s / steps for i in range(3)]); time.sleep(0.05)
time.sleep(0.8)
print("holding", target, "deg, pulse us:", [hw.pi.get_servo_pulsewidth(p) for p in pins])
hw.pi.stop()
