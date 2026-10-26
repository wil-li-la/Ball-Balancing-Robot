#!/usr/bin/env python3
# Does a slope u really make the ball accelerate along +u in the vision frame? Open-loop test of the whole chain
# camera -> calibration -> vision -> kinematics -> servos -> plate -> ball, trial by trial:
#   the plate waits level; when the ball has been seen near the center and nearly still for a few frames, a fixed
#   slope is applied for a fraction of a second while the ball is tracked, then the plate goes back to level. A
#   parabola is fitted to the track, so the speed the ball had when it was let go drops out and only its
#   acceleration is compared with the direction of u. Put the ball near the center and let it go; repeat when asked.
#
#   python3 signtest.py [--camera sonix] [--u 0.15] [--frames 18] [--trials 2]
#
# The controllers need the ball to accelerate along +u (README section 6 and 8): "ALIGNED" on all four directions
# means the vision frame's axes are the mechanism's (calibration.json rot_deg is right); a consistent offset is the
# correction to add to rot_deg; "OPPOSITE" on one axis only would mean a mirrored image.

import argparse
import time

import numpy as np

import hardware
import kinematics
import vision

p = argparse.ArgumentParser()
p.add_argument("--camera", default="sonix")
p.add_argument("--u", type=float, default=0.15)
p.add_argument("--frames", type=int, default=12, help="frames the slope is held")
p.add_argument("--trials", type=int, default=2, help="trials per direction")
p.add_argument("--detect", choices=["color", "background"], default="color")
a = p.parse_args()

G_PX = 0.6 * 9.81 * 1000 * vision.PX_PER_MM     # px/s^2 per unit slope for a hollow ball, README section 8
A_MIN = 0.3 * G_PX * a.u                        # an acceleration smaller than this is not the plate's doing: the ball was held or stuck

cam = hardware.start_uvc(a.camera, vision.W, vision.H)
hardware.home()
time.sleep(0.5)
if a.detect == "background":
    print(hardware.lock_exposure(cam))
    vision.calibrate([cam.capture_array() for _ in range(vision.BG_FRAMES)])


def ball():
    b, _ = vision.detect(cam.capture_array(), a.detect)
    return None if b is None else np.array(b[:2]) - np.array(vision.center)


def q_for(u):
    q = kinematics.solve(np.array(u, float), kinematics.H_NOM)
    return [float(np.clip(q[i], hardware.Q_MIN, hardware.Q_MAX)) + hardware.OFFSETS[i] for i in range(3)]


def direction(d):
    return np.degrees(np.arctan2(d[1], d[0])) % 360     # image angle: x right, y down


def accel(track):
    # (acceleration, speed at the start) from a parabola through the track, [px/s^2], [px/s]
    tt = np.array([t for t, _ in track]); pp = np.array([q for _, q in track])
    tt = tt - tt[0]
    A = np.c_[np.ones_like(tt), tt, tt**2 / 2]
    coef, *_ = np.linalg.lstsq(A, pp, rcond=None)
    return coef[2], coef[1]


results = []
try:
    for u in ([a.u, 0], [-a.u, 0], [0, a.u], [0, -a.u]):
        done = 0
        while done < a.trials:
            print(f"\nnext: u = {u}. Put the ball near the center and let go...", flush=True)
            recent = []
            while True:                                      # the ball near the center and slow, six frames in a row
                b = ball()
                recent = (recent + [b])[-6:]
                if len(recent) == 6 and all(x is not None and np.linalg.norm(x) < 80 for x in recent) \
                        and np.linalg.norm(recent[-1] - recent[0]) < 12:
                    break
            track = []
            hardware.set_angle(q_for(u), slew=False)         # a crisp step, the servos take it in a frame or two
            for _ in range(a.frames):
                b = ball()
                if b is not None:
                    track.append((time.monotonic(), b))
            hardware.home()
            if len(track) < 5:                               # a ball that leaves the view early still gives its first frames
                print(f"  ball seen in only {len(track)} frames during the tilt, again")
                time.sleep(1.5)
                continue
            acc, v0 = accel(track)
            if np.linalg.norm(acc) < A_MIN:
                print(f"  acceleration only {np.linalg.norm(acc):.0f} px/s^2 (expected ~{G_PX * a.u:.0f}): was the ball held? again")
                time.sleep(1.5)
                continue
            err = (direction(acc) - direction(u) + 180) % 360 - 180
            verdict = "ALIGNED" if abs(err) < 30 else "OPPOSITE" if abs(err) > 150 else "ROTATED"
            print(f"  accel {np.round(acc).astype(int).tolist()} px/s^2 ({np.linalg.norm(acc):.0f}, expected ~{G_PX * a.u:.0f}), "
                  f"direction {direction(acc):4.0f} deg, u is {direction(u):4.0f} deg, off by {err:+4.0f} -> {verdict}   (start speed {np.linalg.norm(v0):.0f} px/s)", flush=True)
            results.append((u, err, verdict, np.linalg.norm(acc)))
            done += 1
            time.sleep(1.5)                                  # time to catch the ball
finally:
    hardware.home()
    time.sleep(0.5)
    hardware.shut_down(cam)
    if results:
        print("\nsummary:")
        for u, err, v, mag in results:
            print(f"  u = {str(u):14s} off by {err:+4.0f} deg  {v}   |a| {mag:.0f} px/s^2")
        errs = np.array([e for _, e, _, _ in results])
        mean = np.degrees(np.arctan2(np.sin(np.radians(errs)).mean(), np.cos(np.radians(errs)).mean()))
        print(f"  mean angle from u to the ball's acceleration: {mean:+.0f} deg -> add this to rot_deg in calibration.json if it is not ~0")
        print(f"  mean |a| / expected: {np.mean([m for *_, m in results]) / (G_PX * a.u):.2f}")
