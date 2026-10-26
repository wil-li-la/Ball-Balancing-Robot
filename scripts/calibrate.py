#!/usr/bin/env python3
# Camera calibration from the rim of the plate: where the plate center is in the frame, and how many pixels a
# millimeter is there. The rim is the underside of the printed ring (teal in this build), an annulus from 69.1 mm to
# 76.0 mm measured on 3d files/plate.stl (the three lugs of the ball joints reach 84.9 mm), so neither the ball nor
# its size enters the calibration. The inner edge of the ring is the reference: it is a clean circle, the lugs are
# outside it; the outer edge is fitted too and only serves as a check.
#
#   python3 calibrate.py [--camera sonix] [--frames 30]     # the plate empty and at home, then the result is written to
#   scripts/calibration.json: {"cx", "cy", "r_px", "px_per_mm", "width", "height", "camera", "time"}
#
# hardware.UvcCamera reads that file and crops the frame around (cx, cy) so that the plate center is the center of
# the vision frame and the scale is vision.PX_PER_MM, the scale the controllers were tuned at. Run it again whenever
# the camera moves.
#
# "rot_deg" is the angle of the camera about its axis: the direction in the image (x right, y down, degrees) where
# the mechanism's +x axis (between motors 2 and 3, README section 2) appears. The vision frame is turned back by it, so
# its axes are the mechanism's and u maps onto the plate as the controllers expect. It is not measured by the rim
# (a circle has no angle): set it with --rot after placing a marker between motors 2 and 3, or from signtest.py.

import argparse
import json
import os
import time

import cv2
import numpy as np

import camclient
import kinematics

HERE = os.path.dirname(os.path.abspath(__file__))
PATH = os.path.join(HERE, "calibration.json")
RIM_HSV_LO = (80, 80, 40)                 # teal ring
RIM_HSV_HI = (105, 255, 255)
RIM_PIXELS_MIN = 2000                     # fewer rim pixels than this and the ring is not in view
R_RIM_IN = 69.07                          # [mm] inner radius of the ring's underside, from plate.stl
R_RIM_OUT = 76.00                         # [mm] outer radius of the ring's underside away from the lugs
EDGE_RESID_PX = (12.0, 6.0, 3.0, 2.0)     # edge pixels further than this from the fitted circle are dropped, round by round
                                          # (lugs, the clip, a hand over the ring, glare): wide first, since the first fit is skewed by them
BORDER_PX = 2                             # edge pixels this close to the frame border are the frame's edge, not the ring's
EDGE_SHARE_MIN = 0.5                      # a frame whose inner edge has fewer pixels than this share of the best frame's is dropped


def fit_circle(xs, ys):
    # algebraic least squares circle through points, (cx, cy, r): the ring is partly outside the frame, so an
    # enclosing circle would not do
    xs, ys = xs.astype(float), ys.astype(float)
    A = np.c_[xs, ys, np.ones_like(xs)]
    c = np.linalg.lstsq(A, xs**2 + ys**2, rcond=None)[0]
    cx, cy = c[0] / 2, c[1] / 2
    return cx, cy, np.sqrt(c[2] + cx**2 + cy**2)


def fit_edge(xs, ys):
    # circle fit with the outliers dropped: the lugs on the outer edge, a clip, a hand, glare on the plexiglass
    keep = np.ones(len(xs), bool)
    for thr in EDGE_RESID_PX:
        if keep.sum() < 50:
            return None
        cx, cy, r = fit_circle(xs[keep], ys[keep])
        resid = np.abs(np.hypot(xs - cx, ys - cy) - r)
        keep = resid < thr
    if keep.sum() < 50:
        return None
    cx, cy, r = fit_circle(xs[keep], ys[keep])
    return cx, cy, r, int(keep.sum())


def fit_rim(frame):
    # (cx, cy, r_in, r_out, n_in, n_out) in pixels, or None when the ring is not in view
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, RIM_HSV_LO, RIM_HSV_HI)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    ys, xs = np.nonzero(m)
    if len(xs) < RIM_PIXELS_MIN:
        return None
    cx0, cy0, _ = fit_circle(xs, ys)                      # rough center from every ring pixel, to tell the two edges apart
    edge = m - cv2.erode(m, np.ones((3, 3), np.uint8))
    edge[:BORDER_PX, :] = edge[-BORDER_PX:, :] = edge[:, :BORDER_PX] = edge[:, -BORDER_PX:] = 0
    ey, ex = np.nonzero(edge)
    d = np.hypot(ex - cx0, ey - cy0)
    mid = np.median(np.hypot(xs - cx0, ys - cy0))
    inner = fit_edge(ex[d < mid], ey[d < mid])
    outer = fit_edge(ex[d >= mid], ey[d >= mid])
    if inner is None:
        return None
    cx, cy, r_in, n_in = inner
    r_out, n_out = (outer[2], outer[3]) if outer is not None else (np.nan, 0)
    return cx, cy, r_in, r_out, n_in, n_out


def load(camera=None):
    # the live calibration (calibration.json, not in git), or, for a camera it is not about, the snapshot committed for that
    # camera (calibration.<camera>.json), so that a fresh clone starts from the last known values
    for path in (PATH, os.path.join(HERE, f"calibration.{camera}.json") if camera else None):
        if path and os.path.exists(path):
            with open(path) as f:
                cal = json.load(f)
            if camera is None or cal.get("camera") == camera:
                if path != PATH:
                    print(f"using the committed snapshot {os.path.basename(path)}: run calibrate.py after mounting the camera")
                return cal
    return None


def calibrate(camera, frames):
    cam = camclient.Camera(camera)
    fits = []
    for _ in range(frames):
        _, _, frame = cam.read()
        fit = fit_rim(frame)
        if fit is not None:
            fits.append(fit)
    cam.close()
    if len(fits) < frames // 2:
        raise SystemExit(f"rim not found in {frames - len(fits)} of {frames} frames: is the ring in view and the plate empty?")
    fits = np.array(fits, dtype=float)
    fits = fits[fits[:, 4] >= EDGE_SHARE_MIN * fits[:, 4].max()]     # frames with the ring largely covered (a hand) are dropped
    cx, cy, r_in, r_out, n_in, n_out = np.nanmedian(fits, axis=0)
    spread = np.ptp(fits[:, :3], axis=0)
    px_per_mm = r_in / R_RIM_IN
    cal = {"camera": camera, "width": cam.width, "height": cam.height,
           "cx": round(float(cx), 1), "cy": round(float(cy), 1), "r_px": round(float(r_in), 1),
           "px_per_mm": round(float(px_per_mm), 4),
           "time": time.strftime("%Y-%m-%d %H:%M:%S")}
    print(f"rim center ({cal['cx']}, {cal['cy']}) px, inner radius {cal['r_px']} px = {R_RIM_IN} mm -> {cal['px_per_mm']} px/mm")
    print(f"spread over {len(fits)} frames: {spread.round(1).tolist()} px (center x, center y, inner radius), {int(n_in)} inner edge pixels")
    if n_out:
        check = r_out / R_RIM_OUT
        print(f"check: outer radius {r_out:.1f} px = {R_RIM_OUT} mm -> {check:.4f} px/mm ({(check / px_per_mm - 1) * 100:+.1f}% vs inner, {int(n_out)} edge pixels)")
    return cal


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--camera", default="sonix")
    p.add_argument("--frames", type=int, default=60)
    p.add_argument("--dry", action="store_true", help="measure, do not write calibration.json")
    p.add_argument("--rot", type=float, help="camera rotation [deg]: image direction of the mechanism's +x axis (kept from the last calibration if not given)")
    p.add_argument("--rot-only", action="store_true", help="only store --rot in the existing calibration, do not refit the rim")
    a = p.parse_args()
    if a.rot_only:
        cal = load()
        if cal is None or a.rot is None:
            raise SystemExit("--rot-only needs an existing calibration.json and --rot")
    else:
        cal = calibrate(a.camera, a.frames)
        previous = load()
        if a.rot is None and (previous is None or "rot_deg" not in previous):
            print("WARNING: no camera rotation known, rot_deg = 0: set it with --rot once the mechanism's +x direction is known")
        cal["rot_deg"] = previous.get("rot_deg", 0.0) if previous else 0.0
    if a.rot is not None:
        cal["rot_deg"] = a.rot
    print(f"camera rotation {cal['rot_deg']} deg (mechanism +x appears at this image angle)")
    if not a.dry:
        with open(PATH, "w") as f:
            json.dump(cal, f, indent=2)
        print(f"written to {PATH}")
