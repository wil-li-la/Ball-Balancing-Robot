# Finding the ball in the frame. Everything here works in pixels.
#
# Two ways to tell the ball from the rest of the frame, chosen with --detect in ball_balancer.py:
# - "background" (default): the camera is fixed to the base and looks up through the transparent plate, so what it
#   sees behind the plate does not move when the plate tilts. A model of the empty frame is taken at the start
#   (calibrate), and the ball is what differs from it: any color, as long as it is not the color of what is behind it
# - "color": the pixels inside the HSV range of the ball, which has to be tuned again for a ball of another color
# Both end in the same shape test (find_ball), which does not look at the color.

import cv2
import numpy as np

# DETECTION CONSTANTS
R_BALL_PX = 50.0                         # ball radius in pixels at the nominal height
PX_PER_MM = R_BALL_PX / 20.0             # the scale of the frame the controllers were tuned at: a ping pong ball is 20 mm in radius
AREA_MIN = 0.5 * np.pi * R_BALL_PX**2    # a blob smaller then this cannot be the ball
R_MAX = 1.6 * R_BALL_PX                  # nor a blob whose enclosing circle is much larger than the ball (a hand, a shadow)
FILL_MIN = 0.50                          # the contour detected as the ball should fill at least 50% of its minimum enclosing circle
HSV_LO = (4, 170, 45)                    # orange ball (the original build used a pink one: (148, 110, 50)..(180, 255, 255))
HSV_HI = (20, 255, 255)

# BACKGROUND CONSTANTS
BG_FRAMES = 30                           # frames of the empty plate averaged by calibrate, 1 to 1.5 s
BG_THR_MIN = 12.0                        # [Lab units] smallest difference from the background that counts as foreground
BG_K_SIGMA = 4.0                         # ... or this many standard deviations of the noise of that pixel, if larger
BG_SHIFT_PX = 1.0                        # [px] shift of the image the threshold tolerates: the servos shake the base, and the camera with it,
                                         # so the edges of what is behind the plate move by a pixel or so
BG_ALPHA = 0.02                          # rate at which the background follows slow changes of light where nothing is detected (~2.5 s at 20 Hz)
BG_ALPHA_FG = 0.002                      # rate on foreground that is not the ball, so that a lasting change (a lamp turned on) is absorbed in ~25 s

# IMAGE PROCESSING CONFIGURATION
W, H = 352, 352             # size of the vision frame: at PX_PER_MM it covers the whole plate (inner radius 69 mm) around its center,
center = (W // 2, H // 2)   # whatever the camera sees; the original build had a 320x240 frame, 128 x 96 mm

# two elliptical kernels
k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

# BACKGROUND MODEL, set by calibrate
bg = None                   # Lab image of the empty frame, float32
bg_thr = None               # per pixel threshold on the difference from bg


def find_ball(cnts):
    # the largest contour that looks like the ball: other blobs (a hand reaching in) can be larger
    for c in sorted(cnts, key=cv2.contourArea, reverse=True):
        area = cv2.contourArea(c)                   # area of the contour
        if area < AREA_MIN:                         # first gate: the contour area must be >= 50% of a circle of radius R_BALL_PX
            return None                             # sorted by area, so none of the next ones passes either
        (x, y), r = cv2.minEnclosingCircle(c)       # min eclosing circle of the candidate contour
        if r > R_MAX:                               # second gate: not much larger than the ball
            continue
        if area / (np.pi * r * r) < FILL_MIN:       # third gate: the contour must fill >= 50% of its enclosing circle
            continue
        return x, y, r                              # center and radius of the minimum enclosing circle
    return None


def to_lab(frame):
    # blurred a little: the difference is taken pixel by pixel, and the noise of a single pixel is large
    return cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2LAB).astype(np.float32)


def calibrate(frames):
    # frames of the empty plate -> background and per pixel threshold
    global bg, bg_thr
    lab = np.stack([to_lab(f) for f in frames])
    bg = np.median(lab, axis=0)
    noise = lab.std(axis=0).max(axis=2)
    gx = np.abs(cv2.Sobel(bg, cv2.CV_32F, 1, 0, ksize=3)).max(axis=2) / 8    # Sobel 3x3 is 8 times the gradient per pixel
    gy = np.abs(cv2.Sobel(bg, cv2.CV_32F, 0, 1, ksize=3)).max(axis=2) / 8
    bg_thr = np.maximum(BG_THR_MIN, BG_K_SIGMA * noise) + BG_SHIFT_PX * np.maximum(gx, gy)
    return float(np.median(noise))      # typical noise of a pixel, for the user to judge the calibration


def mask_color(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)                # converts to hsv
    return cv2.inRange(hsv, HSV_LO, HSV_HI)                     # keeps only the ball colour


def mask_background(lab):
    diff = np.abs(lab - bg).max(axis=2)                         # lightness and color together: a white or a black ball differs in L only
    return (diff > bg_thr).astype(np.uint8) * 255


def update_background(lab, mask, ball):
    # follows slow changes of light, but never learns the ball
    alpha = np.where(mask > 0, BG_ALPHA_FG, BG_ALPHA).astype(np.float32)
    if ball is not None:
        x, y, r = ball
        cv2.circle(alpha, (int(x), int(y)), int(r * 1.3), 0.0, -1)
    bg[:] += alpha[..., None] * (lab - bg)


def detect(frame, method="background"):
    # IMAGE PROCESSING
    if method == "background":
        if bg is None:
            raise RuntimeError("vision.calibrate() must see the empty plate before detect(method='background')")
        lab = to_lab(frame)
        mask = mask_background(lab)
    else:
        mask = mask_color(frame)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_open)       # morphological opening and closing to clean the image
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close)

    # mask is a binary image, so we can pass it to findContours() that finds the contour of mask
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    ball = find_ball(cnts) if cnts else None
    if method == "background":
        update_background(lab, mask, ball)
    return ball, mask
