# Finding the ball in the frame. Everything here works in pixels.

import cv2
import numpy as np

# DETECTION CONSTANTS
R_BALL_PX = 50.0                         # ball radius in pixels at the nominal height
AREA_MIN = 0.5 * np.pi * R_BALL_PX**2    # a blob smaller then this cannot be the ball
FILL_MIN = 0.50                          # the contour detected as the ball should fill at least 50% of its minimum enclosing circle
HSV_LO = (4, 170, 45)                    # orange ball (the original build used a pink one: (148, 110, 50)..(180, 255, 255))
HSV_HI = (20, 255, 255)

# IMAGE PROCESSING CONFIGURATION
W, H = 320, 240             # size of the camera frame
center = (W // 2, H // 2)

# two elliptical kernels
k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))


def find_ball(cnts):
    c = max(cnts, key=cv2.contourArea)          # find the contour of max area
    area = cv2.contourArea(c)                   # area of the contour
    if area < AREA_MIN:                         # first gate: the contour area must be >= 10% of a circle of radius R_BALL_PX
        return None
    (x, y), r = cv2.minEnclosingCircle(c)       # min eclosing circle of the candidate contour
    if area / (np.pi * r * r) < FILL_MIN:       # the candidate contour must be filled at >= 45% by the ball
        return None
    return x, y, r                              # center and radius of the minimum enclosing circle


def detect(frame):
    # IMAGE PROCESSING
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)                # converts to hsv
    mask = cv2.inRange(hsv, HSV_LO, HSV_HI)                     # keeps only the ball colour

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_open)       # morphological opening and closing to clean the image
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close)

    # mask is a binary image, so we can pass it to findContours() that finds the contour of mask
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    ball = find_ball(cnts) if cnts else None
    return ball, mask
