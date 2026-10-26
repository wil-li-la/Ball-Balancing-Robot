# Everything that talks to the Raspberry Pi: the three servos and the camera.

from picamera2 import Picamera2
import cv2
import numpy as np
import pigpio
import time

import calibrate
import camclient
import vision

# ACTUATION SETUP
SERVO_PINS = [13, 18, 12]
OFFSETS = [0, -8, -6]
Q_HOME = 60                              # motor angle at the nominal pose
Q_RANGE = 15                             # travel allowed around the nominal pose
Q_MIN, Q_MAX = Q_HOME - Q_RANGE, Q_HOME + Q_RANGE

# CONNECT PIGPIO
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpio daemon not running. Start with: sudo pigpiod")

def angle_to_pulse(angle):
    # 0..180 deg -> 500..2500 us
    return 500 + (angle / 180.0) * 2000

def set_angle(angles):
    for i in range(3):
        pi.set_servo_pulsewidth(SERVO_PINS[i], angle_to_pulse(angles[i]))

def home():
    set_angle([Q_HOME + OFFSETS[i] for i in range(3)])

# CAMERA SETUP
TARGET_FPS = 20                   # frame rate demanded of the sensor
FRAME_US = int(1e6 / TARGET_FPS)  # the frame duration it corresponds to, in us

def start_camera(W, H):
    picam2 = Picamera2()          # start the frame capturing
    config = picam2.create_preview_configuration(
        main={"size": (W, H), "format": "RGB888"},
        controls={"FrameDurationLimits": (FRAME_US, FRAME_US)}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)
    return picam2

# EXPOSURE LOCK: the background detection of vision.py compares every frame with an image of the empty plate, so the
# camera must not change its exposure or its white balance when the ball comes in and darkens the frame
def lock_exposure(cam):
    if isinstance(cam, UvcCamera):
        return cam.lock_exposure()
    meta = cam.capture_metadata()     # libcamera reports what its auto exposure chose, so it can be frozen there
    cam.set_controls({"AeEnable": False, "AwbEnable": False,
                      "ExposureTime": meta["ExposureTime"], "AnalogueGain": meta["AnalogueGain"],
                      "ColourGains": meta["ColourGains"]})
    time.sleep(0.2)
    return f"exposure {meta['ExposureTime']} us, gain {meta['AnalogueGain']:.2f}"

# USB CAMERAS: plain UVC cameras (a Sonix webcam, the D435 colour sensor) read through the camera broker camd.py, which
# owns the V4L2 device and shares every frame with any number of readers (the control loop and the dashboard at once).
# The broker must be running: python3 camd.py --camera sonix
# The controllers were tuned in pixels of a frame where the plate center is the frame center and the scale is
# vision.PX_PER_MM, so the broker's frame is cropped around the plate center found by calibrate.py (the rim of the
# plate, 84 mm) and resized until that scale holds. Without calibration.json the whole frame is used, which is only
# right for a camera that happens to be centered under the plate at the nominal height.
UVC_CAMERAS = ("sonix", "imageplus", "d435", "test")      # test = the synthetic frames of camd.py, to run without a camera

class UvcCamera:
    # same two calls the main loop makes on a Picamera2: capture_array() and stop()
    def __init__(self, name, W, H):
        self.name, self.size = name, (W, H)
        self.cam = camclient.Camera(name)
        self.seq, self.t_capture, self.last_raw = 0, 0.0, None
        cal = calibrate.load()
        if cal is None or cal["camera"] != name:
            print(f"no calibration for {name} (run calibrate.py): using the whole frame")
            self.crop = None
        else:
            w = W * cal["px_per_mm"] / vision.PX_PER_MM          # pixels of the broker's frame that cover the vision frame
            h = H * cal["px_per_mm"] / vision.PX_PER_MM
            self.crop = (cal["cx"] - w / 2, cal["cy"] - h / 2, w, h)    # (x0, y0, w, h), may leave the frame: padded by replication
            self.cal = cal

    def lock_exposure(self):
        return self.cam.ctl("lock")

    def capture_array(self):
        self.seq, self.t_capture, frame = self.cam.read()
        self.last_raw = frame
        if self.crop is not None:
            x0, y0, w, h = self.crop
            frame = cv2.getRectSubPix(frame, (int(round(w)), int(round(h))), (x0 + w / 2, y0 + h / 2))
        return cv2.resize(frame, self.size, interpolation=cv2.INTER_AREA)   # BGR, like RGB888 from picamera2

    def stop(self):
        self.cam.ctl("unlock")             # the camera is left as it was found
        self.cam.close()

def start_uvc(name, W, H):
    cam = UvcCamera(name, W, H)
    for _ in range(30):           # let auto exposure settle
        cam.capture_array()
    return cam

def shut_down(picam2):
    picam2.stop()
    for pin in SERVO_PINS:
        pi.set_servo_pulsewidth(pin, 0)
    pi.stop()
