# Everything that talks to the Raspberry Pi: the three servos and the camera.

from picamera2 import Picamera2
import cv2
import numpy as np
import pigpio
import time

import calibrate
import camclient
import servo
import vision

cv2.setNumThreads(1)      # one thread per process: three processes each spawning four OpenCV threads starve the camera broker on the Pi 4

# ACTUATION SETUP
SERVO_PINS = [13, 18, 12]
OFFSETS = [4, -8, -6]
Q_HOME = 60                              # motor angle at the nominal pose
Q_RANGE = 15                             # travel allowed around the nominal pose
Q_MIN, Q_MAX = Q_HOME - Q_RANGE, Q_HOME + Q_RANGE

# CONNECT PIGPIO
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpio daemon not running. Start with: sudo pigpiod")

# SERVO SENSE: in the original build a larger angle lowers that arm's side of the plate (kinematics.py, README section 5).
# In this build the horns turn the other way: raising an angle raised the plate (and the user leveled it by lowering the
# OFFSETS of the low sides), and with the controllers' angles sent as they were the plate tilted towards the ball instead
# of away from it. Each servo is therefore driven mirrored about its own home pulse, so the home pose and the OFFSETS
# stay as leveled and only the direction of every change flips. Set SERVO_DIR = 1 for a build like the original.
SERVO_DIR = -1

def angle_to_pulse(angle):
    # 0..180 deg -> 500..2500 us
    return 500 + (angle / 180.0) * 2000

def physical_angle(angle, i):
    # the angle really sent to servo i for a commanded angle (IK angle + offset): the same, or its mirror about home
    home = Q_HOME + OFFSETS[i]
    return home + SERVO_DIR * (angle - home)

# SLEW LIMIT: servo.slew, shared with the simulated robot
_q_last = None                           # angles last sent, and when
_t_last = None

def set_angle(angles, slew=True):
    # angles: the three servo angles wanted, with offsets. Returns the angles actually sent
    global _q_last, _t_last
    now = time.monotonic()
    q = np.array(angles, dtype=float)
    if slew and _q_last is not None:
        q = servo.slew(q, _q_last, now - _t_last)
    for i in range(3):
        pi.set_servo_pulsewidth(SERVO_PINS[i], angle_to_pulse(physical_angle(q[i], i)))
    _q_last, _t_last = q, now
    return q

def home():
    set_angle([Q_HOME + OFFSETS[i] for i in range(3)], slew=False)

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
# vision.PX_PER_MM, so the broker's frame is mapped (shifted to the plate center found by calibrate.py, scaled to that
# scale, turned to the mechanism's axes) onto the vision frame, which is large enough for the whole plate: what the
# camera does not see is black. Without calibration.json the whole frame is just resized, which is only right for a
# camera that happens to be centered under the plate at the nominal height.
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
            self.crop, self.map = None, None
        else:
            # the vision frame: the plate center at its center, vision.PX_PER_MM, axes turned to the mechanism's
            scale = vision.PX_PER_MM / cal["px_per_mm"]
            rot = cal.get("rot_deg", 0.0)
            M = cv2.getRotationMatrix2D((cal["cx"], cal["cy"]), rot, scale)   # turns by -rot (OpenCV's angle is counterclockwise with y down)
            M[:, 2] += (W / 2 - cal["cx"], H / 2 - cal["cy"])
            self.map = M
            w, h = 128 * cal["px_per_mm"], 96 * cal["px_per_mm"]             # the 128 x 96 mm the original build saw, for the dashboard
            self.crop = (cal["cx"] - w / 2, cal["cy"] - h / 2, w, h)
            self.cal = cal

    def lock_exposure(self):
        return self.cam.ctl("lock")

    def capture_array(self):
        self.seq, self.t_capture, frame = self.cam.read()
        self.last_raw = frame
        if self.map is None:
            return cv2.resize(frame, self.size, interpolation=cv2.INTER_AREA)   # BGR, like RGB888 from picamera2
        return cv2.warpAffine(frame, self.map, self.size, flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

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
