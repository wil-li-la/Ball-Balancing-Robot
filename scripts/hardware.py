# Everything that talks to the Raspberry Pi: the three servos and the camera.

from picamera2 import Picamera2
import cv2
import pigpio
import time

# ACTUATION SETUP
SERVO_PINS = [13, 18, 12]
OFFSETS = [0, -5, -5]
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

# D435 SETUP: its colour sensor is a plain UVC camera, read through V4L2 instead of libcamera
D435_DEV = "/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435_Intel_R__RealSense_TM__Depth_Camera_435-video-index0"
D435_W, D435_H, D435_FPS = 640, 480, 30     # the closest 4:3 mode it has (no 20 fps mode)

class UvcCamera:
    # same two calls the main loop makes on a Picamera2: capture_array() and stop()
    def __init__(self, W, H):
        self.size = (W, H)
        self.cap = cv2.VideoCapture(D435_DEV, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open {D435_DEV}")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, D435_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, D435_H)
        self.cap.set(cv2.CAP_PROP_FPS, D435_FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)    # always the newest frame, not a queued one

    def capture_array(self):
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("D435 frame read failed")
        return cv2.resize(frame, self.size, interpolation=cv2.INTER_AREA)   # BGR, like RGB888 from picamera2

    def stop(self):
        self.cap.release()

def start_d435(W, H):
    cam = UvcCamera(W, H)
    for _ in range(30):           # let auto exposure settle
        cam.capture_array()
    return cam

def shut_down(picam2):
    picam2.stop()
    for pin in SERVO_PINS:
        pi.set_servo_pulsewidth(pin, 0)
    pi.stop()
