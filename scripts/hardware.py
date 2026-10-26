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

# USB CAMERAS: plain UVC cameras (the D435 colour sensor, a Sonix webcam), read through V4L2 instead of libcamera
# name -> by-id device, capture mode; both run 640x480 YUYV at 30 fps, the closest 4:3 mode they have (no 20 fps mode)
UVC_CAMERAS = {
    "d435": "/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435_Intel_R__RealSense_TM__Depth_Camera_435-video-index0",
    "sonix": "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB2.0_CAM1_USB2.0_CAM1-video-index0",
}
UVC_W, UVC_H, UVC_FPS = 640, 480, 30

class UvcCamera:
    # same two calls the main loop makes on a Picamera2: capture_array() and stop()
    def __init__(self, name, W, H):
        self.name, self.size = name, (W, H)
        dev = UVC_CAMERAS[name]
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open {dev}")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, UVC_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, UVC_H)
        self.cap.set(cv2.CAP_PROP_FPS, UVC_FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)    # always the newest frame, not a queued one

    def capture_array(self):
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError(f"{self.name} frame read failed")
        return cv2.resize(frame, self.size, interpolation=cv2.INTER_AREA)   # BGR, like RGB888 from picamera2

    def stop(self):
        self.cap.release()

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
