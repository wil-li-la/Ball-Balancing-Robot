#!/usr/bin/env python3
# Camera broker: the one process that owns the camera, so that the control loop and the dashboard can read the
# same frames at the same time (a V4L2 device can be opened by one process only).
#
#   python3 camd.py --camera sonix          # runs until killed
#
# Every frame goes to a shared memory file (/dev/shm/camd-<name>) and a notification to every client connected to
# the unix socket (/tmp/camd-<name>.sock), so a client blocks on the socket and never polls. The file holds a header
# and two frame slots. The header carries two counters: wseq, bumped BEFORE slot wseq % 2 is written, and seq, set to
# wseq AFTER. A reader copies slot seq % 2 and then checks that wseq has not reached seq + 2, which would mean the
# slot was being overwritten under it (a torn frame); then it copies again.
# A second socket (/tmp/camd-<name>.ctl) takes text commands: "lock" / "unlock" freeze or release the exposure
# (vision.py's background model needs the exposure not to follow the ball), "status" reports.
# The client side is camclient.py; hardware.UvcCamera uses it.

import argparse
import mmap
import os
import socket
import struct
import subprocess
import threading
import time

import cv2
import numpy as np

cv2.setNumThreads(1)                     # the broker must never wait for a thread pool: late dequeues corrupt frames (see BUFFERSIZE)

DEVICES = {
    "sonix": "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB2.0_CAM1_USB2.0_CAM1-video-index0",
    "d435": "/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435_Intel_R__RealSense_TM__Depth_Camera_435-video-index0",
    "imageplus": "/dev/v4l/by-id/usb-Image+_Fic760xUsbCamera_HU12345678-video-index0",
    "test": None,                        # synthetic frames: the rim of the plate and a ball going round, to run everything without a camera
}


class Synthetic:
    # the few calls of cv2.VideoCapture the broker makes, drawing a plate seen from below and an orange ball on a circle
    def __init__(self, width, height, fps):
        self.width, self.height, self.fps = width, height, fps
        self.t0 = time.monotonic()
        self.n = 0

    def isOpened(self):
        return True

    def set(self, *a):
        pass

    def read(self):
        t_due = self.t0 + self.n / self.fps
        time.sleep(max(0.0, t_due - time.monotonic()))
        self.n += 1
        self.t = time.monotonic()
        f = np.full((self.height, self.width, 3), 235, np.uint8)
        cx, cy, r = self.width // 2 + 40, self.height // 2 - 20, 150            # plate center off the frame center, like a real mount
        cv2.circle(f, (cx, cy), r, (170, 150, 0), 10)                            # teal rim (BGR)
        for k in range(-3, 4):                                                   # the grid of the ceiling lamp behind the plate
            cv2.line(f, (cx + 45 * k - 200, cy - 200), (cx + 45 * k + 200, cy + 200), (120, 120, 120), 2)
            cv2.line(f, (cx - 45 * k - 200, cy + 200), (cx - 45 * k + 200, cy - 200), (120, 120, 120), 2)
        a = 2 * np.pi * 0.2 * (self.t - self.t0)
        bx, by = int(cx + 60 * np.cos(a)), int(cy + 60 * np.sin(a))
        cv2.circle(f, (bx, by), 31, (20, 110, 240), -1)                          # orange ball, 31 px as the Sonix saw it
        return True, f

    def get(self, prop):
        return self.t * 1000.0 if prop == cv2.CAP_PROP_POS_MSEC else 0.0

    def release(self):
        pass
POWER_LINE = 2                           # v4l2 menu: 0 off, 1 = 50 Hz, 2 = 60 Hz (Taiwan)
# CORRUPT FRAMES: when the USB link loses packets, uvcvideo (nodrop) still hands over the frame, with everything after
# the loss shifted: a ball that is not where it is. Such a frame differs from the previous one everywhere, a real
# change (a hand, the ball) only locally, so a frame whose mean difference from the last good one is above
# BAD_DIFF is dropped, up to BAD_MAX in a row (then the scene really changed and the next frame is taken)
BAD_DIFF = 20.0
BAD_MAX = 10
MAGIC = b"CAMD"
HEADER = struct.Struct("<4sIIIQQd")      # magic, width, height, channels, seq, wseq, t_capture [s, CLOCK_MONOTONIC]
HEADER_SIZE = 64                         # the header is padded so the slots start on a round offset


def shm_path(name):
    return f"/dev/shm/camd-{name}"


def sock_path(name):
    return f"/tmp/camd-{name}.sock"


def ctl_path(name):
    return f"/tmp/camd-{name}.ctl"


class Broker:
    def __init__(self, name, width, height, fps, fourcc="MJPG"):
        self.name, self.dev = name, DEVICES[name]
        self.width, self.height, self.fps = width, height, fps
        if self.dev is None:
            self.cap = Synthetic(width, height, fps)
        else:
            self.cap = cv2.VideoCapture(self.dev, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                raise RuntimeError(f"cannot open {self.dev}")
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))   # MJPG: a fifth of the USB bandwidth of YUYV, see --fourcc
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)       # a spare buffer: with one, a late dequeue leaves the driver nowhere to put
                                                           # the next frame and uvcvideo hands over a frame with its rows shifted
            self.v4l2("-c", "exposure_dynamic_framerate=0")   # in dim light the auto exposure would otherwise drop the frame rate
            self.v4l2("-c", "auto_exposure=3,white_balance_automatic=1")
            self.v4l2("-c", f"power_line_frequency={POWER_LINE}")   # the lamp behind the plate flickers at the mains frequency
        self.locked = None                                 # the manual exposure in use, None while auto

        slot = width * height * 3
        self.slot = slot
        fd = os.open(shm_path(name), os.O_CREAT | os.O_RDWR, 0o666)
        os.ftruncate(fd, HEADER_SIZE + 2 * slot)
        self.mem = mmap.mmap(fd, HEADER_SIZE + 2 * slot)
        os.close(fd)
        self.seq, self.t_capture = 0, 0.0
        self.frame = None                                  # last frame, for the exposure lock
        self.small = None                                  # last good frame at quarter size, for the corrupt frame check
        self.dropped, self.dropped_run = 0, 0
        self.lock = threading.Lock()

        self.clients = []
        self.notify = self.listen(sock_path(name))
        self.ctl = self.listen(ctl_path(name))
        threading.Thread(target=self.accept_loop, args=(self.notify, self.clients.append), daemon=True).start()
        threading.Thread(target=self.accept_loop, args=(self.ctl, self.serve_ctl), daemon=True).start()

    # SOCKETS
    def listen(self, path):
        if os.path.exists(path):
            os.unlink(path)
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.bind(path)
        os.chmod(path, 0o666)
        s.listen(8)
        return s

    def accept_loop(self, server, on_client):
        while True:
            c, _ = server.accept()
            on_client(c)

    def serve_ctl(self, c):
        threading.Thread(target=self.ctl_loop, args=(c,), daemon=True).start()

    def ctl_loop(self, c):
        f = c.makefile("rw")
        for line in f:
            cmd = line.strip()
            try:
                if cmd == "lock":
                    reply = self.lock_exposure()
                elif cmd == "unlock":
                    reply = self.unlock_exposure()
                elif cmd == "status":
                    reply = f"{self.name} {self.width}x{self.height} seq {self.seq} dropped {self.dropped} clients {len(self.clients)} exposure {self.locked or 'auto'}"
                else:
                    reply = f"error unknown command {cmd!r}"
            except Exception as e:
                reply = f"error {e}"
            f.write(reply + "\n")
            f.flush()
        c.close()

    # EXPOSURE (the same search as the twin session's UvcCamera.lock_exposure: a UVC camera does not report the
    # exposure its auto mode chose, and switching to manual falls back to a stored value, so the manual exposure is
    # searched until the frame is as bright as the auto exposure made it)
    def v4l2(self, *args):
        if self.dev is None:
            return ""
        out = subprocess.run(["v4l2-ctl", "-d", self.dev, *args], capture_output=True, text=True)
        if out.returncode != 0 or out.stderr:
            raise RuntimeError(f"v4l2-ctl {' '.join(args)}: {out.stderr.strip()}")
        return out.stdout

    def brightness(self, n=6):
        seq0 = self.seq
        while self.seq < seq0 + 3:            # frames taken before a new setting reached the sensor
            time.sleep(0.005)
        vals = []
        while len(vals) < n:
            seq0 = self.seq
            while self.seq == seq0:
                time.sleep(0.005)
            with self.lock:
                vals.append(float(self.frame.mean()))
        return float(np.mean(vals))

    def lock_exposure(self):
        target = self.brightness()
        self.v4l2("-c", "auto_exposure=1,white_balance_automatic=0")
        lo, hi = 1, int(1e4 / self.fps)          # [100 us] up to one frame period
        while hi - lo > 1:
            mid = (lo + hi) // 2
            self.v4l2("-c", f"exposure_time_absolute={mid}")
            if self.brightness() < target:
                lo = mid
            else:
                hi = mid
        self.v4l2("-c", f"exposure_time_absolute={hi}")
        self.locked = hi * 100
        return f"ok exposure {self.locked} us, brightness {self.brightness():.0f} (auto {target:.0f})"

    def unlock_exposure(self):
        self.v4l2("-c", "auto_exposure=3,white_balance_automatic=1")
        self.locked = None
        return "ok exposure auto"

    # FRAMES
    def publish(self, frame, t_capture):
        wseq = self.seq + 1
        self.mem[:HEADER.size] = HEADER.pack(MAGIC, self.width, self.height, 3, self.seq, wseq, self.t_capture)
        off = HEADER_SIZE + (wseq % 2) * self.slot
        self.mem[off:off + self.slot] = frame.tobytes()
        self.seq, self.t_capture = wseq, t_capture
        self.mem[:HEADER.size] = HEADER.pack(MAGIC, self.width, self.height, 3, self.seq, wseq, t_capture)
        token = struct.pack("<Q", self.seq)
        for c in list(self.clients):
            try:
                c.send(token, socket.MSG_DONTWAIT)
            except OSError:                      # a client that left, or one too slow to drain its socket
                self.clients.remove(c)
                c.close()

    def run(self):
        print(f"camd {self.name}: {self.dev} {self.width}x{self.height} @ {self.fps} fps -> {shm_path(self.name)}", flush=True)
        t_report, n_report = time.monotonic(), 0
        while True:
            ok, frame = self.cap.read()
            if not ok:
                raise RuntimeError(f"{self.name} frame read failed")
            ts = self.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0     # V4L2 stamps the buffer at start of exposure, CLOCK_MONOTONIC
            t_capture = ts if ts > 0 else time.monotonic()
            small = frame[::4, ::4].astype(np.int16)
            if self.small is not None and np.abs(small - self.small).mean() > BAD_DIFF and self.dropped_run < BAD_MAX:
                self.dropped += 1
                self.dropped_run += 1
                continue
            self.dropped_run = 0
            self.small = small
            with self.lock:
                self.frame = frame
            self.publish(frame, t_capture)
            n_report += 1
            if time.monotonic() - t_report >= 10:
                print(f"camd {self.name}: {n_report / (time.monotonic() - t_report):.1f} fps, {self.dropped} corrupt frames dropped so far, {len(self.clients)} clients, exposure {self.locked or 'auto'}", flush=True)
                t_report, n_report = time.monotonic(), 0


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="camera broker: owns the camera, shares every frame with any number of readers")
    p.add_argument("--camera", choices=list(DEVICES), default="sonix")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--fourcc", choices=["MJPG", "YUYV"], default="MJPG",
                   help="MJPG (default) keeps the USB isochronous stream small: on the Pi 4 the 18 MB/s of YUYV 640x480 loses packets "
                        "whenever other processes load the memory bus, and the frames arrive with their rows shifted")
    a = p.parse_args()
    Broker(a.camera, a.width, a.height, a.fps, a.fourcc).run()
