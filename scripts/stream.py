# Sends what the loop sees and does at every frame to the digital twin (reinforcement_learning/twin.py),
# one UDP datagram of JSON per frame. UDP because a late frame is worth nothing to the twin, and the loop
# must never wait on the network.

import json
import socket

PORT = 5005                 # default port of twin.py


class Stream:
    def __init__(self, target, W, H, cal=None, px_per_mm=2.5):
        # target: HOST[:PORT], or several separated by commas (the dashboard on the Pi and the twin on the Mac)
        self.addrs = []
        for t in target.split(","):
            host, _, port = t.strip().partition(":")
            ip = socket.getaddrinfo(host, None, socket.AF_INET)[0][4][0]     # resolved once: a lookup per frame would stall the loop
            self.addrs.append((ip, int(port) if port else PORT))
        # cal: hardware.UvcCamera.cal, the plate center and scale in the BROKER's frame. The vision frame is cropped and
        # resized from it so that the plate center is its center and the scale is vision.PX_PER_MM, by construction;
        # "cal" says so in the frame "ball" and "size" are in, "cal_broker" keeps the measured values
        self.cal = None if cal is None else {"cx": W / 2, "cy": H / 2, "px_per_mm": px_per_mm}
        self.cal_broker = None if cal is None else {"cx": cal["cx"], "cy": cal["cy"], "px_per_mm": cal["px_per_mm"]}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.size = [W, H]
        self.seq = 0
        self.failed = 0

    def send(self, t, ball, u, q, q_actual=None, t_capture=None):
        # t: time the frame was read [s, time.monotonic of the Pi]
        # ball: (cx, cy, r) in pixels or None, u: slope applied or None, q: servo angles commanded (no offsets) or None
        # q_actual: the angles the servos were actually sent after the slew limit (same convention as q) or None
        # t_capture: start of exposure of the frame [s, same clock] when the camera reports it
        # cal: the calibration of the camera, (cx, cy) of the plate center and px/mm in the broker's frame, or None
        msg = {"seq": self.seq, "t": t, "size": self.size,
               "ball": None if ball is None else [round(float(v), 2) for v in ball],
               "u": None if u is None else [float(v) for v in u],
               "q": None if q is None else [round(float(v), 3) for v in q],
               "q_actual": None if q_actual is None else [round(float(v), 3) for v in q_actual],
               "t_capture": t_capture, "cal": self.cal, "cal_broker": self.cal_broker}
        self.seq += 1
        data = json.dumps(msg).encode()
        for addr in self.addrs:
            try:
                self.sock.sendto(data, addr)
            except OSError as e:        # the viewers must not stop the robot, but a failure is reported once
                self.failed += 1
                if self.failed == 1:
                    print(f"\nstream to {addr[0]}:{addr[1]} failed: {e}")
