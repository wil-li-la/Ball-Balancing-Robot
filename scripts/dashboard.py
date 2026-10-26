#!/usr/bin/env python3
# Live dashboard of the robot, served on 127.0.0.1:8000 (reach it from the Mac with
# ssh -N -L 8000:127.0.0.1:8000 xiangen@pi4.local, then http://localhost:8000/).
#
# Frames come from the camera broker (camd.py), so the dashboard runs with or without ball_balancer.py:
# - without it, the page shows the camera, the calibration and a preview of the colour detection
# - with it, ball_balancer.py --stream sends one JSON message per frame (stream.py) to 127.0.0.1:5006, and the page
#   shows what the loop measured and commanded: ball, error, slope u, servo angles, with plots of the last seconds
# The page can record those messages to scripts/runs/<time>.jsonl, the format reinforcement_learning/system_id.py reads.
#
#   python3 dashboard.py [--camera sonix] [--port 8000]

import argparse
import collections
import json
import os
import socket
import socketserver
import threading
import time
from http import server

import cv2
import numpy as np

import calibrate
import hardware
import vision

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(HERE, "dashboard.html")
RUNS = os.path.join(HERE, "runs")
STATE_PORT = 5006                 # where ball_balancer.py --stream 127.0.0.1:5006 sends its messages
LOOP_TIMEOUT = 0.5                # [s] without a message the loop counts as stopped
HISTORY = 20 * 30                 # messages kept for the statistics, 20 s at 30 fps
VIDEO_FPS = 20                    # of the overlay stream to the browser
FULL_EVERY = 3                    # the full camera view is sent every FULL_EVERY frames of the overlay stream


class Latest:
    # one slot the producer overwrites and consumers wait on
    def __init__(self):
        self.value, self.n = None, 0
        self.cond = threading.Condition()

    def put(self, value):
        with self.cond:
            self.value, self.n = value, self.n + 1
            self.cond.notify_all()

    def wait(self, n_seen):
        with self.cond:
            self.cond.wait_for(lambda: self.n > n_seen, timeout=1.0)
            return self.value, self.n


video = Latest()      # jpeg of the vision frame with the overlay
full = Latest()       # jpeg of the whole camera frame with the calibration drawn


class State:
    # what ball_balancer.py streams, plus the statistics derived from it
    def __init__(self):
        self.lock = threading.Lock()
        self.msg = None
        self.t_recv = 0.0
        self.hist = collections.deque(maxlen=HISTORY)
        self.q_prev = None
        self.record = None            # open .jsonl file while recording
        self.record_name = None
        self.record_n = 0

    def listen(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", STATE_PORT))
        while True:
            data, _ = s.recvfrom(65536)
            try:
                msg = json.loads(data)
            except ValueError:
                continue
            with self.lock:
                q = msg.get("q_actual") or msg.get("q")
                msg["dq"] = None if q is None or self.q_prev is None else round(float(np.max(np.abs(np.array(q) - self.q_prev))), 2)
                self.q_prev = None if q is None else np.array(q, dtype=float)
                self.msg, self.t_recv = msg, time.monotonic()
                self.hist.append(msg)
                if self.record is not None:
                    self.record.write(json.dumps(msg) + "\n")
                    self.record_n += 1

    def running(self):
        return time.monotonic() - self.t_recv < LOOP_TIMEOUT

    def snapshot(self):
        with self.lock:
            msgs = list(self.hist)
            msg = self.msg
        out = {"running": self.running(), "msg": msg if self.running() else None,
               "recording": None if self.record is None else {"name": self.record_name, "n": self.record_n}}
        if len(msgs) >= 2:
            t = np.array([m["t"] for m in msgs])
            recent = t > t[-1] - 2.0
            dt = np.diff(t[recent])
            out["fps_loop"] = round(float(1 / np.median(dt)), 1) if len(dt) and np.median(dt) > 0 else None
            out["det_rate"] = round(float(np.mean([m["ball"] is not None for m in msgs if m["t"] > t[-1] - 2.0])), 2)
            dq = [m["dq"] for m in msgs if m.get("dq") is not None and m["t"] > t[-1] - 5.0]
            out["dq_p50"] = round(float(np.median(dq)), 2) if dq else None
            out["dq_p90"] = round(float(np.percentile(dq, 90)), 2) if dq else None
        return out

    def start_record(self):
        with self.lock:
            if self.record is not None:
                return self.record_name
            os.makedirs(RUNS, exist_ok=True)
            self.record_name = time.strftime("run_%Y%m%d_%H%M%S.jsonl")
            self.record = open(os.path.join(RUNS, self.record_name), "w")
            self.record_n = 0
            return self.record_name

    def stop_record(self):
        with self.lock:
            if self.record is None:
                return None
            self.record.close()
            name, n = self.record_name, self.record_n
            self.record, self.record_name = None, None
            return {"name": name, "n": n}


state = State()
cam_stats = {"fps": 0.0, "age_ms": 0.0, "seq": 0, "exposure": None, "dropped": None}


def draw_overlay(frame, ball, u, label, color):
    cv2.drawMarker(frame, vision.center, (0, 255, 255), cv2.MARKER_CROSS, 12, 1)
    if ball is not None:
        cx, cy, r = ball
        cv2.circle(frame, (int(cx), int(cy)), int(r), color, 2)
        cv2.circle(frame, (int(cx), int(cy)), 3, color, -1)
        cv2.arrowedLine(frame, vision.center, (int(cx), int(cy)), (255, 0, 0), 2, tipLength=0.15)
        if u is not None:
            tip = (int(vision.center[0] + u[0] * 500), int(vision.center[1] + u[1] * 500))
            cv2.arrowedLine(frame, vision.center, tip, (0, 255, 255), 2, tipLength=0.15)
    cv2.putText(frame, label, (4, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)


def frame_loop(camera):
    cam = hardware.UvcCamera(camera, vision.W, vision.H)
    cal = calibrate.load()
    t_fps, seq_fps, k = time.monotonic(), None, 0
    while True:
        frame = cam.capture_array()
        now = time.monotonic()
        if seq_fps is None:
            seq_fps = cam.seq
        if now - t_fps >= 1.0:                 # the broker's rate, from its sequence numbers: this loop runs slower on purpose
            cam_stats["fps"] = round((cam.seq - seq_fps) / (now - t_fps), 1)
            t_fps, seq_fps = now, cam.seq
            status = cam.cam.ctl("status").split()              # "... seq N dropped M clients K exposure E"
            cam_stats["dropped"] = int(status[status.index("dropped") + 1])
            cam_stats["exposure"] = status[status.index("exposure") + 1]
        msg = state.msg if state.running() else None
        if msg is not None and msg.get("t_capture"):      # the age the control loop saw, exposure start to frame read
            cam_stats["age_ms"] = round((msg["t"] - msg["t_capture"]) * 1000, 1)
        else:
            cam_stats["age_ms"] = round((now - cam.t_capture) * 1000, 1)
        cam_stats["seq"] = cam.seq

        # the colour detection runs here only for the preview and the mask inset; the loop's own result wins when it runs
        preview, mask = vision.detect(frame, "color")
        if msg is not None:
            ball = msg["ball"]
            u = msg["u"]
            label = f"loop  err {vision.center[0] - ball[0]:+.0f} {vision.center[1] - ball[1]:+.0f} px" if ball else "loop  no ball"
            draw_overlay(frame, ball, u, label, (0, 255, 0))
        else:
            label = "preview (colour)  " + (f"r {preview[2]:.0f} px" if preview else "no ball")
            draw_overlay(frame, preview, None, label, (0, 200, 255))
        inset = cv2.resize(mask, (88, 88))
        frame[vision.H - 88:, vision.W - 88:] = cv2.cvtColor(inset, cv2.COLOR_GRAY2BGR)
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            video.put(jpg.tobytes())

        k += 1
        if k % FULL_EVERY == 0:
            raw = cv2.resize(cam.last_raw, (cam.last_raw.shape[1] // 2, cam.last_raw.shape[0] // 2))
            if cal is not None:
                c = (int(cal["cx"] / 2), int(cal["cy"] / 2))
                cv2.circle(raw, c, int(cal["r_px"] / 2), (255, 200, 0), 1)
                rot = np.radians(cal.get("rot_deg", 0.0))              # mechanism +x (red) and +y (green) as seen by the camera
                for ang, col, name in ((rot, (0, 0, 255), "x"), (rot + np.pi / 2, (0, 255, 0), "y")):
                    tip = (int(c[0] + 60 * np.cos(ang)), int(c[1] + 60 * np.sin(ang)))
                    cv2.arrowedLine(raw, c, tip, col, 2, tipLength=0.2)
                    cv2.putText(raw, name, (tip[0] + 4, tip[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
            if cam.crop is not None:
                x0, y0, w, h = [int(v / 2) for v in cam.crop]
                cv2.rectangle(raw, (x0, y0), (x0 + w, y0 + h), (0, 255, 0), 1)
            ok, jpg = cv2.imencode(".jpg", raw, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                full.put(jpg.tobytes())
        # pace the browser stream: the camera runs at 30 fps, the page gets VIDEO_FPS
        time.sleep(max(0.0, 1.0 / VIDEO_FPS - (time.monotonic() - now)))


class Handler(server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def send_bytes(self, body, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)

    def mjpeg(self, slot):
        self.send_response(200)
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.end_headers()
        n = 0
        try:
            while True:
                jpg, n = slot.wait(n)
                if jpg is None:
                    continue
                self.wfile.write(b"--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpg))
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass

    def events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        cal = calibrate.load()
        try:
            while True:
                snap = state.snapshot()
                snap["cam"] = dict(cam_stats)
                snap["cal"] = cal
                snap["t"] = time.monotonic()
                self.wfile.write(b"data: " + json.dumps(snap).encode() + b"\n\n")
                self.wfile.flush()
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            with open(PAGE, "rb") as f:
                self.send_bytes(f.read(), "text/html; charset=utf-8")
        elif self.path == "/video.mjpg":
            self.mjpeg(video)
        elif self.path == "/full.mjpg":
            self.mjpeg(full)
        elif self.path == "/events":
            self.events()
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == "/record/start":
            self.send_bytes(json.dumps({"name": state.start_record()}).encode(), "application/json")
        elif self.path == "/record/stop":
            self.send_bytes(json.dumps(state.stop_record()).encode(), "application/json")
        else:
            self.send_error(404)


class Server(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--camera", default="sonix")
    p.add_argument("--port", type=int, default=8000)
    a = p.parse_args()
    threading.Thread(target=state.listen, daemon=True).start()
    threading.Thread(target=frame_loop, args=(a.camera,), daemon=True).start()
    print(f"dashboard on http://127.0.0.1:{a.port}/  (state from udp 127.0.0.1:{STATE_PORT})", flush=True)
    Server(("127.0.0.1", a.port), Handler).serve_forever()
