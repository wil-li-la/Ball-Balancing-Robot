# Digital twin: the robot streams every frame (ball_balancer.py --stream), this shows it in the MuJoCo viewer.
#
# The plate moves as the servos were commanded, through the servo model of the simulation, and the arms are drawn
# from the angles of the simulated servos (arms.py). The green ghost is
# where the camera sees the ball; the solid ball is the shadow of shadow.py, the simulated ball that rolls on its
# own and is put back on the ghost every `horizon` seconds. The text shows how far apart they drift.
#
# On macOS the viewer must run under mjpython:
#   mjpython twin.py                                  listen on UDP 5005, save the run to twin_logs/
#   mjpython twin.py --replay twin_logs/run.jsonl     replay a saved run (or a --log csv of ball_balancer.py)
#   mjpython twin.py --params fitted.json             use the parameters found by system_id.py

import argparse
import json
import os
import select
import socket
import time
from collections import deque

import mujoco
import mujoco.viewer
import numpy as np

from arms import add_arms, set_arms
from ball_balancer_env import HERE, DEFAULT_CAMERA_CONFIG
from shadow import Shadow
from system_id import load

GHOST_RGBA = [0.2, 0.9, 0.3, 0.45]
STATS_WINDOW = 5.0          # [s] of frames the mean error is taken over
VIEWER_HZ = 60


def display_model():
    # the model of the simulation plus a ghost ball and the arms, mocap bodies that collide with nothing
    spec = mujoco.MjSpec.from_file(os.path.join(HERE, "ball_balancer.xml"))
    add_arms(spec)
    ghost = spec.worldbody.add_body(name="seen", mocap=True, pos=[0, 0, -1])
    ghost.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.0205, 0, 0], rgba=GHOST_RGBA, contype=0, conaffinity=0)
    return spec.compile()


def frames_live(port, save_path):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", port))
    sock.setblocking(False)
    print(f"listening on UDP {port}, saving to {save_path}")
    with open(save_path, "w") as f:
        while True:
            ready, _, _ = select.select([sock], [], [], 1 / VIEWER_HZ)
            batch = []
            while ready:
                try:
                    data, _ = sock.recvfrom(4096)
                except BlockingIOError:
                    break
                f.write(data.decode() + "\n")
                batch.append(json.loads(data))
            f.flush()
            yield batch


def frames_replay(msgs, speed):
    # the frames at the pace they were recorded, gaps longer than a second shortened to one
    t0_wall, t0_run = time.perf_counter(), msgs[0]["t"]
    i = 0
    while i < len(msgs):
        now = t0_run + (time.perf_counter() - t0_wall) * speed
        batch = []
        while i < len(msgs) and msgs[i]["t"] <= now:
            batch.append(msgs[i])
            i += 1
        if i < len(msgs) and msgs[i]["t"] - now > 1.0:
            t0_wall -= (msgs[i]["t"] - now - 1.0) / speed
        yield batch
        time.sleep(1 / VIEWER_HZ)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--replay", help="a .jsonl saved by twin.py or a --log csv of ball_balancer.py")
    parser.add_argument("--speed", type=float, default=1.0, help="replay speed")
    parser.add_argument("--params", help="json of parameters, from system_id.py")
    parser.add_argument("--horizon", type=float, default=0.5, help="[s] between two resyncs of the simulated ball")
    args = parser.parse_args()

    params = None
    if args.params:
        with open(args.params) as f:
            params = json.load(f)
    shadow = Shadow(params, args.horizon)

    if args.replay:
        source = frames_replay(load(args.replay), args.speed)
        name = os.path.basename(args.replay)
    else:
        os.makedirs(os.path.join(HERE, "twin_logs"), exist_ok=True)
        save = os.path.join(HERE, "twin_logs", time.strftime("twin_%Y%m%d_%H%M%S.jsonl"))
        source = frames_live(args.port, save)
        name = f"UDP {args.port}"

    model = display_model()
    data = mujoco.MjData(model)
    ghost = model.body("seen").mocapid[0]
    errors = deque()        # (t, error) within STATS_WINDOW
    last = None             # last frame
    arrivals = deque()      # times of the frames of the last second

    with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
        for key, value in DEFAULT_CAMERA_CONFIG.items():
            setattr(viewer.cam, key, value)
        viewer.cam.lookat[2] = 0.11
        for batch in source:
            if not viewer.is_running():
                break
            for msg in batch:
                row = shadow.feed(msg)
                if row is not None:
                    errors.append((msg["t"], row["err"]))
                last = msg
                arrivals.append(msg["t"])
            while arrivals and arrivals[0] <= arrivals[-1] - 1.0:
                arrivals.popleft()
            if last is None:
                viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, "waiting", f"for frames on {name}"))
                viewer.sync()
                continue
            while errors and errors[0][0] < last["t"] - STATS_WINDOW:
                errors.popleft()

            with viewer.lock():
                data.qpos[:] = shadow.data.qpos
                data.mocap_pos[ghost] = shadow.on_plate(shadow.seen) if shadow.seen is not None else [0, 0, -1]
                set_arms(model, data, shadow.env._q, shadow.env._pose)
                mujoco.mj_forward(model, data)
            fps = len(arrivals)
            p = shadow.params
            left = "source\nframes/s\nball\nshadow\nerror now\nerror mean 5 s\nlatency / tau / deadband"
            right = "\n".join([
                name,
                f"{fps}",
                "seen" if shadow.seen is not None else "not seen",
                "synced" if shadow.synced() else "waiting for the ball",
                f"{errors[-1][1]:.1f} px" if errors and errors[-1][0] == last["t"] else "-",
                f"{np.mean([e for _, e in errors]):.1f} px over {len(errors)} frames" if errors else "-",
                f"{p['latency'] * 1000:.0f} ms / {p['servo_tau'] * 1000:.0f} ms / {p['deadband']:.1f} deg",
            ])
            viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, left, right))
            viewer.sync()
        print("viewer closed" if not viewer.is_running() else "replay finished")
        while viewer.is_running() and args.replay:      # keep the last frame on screen until the window is closed
            time.sleep(0.1)


if __name__ == "__main__":
    main()
