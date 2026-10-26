# The robot without the robot: scripts/ball_balancer.py runs unchanged on the Mac, with the module hardware.py
# replaced by the simulation. The camera renders the "below" camera of ball_balancer.xml, the servos drive the
# simulated plate, in real time, so the detection of vision.py, the controllers and --stream are the code that
# runs on the Pi. The physical parameters are drawn at random (seed) and saved, so that system_id.py can be
# checked on a run whose true parameters are known.
#
# Needs OpenCV next to MuJoCo, which the RL environment does not have (environment_hil.yml):
#   python sim_robot.py --ball-color 0.2,0.4,1 -- --pid --stream 127.0.0.1
# everything after -- goes to ball_balancer.py. Stops with ctrl+c or after --duration seconds.

import argparse
import json
import os
import runpy
import sys
import time
import types
from collections import deque

import mujoco
import numpy as np

from ball_balancer_env import BallBalancerEnv, HERE, Q_MIN, Q_MAX, PLATE_TOP, R_BALL, plate_normal, forward_kinematics

SCRIPTS = os.path.join(HERE, "..", "scripts")      # the loop that runs, --scripts to run another copy of it
FPS = 30                    # the USB cameras of hardware.py
RESPAWN = 1.0               # [s] after the ball falls, a hand puts it back
PUSH_SPEED = (0.04, 0.10)   # [m/s] a push (--push) sets the ball rolling this fast, in a random direction
CEILING_CELL = 48           # [texels] of the grid drawn on the sky, what the camera sees through the plate
PX_PER_MM = 2.5             # scale of the frame at the centre of the ball, as the calibration of the robot sets it


class SimRobot:
    def __init__(self, seed, ball_rgb, push=None):
        self.env = BallBalancerEnv(randomize=True)
        self.env.reset(seed=seed)
        self.model, self.data = self.env.model, self.env.data
        self.dt = self.model.opt.timestep
        self.params = dict(self.env.params)
        if ball_rgb is not None:
            self.model.mat_rgba[self.model.material("ball").id, :3] = ball_rgb
        self._draw_ceiling()
        self.model.vis.headlight.active = 0     # the light comes from above: the camera sees the shaded underside of the ball, as on the robot
        self.renderer = None                                # made by start_camera, at the size the loop asks for
        self.render_data = mujoco.MjData(self.model)
        self.history = deque(maxlen=int(0.3 / self.dt))     # (t, qpos) of the last 0.3 s, the camera shows the past
        self.pending = deque()                              # commands (t, q)
        self.t0 = time.perf_counter()
        self.t_sim = 0.0
        self.next_frame = 0.0
        self.ball_on = False
        self.fell_at = None
        self.rng = np.random.default_rng(seed)
        self.push = push
        self.next_push = None
        self.slew = None                                    # servo.slew of the loop, when its copy of scripts/ has it
        self.q_sent, self.t_sent = None, None               # the last angles sent, for the slew limit
        self._park()
        self.history.append((0.0, self.data.qpos.copy()))

    def _draw_ceiling(self):
        # a ceiling of square tiles instead of the plain gradient: a background with edges is the hard case of the detection
        tex = self.model.texture("skybox").id
        w, h, adr = self.model.tex_width[tex], self.model.tex_height[tex], self.model.tex_adr[tex]
        n = self.model.tex_nchannel[tex]
        img = np.full((h, w, n), 215, np.uint8)
        y, x = np.mgrid[:h, :w]
        img[(x % CEILING_CELL < 3) | (y % CEILING_CELL < 3)] = 90
        self.model.tex_data[adr:adr + w * h * n] = img.ravel()

    # BALL
    def _park(self):
        i, j = self.env._ball_qpos, self.env._ball_qvel
        self.data.qpos[i:i + 3] = [1.0, 1.0, 0.0]          # out of the view of the camera
        self.data.qpos[i + 3:i + 7] = [1, 0, 0, 0]
        self.data.qvel[j:j + 6] = 0.0

    def place_ball(self):
        # put down by hand, near the center, almost still
        r, a = self.rng.uniform(0.0, 0.03), self.rng.uniform(0, 2 * np.pi)
        x, y = r * np.cos(a), r * np.sin(a)
        pose = self.env._pose
        n = plate_normal(pose)
        z = pose[2] / 1000.0 + PLATE_TOP - (n[0] * x + n[1] * y) / n[2]
        i, j = self.env._ball_qpos, self.env._ball_qvel
        self.data.qpos[i:i + 3] = np.array([x, y, z]) + (R_BALL + 1e-4) * n
        self.data.qpos[i + 3:i + 7] = [1, 0, 0, 0]
        self.data.qvel[j:j + 6] = 0.0
        self.ball_on, self.fell_at = True, None
        self.next_push = None if self.push is None else self.t_sim + self.push

    def _push(self):
        # a finger flicks the ball: it rolls off at PUSH_SPEED
        speed, a = self.rng.uniform(*PUSH_SPEED), self.rng.uniform(0, 2 * np.pi)
        vx, vy = speed * np.cos(a), speed * np.sin(a)
        j = self.env._ball_qvel
        self.data.qvel[j:j + 6] = [vx, vy, 0.0, -vy / R_BALL, vx / R_BALL, 0.0]
        self.next_push = self.t_sim + self.push

    # TIME
    def now(self):
        return time.perf_counter() - self.t0

    def advance(self):
        t = self.now()
        while self.t_sim + self.dt <= t:
            while self.pending and self.pending[0][0] <= self.t_sim:
                self.env._command_q(self.pending.popleft()[1])
            self.env._servo_step()
            mujoco.mj_step(self.model, self.data)
            self.t_sim += self.dt
            if self.ball_on and self.data.qpos[self.env._ball_qpos + 2] < self.env._pose[2] / 1000.0 - 0.03:
                self.ball_on, self.fell_at = False, self.t_sim       # fell off the plate
            if self.ball_on and self.next_push is not None and self.t_sim >= self.next_push:
                self._push()
            if not self.ball_on:
                self._park()
                if self.fell_at is not None and self.t_sim - self.fell_at > RESPAWN:
                    self.place_ball()
            self.history.append((self.t_sim, self.data.qpos.copy()))

    # THE MODULE hardware.py
    def start_camera(self, W, H):
        # the field of view that gives PX_PER_MM at the height of the centre of the ball on the level plate
        cam = self.model.camera("below").id
        depth = (self.env._pose[2] / 1000.0 + PLATE_TOP + R_BALL) - self.model.cam_pos0[cam][2]
        self.model.cam_fovy[cam] = np.degrees(2 * np.arctan(H / 2 / (PX_PER_MM * 1000.0 * depth)))
        self.renderer = mujoco.Renderer(self.model, H, W)
        return self

    def capture_array(self):
        # paced at FPS; the image is the state of `latency` ago
        wait = self.next_frame - self.now()
        if wait > 0:
            time.sleep(wait)
        self.next_frame = max(self.next_frame + 1 / FPS, self.now())
        self.advance()
        t_img = self.t_sim - self.params["latency"]
        qpos = next((q for t, q in reversed(self.history) if t <= t_img), self.history[0][1])
        self.render_data.qpos[:] = qpos
        mujoco.mj_forward(self.model, self.render_data)
        self.renderer.update_scene(self.render_data, camera="below")
        return self.renderer.render()[:, :, ::-1].copy()    # RGB -> BGR, like the cameras of hardware.py

    def home(self):
        # at power on the servos go to the position they are sent to, there is no previous command to be within the dead band of
        self.q_sent, self.t_sent = np.full(3, float(Q_HOME)) + OFFSETS, self.now()
        self.env._q_target = np.full(3, float(Q_HOME))
        self.env._q = self.env._q_target.copy()
        self.env._pose_target = forward_kinematics(self.env._q_target)
        self.env._set_plate(self.env._pose_target, velocity=False)

    def set_angle(self, angles):
        # returns the angles sent, offsets included, after the slew limit of servo.py when the loop has one, as hardware.set_angle does
        now = self.now()
        sent = np.array(angles, dtype=float)
        if self.slew is not None:
            sent = self.slew(sent, self.q_sent, now - self.t_sent if self.t_sent is not None else 0.0)
        self.q_sent, self.t_sent = sent, now
        self.pending.append((now, np.clip(sent - np.array(OFFSETS), Q_MIN, Q_MAX)))
        return sent


OFFSETS = [0, 0, 0]
Q_HOME = 60


def fake_hardware(robot):
    hw = types.ModuleType("hardware")
    hw.SERVO_PINS, hw.OFFSETS = [13, 18, 12], OFFSETS
    hw.Q_HOME, hw.Q_RANGE = Q_HOME, 15
    hw.Q_MIN, hw.Q_MAX = Q_MIN, Q_MAX
    hw.TARGET_FPS = FPS
    hw.UVC_CAMERAS = {}
    hw.set_angle = robot.set_angle
    hw.home = robot.home
    hw.start_camera = robot.start_camera
    hw.lock_exposure = lambda cam: "simulated camera, fixed exposure"
    hw.shut_down = lambda cam: None
    return hw


def main():
    argv = sys.argv[1:]
    ours, theirs = (argv[:argv.index("--")], argv[argv.index("--") + 1:]) if "--" in argv else (argv, ["--pid"])
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=0, help="draws the physical parameters of the simulated robot")
    parser.add_argument("--ball-color", help="r,g,b in 0..1 (default: the pink of ball_balancer.xml)")
    parser.add_argument("--duration", type=float, help="[s] stop after this long")
    parser.add_argument("--push", type=float, help="[s] between two pushes of the ball, none by default")
    parser.add_argument("--scripts", default=SCRIPTS, help="folder of the ball_balancer.py to run")
    parser.add_argument("--truth", default=os.path.join(HERE, "twin_logs", "sim_robot_params.json"), help="where the true parameters are saved")
    args = parser.parse_args(ours)

    rgb = [float(c) for c in args.ball_color.split(",")] if args.ball_color else None
    robot = SimRobot(args.seed, rgb, args.push)
    os.makedirs(os.path.dirname(args.truth), exist_ok=True)
    with open(args.truth, "w") as f:
        json.dump(robot.params, f, indent=1)
    print("simulated robot:", {k: round(v, 4) for k, v in robot.params.items()})

    sys.path.insert(0, args.scripts)
    sys.modules["hardware"] = fake_hardware(robot)
    try:
        import servo                        # what hardware.set_angle does to the angles, in the copies of the loop that have it
        robot.slew = servo.slew
        print(f"slew limit of servo.py: {servo.SLEW_DEG_PER_S:.0f} deg/s")
    except ImportError:
        print("no servo.py in", args.scripts, "- the angles are sent as commanded")
    import vision
    calibrate = vision.calibrate

    def calibrate_then_place(frames):       # the empty plate is calibrated first, then the ball goes on
        noise = calibrate(frames)
        robot.place_ball()
        return noise
    vision.calibrate = calibrate_then_place
    if "--detect" in theirs and theirs[theirs.index("--detect") + 1] == "color":
        robot.place_ball()

    if args.duration:
        capture = robot.capture_array

        def capture_until():
            if robot.now() > args.duration:
                raise KeyboardInterrupt     # what ctrl+c does to the loop: it shuts down and saves its log
            return capture()
        robot.capture_array = capture_until

    os.chdir(args.scripts)                  # control.py loads policy.npz from the working directory
    sys.argv = ["ball_balancer.py", *theirs]
    runpy.run_path("ball_balancer.py", run_name="__main__")


if __name__ == "__main__":
    main()
