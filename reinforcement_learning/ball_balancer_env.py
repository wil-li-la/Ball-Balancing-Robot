# Gymnasium environment of the ball balancing robot, simulated in MuJoCo.
#
# The action of the policy is the slope u of the plate, the same quantity that control.pid
# and control.lqr return, so what follows it is the chain that runs on the robot:
#   u -> kinematics.solve -> clip to [Q_MIN, Q_MAX] -> servos -> pose of the plate
# The arms are not simulated: the pose the angles give to the plate is found with the forward
# kinematics and imposed on the joints of the model. The observation is what the camera loop
# measures, the pixel error of the ball, delayed and noisy, with the last actions.

import os
import sys
import time
from collections import deque

import mujoco
import numpy as np
from gymnasium import spaces
from gymnasium.envs.mujoco import MujocoEnv

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import kinematics  # noqa: E402


# ACTION
U_MAX = 0.14                             # the largest for which no action of the square is clipped, see action_limits.ipynb


# ACTUATION, as in scripts/hardware.py
Q_MIN, Q_MAX = 45.0, 75.0
SERVO_SPEED = 500.0                      # [deg/s] the SG90 does 60 deg in 0.1 s without load


# LOOP
SUBSTEPS = 25                            # physics steps (2 ms) per control step (50 ms), the loop runs at 20 Hz


# PLATE
PLATE_TOP = 0.004                        # m, surface of the disc above h: half the ring plus the disc


# CAMERA, as in scripts/vision.py
W, H = 320, 240                          # size of the camera frame
R_BALL = 0.02                            # physical ball radius [m]
PX_PER_M = 50.0 / R_BALL                 # camerta ball radius [px]


# OBSERVATION
N_ERR = 6                                # last errors, the velocity is not measured
N_ACT = 3                                # last actions, they carry what the delay hides
ERR_SCALE = W / 2                        # px ([-W/2, +W/2]) -> observation for the networj ([-1, +1]), a ball at the border is about 1


# REWARD
W_DIST = 1                               # weight of the error
W_ACT = 0.1                             # weight of the action magnitude
W_DACT = 5                               # weight of the action change, the servos do not like chattering


# FORWARD KINEMATICS
FK_ITERS = 10
FK_TOL = 1e-3                            # [deg]
POSE_NOM = np.array([0.0, 0.0, kinematics.H_NOM])    # u_x, u_y, h [mm]


# KINEMATICS
def inverse_kineamtics(pose):
    return kinematics.solve(pose[:2], pose[2])

def forward_kinematics(q, pose0=POSE_NOM):
    # configuration -> pose
    # works by quasi Newton on the inverse kinematics with the Jacobian of the nominal pose
    pose = np.array(pose0, dtype=float)
    for _ in range(FK_ITERS):
        q_pose = inverse_kineamtics(pose)
        if q_pose is None:
            break
        dq = q - q_pose
        if np.max(np.abs(dq)) < FK_TOL:
            break
        pose = pose + np.matmul(J_nom_inv, dq)
    return pose

def plate_normal(pose):
    n = np.array([pose[0], pose[1], 1.0])
    return n / np.linalg.norm(n)

def plate_angles(pose):
    # joint angles of the plate, with R = Ry(ry) Rx(rx) (the plate normal starts at [0 0 1], is rotated by rx about x and by by ry about y)
    n = plate_normal(pose)
    return -np.arcsin(n[1]), np.arctan2(n[0], n[2])


# JACOBIANS
def jacobian(pose, eps=(1e-4, 1e-4, 1e-2)):
    # the jacobian defines how much the configuration (q1, q2, q3) changes as the pose (u_x, u_y, h) varies when the pose is close to a nominal one
    # J = d q / d (u_x, u_y, h) by central differences
    columns = []
    for step, e in zip(np.diag(eps), eps):
        # step = variation in the pose (e.g. [0.0001, 0, 0])
        # eps = value of the variation (e.g. 0.0001)
        columns.append((inverse_kineamtics(pose + step) - inverse_kineamtics(pose - step)) / (2 * e))
    return np.column_stack(columns)

def jacobian_inv(pose, eps=(1e-4, 1e-4, 1e-2)):
    # the jacobian inverse defines how much the pose (u_x, u_y, h) changes as the configuration (q1, q2, q3) varies when the pose is close to a nominal one
    return np.linalg.inv(jacobian(pose, eps))  # d pose / d q at the nominal pose

J_nom = jacobian(POSE_NOM)
J_nom_inv = jacobian_inv(POSE_NOM) 


# PHYSICAL PARAMETERS: the ones the transfer to the robot depends on, drawn at every reset.
# the deadband is what makes the ball oscillate around the centre instead of settling on it
NOMINAL = dict(
    deadband=2.0,                        # [deg] dead band of the servo and backlash of its gears
    latency=0.06,                        # [s] from the frame to the command of the servos (rough estimate)
    servo_tau=0.025,                     # [s] time constant of the servos (rough estimate)
    noise_px=2.0,                        # [px] std of the detected position (rough estimate)
    px_per_m=PX_PER_M,                   # the scale of the frame, R_BALL_PX is measured roughly
)
RANDOM = dict(
    deadband=(1.5, 2.5),
    latency=(0.04, 0.10),
    servo_tau=(0.015, 0.05),
    noise_px=(0.5, 2.0),
    px_per_m=(0.9 * PX_PER_M, 1.1 * PX_PER_M),
)


# ENVIRONMENT VISUALIZATION CAMERA SETTINGS
DEFAULT_CAMERA_CONFIG = dict(distance=0.45, azimuth=135.0, elevation=-25.0, lookat=np.array([0.0, 0.0, 0.04]))


# ENVIRONMENT CLASS
class BallBalancerEnv(MujocoEnv):
    metadata = {"render_modes": ["human", "rgb_array", "depth_array"], "render_fps": 20}

    def __init__(self, randomize=True, max_episode_steps=400, render_mode=None, width=960, height=720, camera_id=None, camera_name=None):
        self.randomize = randomize
        self.max_episode_steps = max_episode_steps      # 400 steps, 20 s
        free_camera = camera_id is None and camera_name is None
        super().__init__(
            os.path.join(HERE, "ball_balancer.xml"),
            frame_skip=SUBSTEPS,
            observation_space=spaces.Box(-np.inf, np.inf, shape=(2 * N_ERR + 2 * N_ACT,), dtype=np.float32),
            render_mode=render_mode,
            width=width,
            height=height,
            camera_id=camera_id,
            camera_name=camera_name,
            default_camera_config=DEFAULT_CAMERA_CONFIG if free_camera else None,
        )
        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)

        self._ball_qpos = self.model.jnt_qposadr[self.model.joint("ball").id]
        self._ball_qvel = self.model.jnt_dofadr[self.model.joint("ball").id]
        self._px_buffer = deque(maxlen=int(RANDOM["latency"][1] / self.model.opt.timestep) + 2)

    
    # PARAMETERS
    def _sample_params(self):
        if not self.randomize:
            return dict(NOMINAL)
        return {name: self.np_random.uniform(lo, hi) for name, (lo, hi) in RANDOM.items()}

    def _apply_params(self):
        p = self.params
        self._alpha = 1 - np.exp(-self.model.opt.timestep / p["servo_tau"])
        self._delay = round(p["latency"] / self.model.opt.timestep)

    
    # ACTUATION
    def _command(self, action):
        # the chain of ball_balancer.py: slope -> inverse kinematics -> clip
        q = kinematics.solve(action * U_MAX, kinematics.H_NOM)
        if q is None:       # unreachable: as on the robot, the servos keep the previous command
            return
        self._command_q(np.clip(q, Q_MIN, Q_MAX))

    def _command_q(self, q):
        # servo angles in, as the robot sends them (twin.py): each servo moves only past its dead band
        moved = np.abs(q - self._q_target) >= self.params["deadband"]
        self._q_target = np.where(moved, q, self._q_target)
        self._pose_target = forward_kinematics(self._q_target, self._pose_target)

    def _servo_step(self):
        # first order lag with a rate limit, per servo. Close to the target the pose is linear in q,
        # so the exact forward kinematics is solved once per control step
        max_step = SERVO_SPEED * self.model.opt.timestep
        self._q = self._q + np.clip((self._q_target - self._q) * self._alpha, -max_step, max_step)
        self._set_plate(self._pose_target + J_nom_inv @ (self._q - self._q_target))

    def _set_plate(self, pose, velocity=True):
        rx, ry = plate_angles(pose)
        qpos = np.array([pose[2] / 1000.0, ry, rx])
        # the velocity is the change of the imposed pose, not of data.qpos: mj_step has already moved data.qpos by the
        # previous velocity, and the difference with it gives v_k = dq_k/dt - v_(k-1), a velocity that rings at +-v forever
        if velocity:
            self.data.qvel[:3] = (qpos - self._plate_qpos) / self.model.opt.timestep
        else:
            self.data.qvel[:3] = 0.0
        self.data.qpos[:3] = qpos
        self._plate_qpos = qpos
        self._pose = pose

    
    # SENSING
    def _ball_px(self):
        # position of the ball in the frame, from its centre
        xy = self.data.qpos[self._ball_qpos:self._ball_qpos + 2]
        return self.params["px_per_m"] * xy

    def _in_view(self, px):
        return abs(px[0]) < W / 2 and abs(px[1]) < H / 2

    def _measure(self):
        # error = frame centre - ball, on the frame taken `latency` ago
        delay = min(self._delay, len(self._px_buffer) - 1)
        px = self._px_buffer[-1 - delay]
        return -px + self.np_random.normal(0.0, self.params["noise_px"], size=2)

    def _get_obs(self):
        errors = self._err_hist.ravel() / ERR_SCALE
        return np.concatenate([errors, self._act_hist.ravel()]).astype(np.float32)

    
    # GYMNASIUM
    def reset_model(self):
        self.params = self._sample_params()
        self._apply_params()

        # servos at the command of the level plate
        self._q_target = np.clip(kinematics.solve([0.0, 0.0], kinematics.H_NOM), Q_MIN, Q_MAX)
        self._q = self._q_target.copy()
        self._pose_target = forward_kinematics(self._q_target)
        self._set_plate(self._pose_target, velocity=False)

        # ball on the plate somewhere in view, possibly already rolling
        radius = self.np_random.uniform(0.0, 0.03)
        angle = self.np_random.uniform(0.0, 2 * np.pi)
        x, y = radius * np.cos(angle), radius * np.sin(angle)
        n = plate_normal(self._pose)
        z = self._pose[2] / 1000.0 + PLATE_TOP - (n[0] * x + n[1] * y) / n[2]
        vx, vy = self.np_random.uniform(-0.05, 0.05, size=2)

        i, j = self._ball_qpos, self._ball_qvel
        self.data.qpos[i:i + 3] = np.array([x, y, z]) + (R_BALL + 1e-4) * n
        self.data.qpos[i + 3:i + 7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qvel[j:j + 6] = [vx, vy, 0.0, -vy / R_BALL, vx / R_BALL, 0.0]    # rolling
        mujoco.mj_forward(self.model, self.data)

        self._px_buffer.clear()
        self._px_buffer.append(self._ball_px())
        self._err_hist = np.tile(self._measure(), (N_ERR, 1))    # as control.reset on reacquisition
        self._act_hist = np.zeros((N_ACT, 2))
        self._steps = 0
        self._wall = time.perf_counter()
        return self._get_obs()

    def _render_real_time(self):
        # the viewer of gymnasium paces itself on the physics step while render is called once per
        # control step, 25 of them: without this the episode plays about 25 times faster than reality
        self.render()
        self.mujoco_renderer.viewer._render_every_frame = True
        late = time.perf_counter() - self._wall
        if late < self.dt:
            time.sleep(self.dt - late)
        self._wall = time.perf_counter()

    def _get_reset_info(self):
        return {"err": self._err_hist[0].copy(), "dt": self.dt, "params": self.params}

    def step(self, action):

        # ACTION
        action = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        self._command(action)

        # TIME ADVANCE
        for _ in range(SUBSTEPS):
            self._servo_step()
            mujoco.mj_step(self.model, self.data)
            self._px_buffer.append(self._ball_px())
        self._steps += 1

        # HISTORY ROLL
        px = self._px_buffer[-1]
        action_prev = self._act_hist[0].copy()
        self._err_hist = np.roll(self._err_hist, 1, axis=0)
        self._err_hist[0] = self._measure()
        self._act_hist = np.roll(self._act_hist, 1, axis=0)
        self._act_hist[0] = action

        # REWARD
        distance = np.linalg.norm(px) / (H / 2)
        chatter = np.sum((action - action_prev)**2)
        reward = 1.0 - W_DIST * distance - W_ACT * np.sum(action**2) - W_DACT * chatter

        # EPISODE END
        terminated = not (np.all(np.isfinite(self.data.qpos)) and self._in_view(px))
        truncated = self._steps >= self.max_episode_steps
        info = {"err": self._err_hist[0].copy(), "ball_px": px.copy(), "dt": self.dt, "q": self._q.copy(), "pose": self._pose.copy()}

        if self.render_mode == "human":
            self._render_real_time()
        return self._get_obs(), float(reward), terminated, truncated, info
