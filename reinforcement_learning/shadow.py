# Shadow model of the robot: the simulation of ball_balancer_env.py driven by what the robot streams.
#
# Every frame of the robot (scripts/stream.py) carries the ball it measured and the servo angles it commanded.
# The shadow applies the same commands to the simulated servos and lets the simulated ball roll on its own,
# and compares where the simulated ball is with where the camera saw the real one. The ball cannot be followed
# open loop for long, it is an unstable system and small errors grow, so every `horizon` seconds the simulated
# ball is put back where the camera sees it (resync). The error a fraction of a second after a resync measures
# how well the simulation predicts the robot, and system_id.py fits the parameters that make it smallest.
#
# Time: the robot stamps a frame when it reads it, but the image is older than that by the latency of the camera;
# the commands act from the moment they are sent. The latency of the simulation is therefore placed on the
# measurements: the frame stamped t shows the ball at t - latency.

from collections import deque

import mujoco
import numpy as np

from ball_balancer_env import BallBalancerEnv, NOMINAL, PLATE_TOP, R_BALL, plate_normal, forward_kinematics

ROLLING = 1e-4              # rolling friction of ball_balancer.xml, a parameter that can be fitted like the ones of NOMINAL
EXTRA = dict(rolling=ROLLING,
             bias_x=0.0, bias_y=0.0)    # slope of the real plate when the servos are sent to level: the offsets of hardware.py are set by eye
RESYNC_GAP = 0.2            # [s] a gap this long without the ball is a reacquisition: the shadow restarts from the measurement
N_VEL = 6                   # measurements whose straight line gives position and velocity at a resync
PARK = np.array([0.0, 0.0, -0.5])   # where the simulated ball waits, out of sight, while it is not synced
Q_HOME = 60.0               # hardware.Q_HOME: ball_balancer.py sends the servos home before the first frame


class Shadow:
    def __init__(self, params=None, horizon=0.5):
        self.env = BallBalancerEnv(randomize=False)
        self.model, self.data = self.env.model, self.env.data
        self.dt = self.model.opt.timestep
        self._geoms = [self.model.geom("ball").id, self.model.geom("plate").id]
        self._ball = self.model.body("ball").id
        set_plate = self.env._set_plate
        self.env._set_plate = lambda pose, velocity=True: set_plate(np.array(pose) + self._bias, velocity)
        self.horizon = horizon
        self.reset(params)

    def reset(self, params=None):
        self.params = {**NOMINAL, **EXTRA, **(params or {})}
        self._scale_given = "px_per_m" in (params or {})
        self._bias = np.array([self.params["bias_x"], self.params["bias_y"], 0.0])
        self.env.reset(seed=0)
        self.env.params = self.params
        self.env._apply_params()
        self.model.geom_friction[self._geoms, 2] = self.params["rolling"]
        self.t_sim = None               # time of the simulated state, in the clock of the robot
        self.t_sync = None              # time of the last resync, None while the ball is not synced
        self.t_meas = -np.inf           # time of the last measurement
        self.pending = deque()          # commands (t, q) the simulation has not reached yet
        self.meas = deque(maxlen=N_VEL)  # recent measurements (t, xy) since the ball was acquired
        self.seen = None                # last measured ball, xy [m]
        self._park()

    # CONVERSIONS: pixels of the frame, from the top left corner, <-> plate coordinates in meters. The plate center is
    # the one of the calibration of the robot when the frame carries one ("cal"), otherwise the frame center. The scale
    # is px_per_m when it is given (system_id.py fits it); otherwise the one of the calibration, or NOMINAL. The
    # calibration measures the plate, and the center of the ball is R_BALL + PLATE_TOP farther from the camera, where
    # the frame is smaller by the ratio of the two distances: px_per_m is the scale at the center of the ball
    def _frame(self, msg):
        cal = msg.get("cal")
        center = np.array([cal["cx"], cal["cy"]]) if cal else np.array(msg["size"]) / 2
        if self._scale_given or not cal:
            return center, self.params["px_per_m"]
        return center, cal["px_per_mm"] * 1000.0

    def to_xy(self, px, msg):
        center, scale = self._frame(msg)
        return (np.array(px[:2]) - center) / scale

    def to_px(self, xy, msg):
        center, scale = self._frame(msg)
        return np.array(xy) * scale + center

    def on_plate(self, xy):
        # centre of a ball resting on the plate at xy
        pose = self.env._pose
        n = plate_normal(pose)
        z = pose[2] / 1000.0 + PLATE_TOP - (n[0] * xy[0] + n[1] * xy[1]) / n[2]
        return np.array([xy[0], xy[1], z]) + (R_BALL + 1e-4) * n

    def ball_xy(self):
        return self.data.xpos[self._ball][:2].copy()

    def synced(self):
        return self.t_sync is not None

    # SIMULATION
    def _park(self):
        i, j = self.env._ball_qpos, self.env._ball_qvel
        self.data.qpos[i:i + 3] = PARK
        self.data.qpos[i + 3:i + 7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qvel[j:j + 6] = 0.0

    def _advance(self, t):
        while self.t_sim + self.dt <= t:
            while self.pending and self.pending[0][0] <= self.t_sim:
                self.env._command_q(self.pending.popleft()[1])
            self.env._servo_step()
            mujoco.mj_step(self.model, self.data)
            self.t_sim += self.dt
            if self.synced() and (self.t_sim - self.t_meas > RESYNC_GAP                 # no measurement backs it any more
                                  or not np.all(np.abs(self.ball_xy()) < 0.075)):       # rolled off the simulated plate
                self.t_sync = None
            if not self.synced():
                self._park()
        mujoco.mj_forward(self.model, self.data)

    def _resync(self, t):
        # the ball is put where the line through the last measurements says it is, with that velocity. The ball
        # accelerates down the slope of the plate, (3/5) g along it for a hollow shell (README §8), and a straight line
        # through accelerating points gives the velocity of the middle of the window, half a window late: the
        # acceleration the plate gives is taken out of the measurements before the line is fitted
        n = plate_normal(self.env._pose)
        acc = 0.6 * 9.81 * n[2] * n[:2]
        ts = np.array([m[0] for m in self.meas]) - t
        xy = np.array([m[1] for m in self.meas]) - 0.5 * np.outer(ts**2, acc)
        A = np.column_stack([np.ones_like(ts), ts])
        (p, v), *_ = np.linalg.lstsq(A, xy, rcond=None)
        vz = -(n[0] * v[0] + n[1] * v[1]) / n[2]
        i, j = self.env._ball_qpos, self.env._ball_qvel
        self.data.qpos[i:i + 3] = self.on_plate(p)
        self.data.qpos[i + 3:i + 7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qvel[j:j + 6] = [v[0], v[1], vz, -v[1] / R_BALL, v[0] / R_BALL, 0.0]     # rolling
        mujoco.mj_forward(self.model, self.data)
        self.t_sync = t

    def _start(self, t):
        # the first frame: the servos are at home, where ball_balancer.py sent them. The commands that follow go
        # through the dead band from there, as on the robot: starting from the first command instead leaves the
        # simulated plate tilted wherever that command was, whenever the next ones stay inside the dead band
        self.t_sim = t
        self.env._q_target = np.full(3, Q_HOME)
        self.env._q = self.env._q_target.copy()
        self.env._pose_target = forward_kinematics(self.env._q_target)
        self.env._set_plate(self.env._pose_target, velocity=False)

    def feed(self, msg):
        # one frame of the robot in, the comparison of measured and simulated ball out (None if there is nothing to compare)
        t_img = msg["t"] - self.params["latency"]
        if self.t_sim is None:
            self._start(t_img)
        self._advance(t_img)

        row = None
        if msg["ball"] is not None:
            xy = self.to_xy(msg["ball"], msg)
            if t_img - self.t_meas > RESYNC_GAP:
                self.meas.clear()
                self.t_sync = None
            self.meas.append((t_img, xy))
            self.t_meas, self.seen = t_img, xy
            if self.synced() and t_img - self.t_sync < self.horizon:
                sim = self.to_px(self.ball_xy(), msg)
                meas = np.array(msg["ball"][:2])
                row = {"t": msg["t"], "age": t_img - self.t_sync, "meas": meas, "sim": sim,
                       "err": float(np.linalg.norm(sim - meas))}
            elif len(self.meas) == N_VEL:
                self._resync(t_img)
        else:
            self.seen = None

        q = msg.get("q_actual") or msg["q"]       # what reached the servos, if the robot limits their rate after the IK
        if q is not None:
            self.pending.append((msg["t"], np.array(q, dtype=float)))
        return row
