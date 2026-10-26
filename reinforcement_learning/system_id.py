# System identification: the parameters of the simulation that predict a recorded run of the robot best.
#
# A run is the sequence of frames the robot streamed (twin.py saves them as .jsonl), or a --log csv of
# ball_balancer.py, which has the error and the slope of every frame with the ball. The shadow of shadow.py
# replays it, and the cost of a set of parameters is the mean distance in pixels between the simulated and the
# measured ball within `horizon` seconds of each resync: the error of the prediction the simulation makes.
#
#   python system_id.py run.jsonl [--keys latency servo_tau deadband] [--horizon 0.5]

import argparse
import json
import os
import sys

import numpy as np
from scipy.optimize import minimize

from ball_balancer_env import NOMINAL, Q_MIN, Q_MAX, W, H
from shadow import Shadow, EXTRA

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import kinematics  # noqa: E402

R_BALL_PX = 50.0            # vision.R_BALL_PX, the csv logs do not record the radius

# BOUNDS of the parameters that can be fitted, from the ranges of RANDOM widened
BOUNDS = dict(
    latency=(0.0, 0.20),
    servo_tau=(0.0005, 0.15),
    deadband=(0.0, 4.0),
    px_per_m=(1500.0, 3000.0),       # at the center of the ball, which is farther from the camera than the plate the calibration measures
    rolling=(0.0, 2e-3),
    bias_x=(-0.08, 0.08),           # the plate of this build is ~3 deg off level at home (README §16.4)
    bias_y=(-0.08, 0.08),
)


def load(path):
    # a run as the list of the messages of scripts/stream.py
    if path.endswith(".jsonl"):
        with open(path) as f:
            return [json.loads(line) for line in f if line.strip()]
    data = np.genfromtxt(path, delimiter=",", names=True)      # t,ex,ey,ux,uy of ball_balancer.py --log
    msgs = []
    for row in np.atleast_1d(data):
        u = np.array([row["ux"], row["uy"]])
        q = kinematics.solve(u, kinematics.H_NOM)
        msgs.append({"t": float(row["t"]), "size": [W, H],
                     "ball": [W // 2 - row["ex"], H // 2 - row["ey"], R_BALL_PX],
                     "u": u.tolist(), "q": None if q is None else np.clip(q, Q_MIN, Q_MAX).tolist()})
    return msgs


def replay(msgs, params=None, horizon=0.5, shadow=None):
    # the rows of the comparison of measured and simulated ball over a whole run
    shadow = shadow or Shadow(params, horizon)
    shadow.horizon = horizon
    shadow.reset(params)
    return [r for r in map(shadow.feed, msgs) if r is not None]


def cost(rows):
    return float(np.mean([r["err"] for r in rows])) if rows else np.inf


def fit(msgs, keys=("latency", "servo_tau", "deadband"), horizon=0.5, start=None, maxiter=150):
    # Nelder-Mead on parameters scaled to their bounds: the cost is not smooth (the dead band, the 2 ms step
    # the latency is rounded to), which rules out gradients. Starts from NOMINAL, or from `start`
    shadow = Shadow(horizon=horizon)
    base = {**NOMINAL, **EXTRA, **(start or {})}
    lo = np.array([BOUNDS[k][0] for k in keys])
    hi = np.array([BOUNDS[k][1] for k in keys])

    def params_of(z):
        return {**base, **dict(zip(keys, lo + np.clip(z, 0, 1) * (hi - lo)))}

    def f(z):
        return cost(replay(msgs, params_of(z), horizon, shadow))

    z0 = (np.array([base[k] for k in keys]) - lo) / (hi - lo)
    simplex = np.vstack([z0] + [z0 + 0.15 * np.eye(len(keys))[i] * (1 if z0[i] < 0.5 else -1) for i in range(len(keys))])
    res = minimize(f, z0, method="Nelder-Mead", options=dict(maxiter=maxiter, initial_simplex=simplex, xatol=1e-3, fatol=1e-3))
    best = params_of(res.x)
    return {k: float(best[k]) for k in keys}, float(res.fun), f(z0)


def by_age(rows, edges):
    # mean error as a function of the time since the resync: how fast the prediction diverges
    age = np.array([r["age"] for r in rows])
    err = np.array([r["err"] for r in rows])
    return [float(err[(age >= a) & (age < b)].mean()) if np.any((age >= a) & (age < b)) else np.nan
            for a, b in zip(edges[:-1], edges[1:])]


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("run", help="a .jsonl saved by twin.py or a --log csv of ball_balancer.py")
    parser.add_argument("--keys", nargs="+", default=["latency", "servo_tau", "deadband"], choices=list(BOUNDS))
    parser.add_argument("--horizon", type=float, default=0.5)
    args = parser.parse_args()

    msgs = load(args.run)
    params, after, before = fit(msgs, args.keys, args.horizon)
    print(f"{len(msgs)} frames, mean prediction error within {args.horizon} s of a resync")
    print(f"  NOMINAL: {before:.2f} px")
    print(f"  fitted:  {after:.2f} px")
    for k, v in params.items():
        print(f"  {k:10s} {({**NOMINAL, **EXTRA})[k]:10.4g} -> {v:.4g}")
