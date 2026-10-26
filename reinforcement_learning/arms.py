# The three arms, for display only. The simulation drives the plate through its joints and leaves the arms out
# (ball_balancer.xml); here they are drawn back from the servo angles and the pose of the plate, as capsules of the
# lengths of kinematics.py on mocap bodies that collide with nothing.
#
# Each arm stays in its vertical plane (README §5.3). The lower link turns about the shaft of its servo by q, from the
# vertical, towards the side where the inverse kinematics puts the elbow; the ball joint is the point of the plate
# circle that lies in the plane of the arm. When the plate pose is the forward kinematics of q, the upper link spans
# exactly L1 between them.

import mujoco
import numpy as np

from ball_balancer_env import kinematics, plate_normal

K = kinematics
SHOULDERS = [K.M1, K.M2, K.M3]                      # [mm]
ANGLES = np.radians([180.0, 300.0, 60.0])           # direction of each arm from the center, README §2
RADIUS = 0.004                                      # [m] of the capsules
RGBA = [0.92, 0.92, 0.9, 1.0]                       # the printed parts of ball_balancer.xml


def ball_joints(pose):
    # [mm] the ball joints for the plate pose (u_x, u_y, h)
    n = plate_normal(pose)
    center = np.array([0.0, 0.0, pose[2]])
    joints = []
    for a in ANGLES:
        d = np.cross(n, [-np.sin(a), np.cos(a), 0.0])     # the plate circle meets the plane of the arm along n x (its normal)
        d /= np.linalg.norm(d)
        if d[:2] @ [np.cos(a), np.sin(a)] < 0:
            d = -d
        joints.append(center + K.R_plane * d)
    return joints


def _elbow_directions():
    # horizontal direction from each shoulder to its elbow, from the inverse kinematics at the level pose
    B = ball_joints([0.0, 0.0, K.H_NOM])
    E = [(0.0, -1), (-np.sqrt(3), +1), (np.sqrt(3), +1)]     # plane and root of solve_P, as in kinematics.solve
    dirs = []
    for M, b, (e, sign) in zip(SHOULDERS, B, E):
        h = (K.solve_P(M, b, e, sign) - M)[:2]
        dirs.append(h / np.linalg.norm(h))
    return dirs


DIRECTIONS = _elbow_directions()


def elbows(q):
    # [mm] the elbows for the servo angles q [deg], measured from the vertical as kinematics.solve returns them
    s, c = np.sin(np.radians(q)), np.cos(np.radians(q))
    return [M + K.L2 * np.array([si * d[0], si * d[1], ci]) for M, d, si, ci in zip(SHOULDERS, DIRECTIONS, s, c)]


def add_arms(spec):
    # six mocap capsules, lower and upper link of each arm, to the spec of a display model
    for i in range(3):
        for name, length in (("lower", K.L2), ("upper", K.L1)):
            body = spec.worldbody.add_body(name=f"arm{i + 1}_{name}", mocap=True)
            body.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[RADIUS, length / 2000.0, 0],
                          rgba=RGBA, contype=0, conaffinity=0)


def set_arms(model, data, q, pose):
    # place the capsules for the servo angles q [deg] and the plate pose (u_x, u_y, h [mm])
    P, B = elbows(q), ball_joints(pose)
    quat = np.zeros(4)
    for i in range(3):
        for name, a, b in (("lower", SHOULDERS[i], P[i]), ("upper", P[i], B[i])):
            mocap = model.body(f"arm{i + 1}_{name}").mocapid[0]
            axis = (b - a) / np.linalg.norm(b - a)
            mujoco.mju_quatZ2Vec(quat, axis)
            data.mocap_pos[mocap] = (a + b) / 2000.0
            data.mocap_quat[mocap] = quat
