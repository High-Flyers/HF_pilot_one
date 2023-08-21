import numpy as np
from numpy import sin, cos
import time


class LowPassFilter:
    def __init__(self, init_val=None, alpha=0.3):
        self.alpha = alpha
        self.prev = init_val

    def get(self, new_val):
        curr = self.alpha * new_val + (1-self.alpha) * self.prev
        self.prev = curr
        return curr


class ComplimentaryFilter:
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.last_updated = time.time()
        self.val = 0.0

    def update(self, val, val_dt):
        now = time.time()
        dt = now - self.last_updated
        self.val = self.alpha * val + \
            (1.0 - self.alpha) * (self.val + val_dt * dt)
        self.last_updated = now
        return self.val


def get_rot_mat(roll, pitch, yaw):
    a = roll
    b = pitch
    y = yaw
    return np.array([
        [cos(b)*cos(y), sin(a)*sin(b)*cos(y)-cos(a) *
         sin(y), cos(a)*sin(b)*cos(y)+sin(a)*sin(y)],
        [cos(b)*sin(y), sin(a)*sin(b)*sin(y)+cos(a) *
         cos(y), cos(a)*sin(b)*sin(y)-sin(a)*cos(y)],
        [-sin(b), sin(a)*cos(b), cos(a)*cos(b)],
    ], dtype=np.float32)


def quat_inv(quat):
    return np.array([
        quat[0],
        -quat[1],
        -quat[2],
        -quat[3],
    ])


def quat_mult(p, q):
    return np.array([
        p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3],
        p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2],
        p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1],
        p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0],
    ])


def quat_rot(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
        np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
        np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qw, qx, qy, qz]


def quat_sandwich(q, v):
    '''returns q*v*q_inv'''
    return quat_mult(quat_mult(q, v), quat_inv(q))

# drone_transform_quat = quat_rot(0, np.pi, -0.75*np.pi)
# [0.0, 0.9238795325112867, 0.38268343236508984, 0.0]

# drone_transform = get_rot_mat(0, np.pi, -0.75*np.pi)
# print(drone_transform)
# [[ 7.0710677e-01  7.0710677e-01 -8.6595606e-17]
#  [ 7.0710677e-01 -7.0710677e-01 -8.6595606e-17]
#  [-1.2246469e-16 -0.0000000e+00 -1.0000000e+00]]