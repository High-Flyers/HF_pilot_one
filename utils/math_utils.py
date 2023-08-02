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
    den = quat[0] * quat[0] + \
        quat[1] * quat[1] + \
        quat[2] * quat[2] + \
        quat[3] * quat[3]
    return np.array([
        quat[0] / den,
        -quat[1] / den,
        -quat[2] / den,
        -quat[3] / den,
    ])


def quat_mult(p, q):
    return np.array([
        p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3],
        p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2],
        p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1],
        p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0],
    ])


def rot_mat_to_quat(m):
    if m[2, 2] < 0:
        if m[0, 0] > m[1, 1]:
            t = 1 + m[0, 0] - m[1, 1] - m[2, 2]
            q = [t, m[0, 1]+m[1, 0], m[2, 0]+m[0, 2], m[1, 2]-m[2, 1]]
        else:
            t = 1 - m[0, 0] + m[1, 1] - m[2, 2]
            q = [m[0, 1]+m[1, 0], t, m[1, 2] + m[2, 1], m[2, 0] - m[0, 2]]
    else:
        if m[0, 0] < -m[1, 1]:
            t = 1 - m[0, 0] - m[1, 1] + m[2, 2]
            q = [m[2, 0] + m[0, 2], m[1, 2] + m[2, 1], t, m[0, 1] - m[1, 0]]
        else:
            t = 1 + m[0, 0] + m[1, 1] + m[2, 2]
            q = [m[1, 2]-m[2, 1], m[2, 0]-m[0, 2], m[0, 1]-m[1, 0], t]
    q = np.array(q) * (0.5 / np.sqrt(t))
    return q


def rot_mat_to_quat_old(mat):
    eigenvalues, eigenvectors = np.linalg.eig(mat)
    target = 0
    for i in range(1, 3):
        if np.abs(eigenvalues[i] - 1) < np.abs(eigenvalues[target] - 1):
            target = i
    vec = eigenvalues[target] * eigenvectors[target]
    vec = vec / np.linalg.norm(vec)
    theta = np.arccos(0.5 * (np.trace(mat) - 1))
    # w x y z
    quat = np.array([
        np.cos(theta / 2),
        np.sin(theta / 2) * vec[0],
        np.sin(theta / 2) * vec[1],
        np.sin(theta / 2) * vec[2],
    ])
    return quat


def quat_sandwich(q, v):
    '''returns q*v*q_inv'''
    return quat_mult(quat_mult(q, v), quat_inv(q))


for i in np.arange(0, np.pi, 0.1):
    drone_transform = get_rot_mat(i, i, -0.75*i)
    q_rot = rot_mat_to_quat(drone_transform)

    vec = np.array([1, 0, 0])
    vec = np.matmul(drone_transform, vec)

    quat = np.array([0, 1, 0, 0])
    quat = quat_sandwich(q_rot, quat)[1::]

    print(np.linalg.norm(vec - quat))
