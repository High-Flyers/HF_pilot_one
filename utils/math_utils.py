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
    a = yaw
    b = pitch
    y = roll
    return np.array([
        [cos(a)*cos(b), cos(a)*sin(b)*sin(y)-sin(a) *
         cos(y), cos(a)*sin(b)*cos(y)+sin(a)*sin(y)],
        [sin(a)*cos(b), sin(a)*sin(b)*sin(y)+cos(a) *
         cos(y), sin(a)*sin(b)*cos(y)-cos(a)*sin(y)],
        [-sin(b), cos(b)*sin(y), cos(b)*cos(y)]
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
        p[0]*q[0]-q[1]*q[1]-p[2]*q[2]-p[3]*q[3],
        p[0]*q[1]+p[1]*q[0]+p[2]*q[3]-p[3]*q[2],
        p[0]*q[2]-p[1]*q[3]+p[2]*q[0]+p[3]*q[1],
        p[0]*q[3]+p[1]*q[2]-p[2]*q[1]+p[3]*q[0],
    ])


drone_transform = get_rot_mat(0, np.pi, -0.75*np.pi)
eigenvalues, eigenvectors = np.linalg.eig(drone_transform)

pointing_vec = (eigenvalues[0] * eigenvectors[0]) / \
    np.linalg.norm(eigenvalues[0] * eigenvectors[0])

theta = np.arccos(0.5 * (np.trace(drone_transform) - 1))
# w x y z
quat = np.array([
    np.cos(theta / 2),
    np.sin(theta / 2) * pointing_vec[0],
    np.sin(theta / 2) * pointing_vec[1],
    np.sin(theta / 2) * pointing_vec[2],
])

rot_vec1 = np.array([1, 0, 0])
rot_q1 = np.array([0, 1, 0, 0])

rot_vec2 = np.matmul(drone_transform, rot_vec1)
rot_q2 = quat_mult(quat_mult(quat, rot_q1), quat_inv(quat))

print(rot_vec2, rot_q2)
print(quat_mult(rot_q1, quat_inv(rot_q1)))
