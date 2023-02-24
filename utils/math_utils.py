import numpy as np
from numpy import sin, cos
import time

class LowPassFilter:
    def __init__(self, init_val=None, alpha = 0.3):
        self.alpha = alpha
        self.prev = init_val

    def get(self, new_val):
        curr = self.alpha * new_val + (1-self.alpha) * self.prev
        self.prev = curr
        return curr

class ComplimentaryFilter:
    def __init__(self, alpha = 0.3):
        self.alpha = alpha
        self.last_updated = time.time()
        self.val = 0.0

    def update(self, val, val_dt):
        now = time.time()
        dt = now - self.last_updated
        self.val = self.alpha * val + (1.0 - self.alpha) * (self.val + val_dt * dt)
        self.last_updated = now
        return self.val


def get_rot_mat(roll, pitch, yaw):
    a = yaw
    b = pitch
    y = roll
    return np.array([
        [cos(a)*cos(b), cos(a)*sin(b)*sin(y)-sin(a)*cos(y), cos(a)*sin(b)*cos(y)+sin(a)*sin(y)],
        [sin(a)*cos(b), sin(a)*sin(b)*sin(y)+cos(a)*cos(y), sin(a)*sin(b)*cos(y)-cos(a)*sin(y)],
        [-sin(b), cos(b)*sin(y), cos(b)*cos(y)]
    ], dtype=np.float32)