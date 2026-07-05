import numpy as np

class MecanumModel:
    def __init__(self, wheel_base, track_width):
        self.wheel_base = wheel_base
        self.track_width = track_width

    def update(self, state, vx, vy, wz, dt):
        x, y, theta = state
        dx = vx * np.cos(theta) - vy * np.sin(theta)
        dy = vx * np.sin(theta) + vy * np.cos(theta)
        dtheta = wz
        return np.array([x + dx * dt, y + dy * dt, theta + dtheta * dt])