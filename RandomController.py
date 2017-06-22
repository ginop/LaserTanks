import numpy as np
from math import pi
from LaserTankController import LaserTankController


class RandomController(LaserTankController):

    def __init__(self):
        self.alpha = 1/30
        self.drive = np.array([0., 0.])
        self.aim_pid = PID(1.0, 0.01, 0.1)

    def main(self, t, Tank, me, them):

        if len(them) == 0:
            them = [{"position": np.array([20., 20.])}]

        them = them[0]  # only concerned with one target

        self.drive *= 1 - self.alpha
        self.drive += self.alpha * np.random.uniform(-0.2, 1.0, (2))

        # Use a PID controller to keep the turret focused on the enemy
        vec = them['position'] - me['position']
        angle = np.arctan2(vec[1], vec[0])*180/pi - me['orientation'][0]
        # Determine difference between aim and desired aim for PID controller
        d_angle = angle - me['orientation'][1]
        d_angle = (d_angle + 180) % 360 - 180  # in +-180
        turret_spin = self.aim_pid.step(d_angle)

        return np.tanh(self.drive), np.tanh(turret_spin), False


class PID():
    # PID controller that drives x to zero
    def __init__(self, P, I, D):
        self.P = P
        self.I = I
        self.D = D
        self.ix = 0
        self.lastx = 0

    def step(self, x):
        self.ix += x
        dx = x - self.lastx
        self.lastx = x
        return self.P * x + self.I * self.ix + self.D * dx

    def reset(self):
        self.ix = 0
        self.lastx = 0
