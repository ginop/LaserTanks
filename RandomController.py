import numpy as np
from math import pi
from LaserTankController import LaserTankController


class RandomController(LaserTankController):

    def __init__(self):
        self.alpha = 1/50
        self.drive = np.array([0., 0.])

    def main(self, t, Tank, me, them):
        self.drive *= 1 - self.alpha
        self.drive += self.alpha * np.random.uniform(-0.2, 1.0, (2))
        return self.drive, 0, True
