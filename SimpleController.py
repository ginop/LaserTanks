import numpy as np
from math import pi
from LaserTankController import LaserTankController


class SimpleController(LaserTankController):

    def __init__(self):
        pass

    def main(self, t, Tank, me, them):
        if (t % 10) > 5:
            spin = 0.25
        else:
            spin = -0.50
        return(np.array([0.36, 0.4]), spin, True)
