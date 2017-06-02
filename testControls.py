import numpy as np
from math import pi

"""
Controller Inputs:
    time (seconds)
    Tank (the Tank class so it's class constants are usable here)

Controller outputs:
    drive (array of left, right tread speed in units per sec)
    spin (turret rotation speed in degrees per sec)
    shoot (logical to fire laser, ignored while recharging)
"""

def dummy(t, Tank, info):
    drive = np.array([0., 0.])
    spin = 0.
    shoot = False
    return drive, spin, shoot

def shooty(t, Tank, info):
    drive = np.array([0., 0.])
    spin = 0.
    shoot = True
    return drive, spin, shoot
    

def R(t, Tank, info):
    drive = np.array([4., 1.])
    spin = 0.
    shoot = (t % 3) < 0.2
    return drive, spin, shoot


def G(t, Tank, info):
    drive = np.array([2., -2.])
    spread = Tank.body_width + Tank.tread_width - 2*Tank.tread_overlap
    w = (drive[1] - drive[0])/spread
    spin = -w*180/pi  # counter spin
    shoot = (t % 4) < 0.2
    return drive, spin, shoot


def B(t, Tank, info):
    drive = np.array([3., 5.])
    spread = Tank.body_width + Tank.tread_width - 2*Tank.tread_overlap
    w = (drive[1] - drive[0])/spread
    spin = -w*180/pi/2  # partial counter spin
    shoot = True
    return drive, spin, shoot
