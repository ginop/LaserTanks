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


def rotate(points, angle):
    """
    A helper function for 2D rotations.
    Inputs:
        points: a nx2 numpy array
        angle: rotation angle in degrees
    Outputs:
        points: nx2 rotated array
    """
    ca = np.cos(angle*pi/180)
    sa = np.sin(angle*pi/180)
    R = np.array([[ca, -sa], [sa, ca]])  # positive is CCW
    R.shape += (1,)  # add dim for broadcasting over n points
    points = points.T
    points.shape = (1,) + points.shape  # 1x2xn
    points = (R*points).sum(axis=1).T  # do rotation and return original shape
    return points


def main(t, Tank, me, them):

    if len(them) == 0:
        return(np.array([0., 0.]), 0., False)

    them = them[0]  # only concerned with one target

    targetoffset = [-9, 5]

    target = them['position'] + rotate(np.array(targetoffset),
                                       them['orientation'][0])
    vec = target - me['position']

    ang = (np.arctan2(vec[0, 1], vec[0, 0]) * 180/pi %
           360 - me['orientation'][0]) % 360

    ang = np.where(ang > 180, ang - 360, ang)

    speed = 10
    diff = -ang / 180

    drive = speed * np.array([1. + diff, 1. - diff])

    vec = them['position'] - me['position']
    angle = np.arctan2(vec[1], vec[0])*180/pi - me['orientation'][0]
    d_angle = (angle - me['orientation'][1] + 180) % 360 - 180  # [-180, 180]
    spin = 2. * d_angle

    return drive, spin, True


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


def dummy(t, Tank, me, info):
    drive = np.array([0., 0.])
    spin = 0.
    shoot = False
    return drive, spin, shoot


def shooty(t, Tank, me, info):
    drive = np.array([0., 0.])
    spin = 0.
    shoot = True
    return drive, spin, shoot


def R(t, Tank, me, info):
    drive = np.array([4., 1.])
    spin = 0.
    shoot = (t % 3) < 0.2
    return drive, spin, shoot


def G(t, Tank, me, info):
    drive = np.array([1., -1.])
    spread = Tank.body_width + Tank.tread_width - 2*Tank.tread_overlap
    w = (drive[1] - drive[0])/spread
    spin = -w*180/pi  # counter spin
    shoot = (t % 4) < 0.2
    return drive, spin, shoot


def B(t, Tank, me, info):
    drive = np.array([3., 5.])
    spread = Tank.body_width + Tank.tread_width - 2*Tank.tread_overlap
    w = (drive[1] - drive[0])/spread
    spin = -w*180/pi/2  # partial counter spin
    shoot = True
    return drive, spin, shoot
