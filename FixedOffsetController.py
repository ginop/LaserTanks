import numpy as np
from math import pi
from LaserTankController import LaserTankController


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


class FixedOffsetController(LaserTankController):

    def __init__(self):
        pass

    def main(self, t, Tank, me, them):
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
