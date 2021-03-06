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


class Waypoint_PID_Controller(LaserTankController):

    def __init__(self):
        self.waypoint_gen = self.random_waypoint()
        self.destination = next(self.waypoint_gen)
        self.drive_pid = PID(0.5, 0.0, 0.8)
        self.aim_pid = PID(100.0, 0.0, 0.0)

    def main(self, t, Tank, me, them):
        if len(them) == 0:
            them = [{"position": np.array([200., 200.])}]

        them = them[0]  # only concerned with one target

        # import pdb; pdb.set_trace()
        # Use a PID controller to keep the tank pointed to the next waypoint
        vec = self.destination - me['position']
        dist = np.sqrt(vec.dot(vec))
        angle = np.arctan2(vec[1], vec[0]) - me['orientation'][0]
        d_thresh = 2.
        if dist < d_thresh:  # get next waypoint
            self.destination = next(self.waypoint_gen)
            vec = self.destination - me['position']
            dist = np.sqrt(vec.dot(vec))
            angle = np.arctan2(vec[1], vec[0]) - me['orientation'][0]

        angle = (angle + pi) % (2*pi) - pi  # in +-pi
        w = self.drive_pid.step(angle)
        # Drive as fast as possible without saturating either tread drive
        B = Tank.body_width + Tank.tread_width - 2*Tank.tread_overlap
        drive = np.array([0., 0.])
        if -2 < B*w < 2:
            if w > 0:
                drive[1] = 1
                drive[0] = 1 - B*w
            else:
                drive[0] = 1
                drive[1] = 1 + B*w
        else:
            drive[0] = -np.sign(w)
            drive[1] = np.sign(w)

        # Use a PID controller to keep the turret focused on the enemy
        vec = them['position'] - me['position']
        angle = np.arctan2(vec[1], vec[0])
        # Determine difference between aim and desired aim for PID controller
        d_angle = angle - me['orientation'][1]
        d_angle = (d_angle + pi) % (2*pi) - pi  # in +-pi
        turret_spin = self.aim_pid.step(d_angle)

        # Fire when aim is close enough
        # but not during a turn or too close to a waypoint update
        ttg = (dist-d_thresh) / me["velocity"].get_length()
        fire = abs(d_angle) < 4 and 0 < abs(me["spin"]) < 15 and ttg > 0.5

        return np.tanh(drive), np.tanh(turret_spin), True

    def waypoint(self):
        # Create an infinite generator of waypoints, cycling through the list
        from itertools import cycle
        # For a 400x400 arena size
        pts = [(50, 50), (350, 50), (350, 350), (50, 350)]
        for point in cycle(pts):
            yield np.array(point)


    def random_waypoint(self):
        while True:
            # For a 400x400 arena size
            yield np.random.rand(2) * 300. + 50.


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
