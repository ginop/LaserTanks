import numpy as np
from math import pi


def rotate(points, angle):
    """
    A helper function for 2D rotations.
    Inputs:
        points: a 2xn numpy array
        angle: rotation angle in degrees
    Outputs:
        points: rotated array
    """
    ca = np.cos(angle*pi/180)
    sa = np.sin(angle*pi/180)
    R = np.array([[[ca, -sa], [sa, ca]]])
    points.shape += (1,)
    points = (R*points).sum(axis=1)
    return points


class Tank():
    # TODO: Move Tank class to separate file from pygame code

    # Define universal tank shape as Class (not instance) properties
    body_width = 2.
    body_length = 3.
    tread_width = 0.4
    tread_length = body_length + 0.2
    tread_overlap = 0.1
    turret_radius = 0.6
    barrel_width = 0.2
    barrel_length = 1.
    laser_width = barrel_width
    blast_radius = 0.4

    laser_dur = 0.5  # duration of laser shot in seconds
    damage = 100  # HP per sec
    reload_time = 1

    def __init__(self, control, color, pos=[0., 0.], orient=[0., 0.]):
        self.game = None
        self.control = control
        self.position = np.array(pos).astype(float)
        self.orientation = np.array(orient)
        self.velocity = np.array([0, 0]).astype(float)
        self.drive = np.array([0., 0.])  # L and R tread speed in units/sec
        self.tread_offset = np.array([0., 0.])  # for animating treads
        self.spin = 0.  # commanded turret spin in degrees per sec
        self.shoot = False  # command to fire laser
        self.time_to_ready = 0.  # counts down from reload_time after each shot
        self.hull = 100
        self.battery = 100
        self.color = color
        # self.shield = None

    def draw(self):
        # Specify tank shape centered on origin with 0 rotation (pointed up)
        u = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])/2
        body = u * [Tank.body_width, Tank.body_length]
        tread_r = (u * [Tank.tread_width, Tank.tread_length] +
                   [Tank.body_width/2 + Tank.tread_width/2 -
                    Tank.tread_overlap, 0])
        tread_l = -tread_r
        barrel = (u * [Tank.barrel_width, Tank.barrel_length] +
                  [0, Tank.barrel_length/2])

        # Rotate treads and body
        body = rotate(body, self.orientation[0])
        tread_r = rotate(tread_r, self.orientation[0])
        tread_l = rotate(tread_l, self.orientation[0])
        # Rotate barrel
        barrel = rotate(barrel, self.orientation[0]+self.orientation[1])
        # Translate all pieces
        body += self.position
        tread_r += self.position
        tread_l += self.position
        barrel += self.position

        # Draw the pieces as filled shapes with antialiased edges
        self.game.polygon(tread_r, self.color)
        self.game.polygon(tread_l, self.color)

        # Draw tread lines
        for offset, sign in zip(self.tread_offset, [1, -1]):
            n = 5
            for ii in range(n):
                y = (Tank.tread_length*ii/n + offset) % Tank.tread_length
                y -= Tank.tread_length/2
                x1 = Tank.body_width/2 - Tank.tread_overlap
                x2 = Tank.body_width/2 + Tank.tread_width - Tank.tread_overlap
                pts = np.array([[sign*x1, y], [sign*x2, y]])
                pts = rotate(pts, self.orientation[0])
                pts += self.position
                self.game.line(pts, (0, 0, 0))

        self.game.polygon(body, (0, 0, 0))
        self.game.polygon(barrel, self.color)
        self.game.circle(self.position, 0.6, self.color)

        # print HP on tank turret
        self.game.text("{}".format(self.hull), self.position, (0, 0, 0), centered=True)

        # draw laser if shooting (indicated by time_to_read above reload_time)
        # TODO: Implement hit checking and update laser drawing accordingly
        if self.time_to_ready > self.reload_time:
            laser = (u * [Tank.laser_width, 1000] +
                     [0, Tank.barrel_length + 0.5 + 500])
            laser = rotate(laser, self.orientation[0]+self.orientation[1])
            laser += self.position
            self.game.polygon(laser, self.color, (255, 255, 255))

    def update(self, dt, t, info):
        self.drive, self.spin, self.shoot = self.control(t, Tank, info)

        self.time_to_ready -= dt
        if self.time_to_ready > 0.:
            self.shoot = False
        elif self.shoot:
            self.time_to_ready = Tank.reload_time + self.laser_dur

    def move(self, dt):
        # forward motion is average of tread speeds
        v = (self.drive[0] + self.drive[1])/2
        # relative speed in the treads causes rotation
        spread = Tank.body_width + Tank.tread_width - 2*Tank.tread_overlap
        w = (self.drive[1] - self.drive[0])/spread
        # motion is an arc
        self.tread_offset += self.drive*dt
        self.orientation[1] += self.spin*dt
        a = self.orientation[0]*pi/180  # for brevity below
        if w == 0.0:
            self.position[0] += dt*v*np.sin(a)
            self.position[1] += dt*v*np.cos(a)
        else:
            self.position[0] -= v/w*(np.cos(a+w*dt)-np.cos(a))
            self.position[1] += v/w*(np.sin(a+w*dt)-np.sin(a))
            self.orientation[0] += w*dt*180/pi

    def public(self):
        return {"position": self.position,
                "orientation": self.orientation,
                "is_firing": self.time_to_ready > self.reload_time,
                "color": self.color}
