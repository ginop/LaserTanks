import numpy as np
from math import pi
from importlib import import_module


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


class Tank():

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

    laser_length = 999.  # default laser_length, overwritten per instance
    laser_dur = 0.5  # duration of laser shot in seconds
    damage = 50.  # HP per sec
    reload_time = 1.

    def __init__(self, control, color, pos=[0., 0.], orient=[0., 0.]):
        self.game = None
        # Load named module and instantiate object of class with same name
        self.control = import_module(control).__dict__[control]()
        self.position = np.array(pos).astype(float)
        self.orientation = np.array(orient)
        self.velocity = np.array([0, 0]).astype(float)
        self.drive = np.array([0., 0.])  # L and R tread speed in units/sec
        self.tread_offset = np.array([0., 0.])  # for animating treads
        self.spin = 0.  # commanded turret spin in degrees per sec
        self.shoot = False  # command to fire laser
        self.time_to_ready = self.reload_time
        self.hull = 100.
        self.battery = 100.
        self.color = color
        self.laser_length = Tank.laser_length  # made shorter on hit
        # self.shield = None

    def draw(self):
        # Specify tank shape centered on origin with 0 rotation (pointed right)
        u = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])/2
        body = u * [Tank.body_length, Tank.body_width]
        tread_r = (u * [Tank.tread_length, Tank.tread_width] +
                   [0, Tank.body_width/2 + Tank.tread_width/2 -
                    Tank.tread_overlap])
        tread_l = -tread_r
        barrel = (u * [Tank.barrel_length, Tank.barrel_width] +
                  [Tank.barrel_length/2, 0])

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
                x = (Tank.tread_length*ii/n + offset) % Tank.tread_length
                x -= Tank.tread_length/2
                y1 = Tank.body_width/2 - Tank.tread_overlap
                y2 = Tank.body_width/2 + Tank.tread_width - Tank.tread_overlap
                pts = np.array([[x, sign*y1], [x, sign*y2]])
                pts = rotate(pts, self.orientation[0])
                pts += self.position
                self.game.line(pts, (0, 0, 0))

        self.game.polygon(body, (0, 0, 0))
        self.game.polygon(barrel, self.color)
        self.game.circle(self.position, 0.6, self.color)

        # print HP on tank turret
        self.game.text("{:.0f}".format(self.hull), self.position,
                       (0, 0, 0), centered=True)

    def draw_laser(self):
        # draw laser if shooting (indicated by time_to_read above reload_time)
        u = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])/2
        if self.time_to_ready > self.reload_time:
            laser = (u * [self.laser_length, self.laser_width] +
                     [self.barrel_length + self.laser_length/2, 0])
            laser = rotate(laser, self.orientation[0]+self.orientation[1])
            laser += self.position
            self.game.polygon(laser, self.color)
            center = self.barrel_length + self.laser_length
            a = (self.orientation[0]+self.orientation[1])*pi/180
            center = [center*np.cos(a), center*np.sin(a)] + self.position
            self.game.circle(center, self.blast_radius, self.color)

    def update(self, dt, t, target_info):
        self.drive, self.spin, self.shoot = self.control.main(t, Tank,
                                                              self.public(),
                                                              target_info)
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
            self.position[1] += dt*v*np.sin(a)
            self.position[0] += dt*v*np.cos(a)
        else:
            self.position[1] -= v/w*(np.cos(a+w*dt)-np.cos(a))
            self.position[0] += v/w*(np.sin(a+w*dt)-np.sin(a))
            self.orientation[0] += w*dt*180/pi

    def public(self):
        return {"position": self.position,
                "orientation": self.orientation,
                "is_firing": self.time_to_ready > self.reload_time,
                "color": self.color}

    def get_beam(self):
        if self.time_to_ready > self.reload_time:
            barrel = np.array([[Tank.barrel_length, 0]])
            barrel = rotate(barrel, self.orientation[0]+self.orientation[1])
            barrel += self.position
            return (barrel[0, 0], barrel[0, 1],
                    self.orientation[0]+self.orientation[1])
        else:
            return None

    def detect_hit(self, laser):
        """
        Inputs:
            laser: a tuple of origin and angle (x, y, angle)
        Outputs:
            dist: distance from laser origin to impact point, [] if no impact
        """
        # Determine hitbox for tank (just use body, not treads, for simplicity)
        u = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])/2
        hitbox = u * [self.body_length, self.body_width]
        # import pdb; pdb.set_trace()
        hitbox = rotate(hitbox, self.orientation[0])
        hitbox += self.position
        # Translate laser to origin
        hitbox -= laser[:2]
        # Rotate laser to x-axis
        hitbox = rotate(hitbox, -laser[2])
        # There is a hit if any edge crosses x-axis at non-negative x
        hitbox = np.vstack((hitbox, hitbox[0, :]))
        dist = []
        for ii in range(4):
            # TODO: switch x and y when angle is fixed to CCW from x-axis
            x1 = hitbox[ii, 0]
            y1 = hitbox[ii, 1]
            x2 = hitbox[ii+1, 0]
            y2 = hitbox[ii+1, 1]
            if x1 >= 0 or x2 >= 0:
                if y1 == 0 and y2 == 0:
                    """ when both point lie on the x axis, the hit occurs at
                    the closer to 0, or at 0 if they overlap (which would
                    indicate a tank crash)"""
                    dist.append(max(0, hitbox[ii:ii+1, 0].min()))
                elif (y1 <= 0 and y2 >= 0) or (y1 >= 0 and y2 <= 0):
                    dist.append(x1 + y1*(x2-x1)/(y2-y1))
        if len(dist) > 0:
            return min(dist)
        else:
            return None
