import numpy as np
import pymunk
from pymunk.vec2d import Vec2d
from math import sqrt, cos, sin, pi
from math import pi, degrees, radians
from importlib import import_module
from scipy.integrate import quad, odeint

def rotate(points, angle):
    """
    A helper function for 2D rotations.
    Inputs:
        points: a nx2 numpy array
        angle: rotation angle in radians
    Outputs:
        points: nx2 rotated array
    """
    ca = np.cos(angle)
    sa = np.sin(angle)
    R = np.array([[ca, -sa], [sa, ca]])  # positive is CCW
    R.shape += (1,)  # add dim for broadcasting over n points
    points = points.T
    points.shape = (1,) + points.shape  # 1x2xn
    points = (R*points).sum(axis=1).T  # do rotation and return original shape
    return points


class Tank():

    # Define universal tank shape as Class (not instance) properties
    body_width = 20.
    body_length = 30.
    tread_width = 4.
    tread_length = body_length + 2.
    tread_overlap = 1.
    turret_radius = 6.
    barrel_width = 2.
    barrel_length = 10.
    laser_width = barrel_width
    blast_radius = 4.

    laser_length = 999.  # default laser_length, overwritten per instance
    laser_dur = 0.5  # duration of laser shot in seconds
    damage = 10.  # HP per sec
    reload_time = 1.0

    def __init__(self, control, color, pos=[0., 0.], orient=[0., 0.]):
        self.game = None
        # Load named module and instantiate object of class with same name
        self.control = import_module(control).__dict__[control]()

        self.drive = np.array([0., 0.])  # normalized L, R tread force (in +-1)
        self.turret_torque = 0.  # normalized turret torque (in +-1)

        self.tread_offset = np.array([0., 0.])  # for animating treads

        self.shoot = False  # command to fire laser
        self.time_to_ready = self.reload_time
        self.laser_length = Tank.laser_length  # shorter when it hits something

        self.hull = 100.
        self.battery = 100.  # not used (yet)
        self.color = color

        # Create the pymunk object that will be used for collision detection and resolution
        mass = 100
        size = (max(self.body_length, self.tread_length),
                self.body_width+2*(self.tread_width-self.tread_overlap))
        moment = pymunk.moment_for_box(mass, size)
        self.body = pymunk.Body(mass, moment)
        self.box = pymunk.Poly.create_box(self.body, size, 0.1)  # round the corners a bit
        self.box.friction = 0.5
        self.body.position = pos
        self.body.angle = orient[0]
        # And add turret pymunk object with pivot joint with friction
        turret_mass = 20
        barrel_mass = 4
        mass = turret_mass + barrel_mass
        moment = turret_mass/2*self.turret_radius**2 + \
                 barrel_mass/3*self.barrel_length**2
        self.turret_body = pymunk.Body(mass, moment)
        self.turret_body.position = pos
        self.turret_body.angle = orient[1]
        self.turret_circle = pymunk.Circle(self.turret_body,
                                           self.turret_radius)
        self.pivot = pymunk.constraint.PivotJoint(self.body,
                                                  self.turret_body,
                                                  (0, 0), (0, 0))
        self.pivot.collide_bodies = False
        # Create friction between the two with a damped spring with no spring constant
        self.pivot_friction = pymunk.constraint.DampedRotarySpring(self.body, self.turret_body, 0, 0, 5e3)


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
        body = rotate(body, self.body.angle)
        tread_r = rotate(tread_r, self.body.angle)
        tread_l = rotate(tread_l, self.body.angle)
        # Rotate barrel
        barrel = rotate(barrel, self.turret_body.angle)
        # Translate all pieces
        body += self.body.position
        tread_r += self.body.position
        tread_l += self.body.position
        barrel += self.body.position

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
                pts = rotate(pts, self.body.angle)
                pts += self.body.position
                self.game.line(pts, (0, 0, 0))

        self.game.polygon(body, (0, 0, 0))
        self.game.polygon(barrel, self.color)
        self.game.circle(self.turret_body.position, self.turret_radius, self.color)

        # print HP on tank turret
        self.game.text("{:.0f}".format(self.hull), self.turret_body.position,
                       (0, 0, 0), centered=True)

    def draw_laser(self):
        # draw laser if shooting (indicated by time_to_read above reload_time)
        u = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])/2
        if self.time_to_ready > self.reload_time:
            laser = (u * [self.laser_length, self.laser_width] +
                     [self.barrel_length + self.laser_length/2, 0])
            laser = rotate(laser, self.turret_body.angle)
            laser += self.turret_body.position
            self.game.polygon(laser, self.color)
            center = self.barrel_length + self.laser_length
            a = self.turret_body.angle
            center = [center*np.cos(a), center*np.sin(a)] + self.turret_body.position
            self.game.circle(center, self.blast_radius, self.color)

    def update(self, dt, t, target_info):
        (self.drive,
         self.turret_torque,
         self.shoot) = self.control.main(t, Tank, self.info(), target_info)
        if (abs(self.drive[0]) > 1 or abs(self.drive[1]) > 1 or
           abs(self.turret_torque) > 1):
            from warnings import warn
            warn("Tank controller outputs for tread and turret " +
                 "torque should all be between -1 and 1!")
            self.drive[0] = min(max(-1, self.drive[0]), 1)
            self.drive[1] = min(max(-1, self.drive[1]), 1)
            self.turret_torque = min(max(-1, self.turret_torque), 1)

        self.time_to_ready -= dt
        if self.time_to_ready > 0.:
            self.shoot = False
        elif self.shoot:
            self.time_to_ready = Tank.reload_time + self.laser_dur

    def get_hitbox(self):
        # Determine hitbox for tank (just use body, not treads, for simplicity)
        u = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])/2
        hitbox = u * [self.tread_length,
                      self.body_width+2*(self.tread_width-self.tread_overlap)]
        hitbox = rotate(hitbox, self.body.angle)
        hitbox += self.body.position
        return hitbox

    def apply_forces(self):

        # Calculate tank's net force, torque from treads and friction
        half_width = self.body_width/2+self.tread_width-self.tread_overlap

        direction = Vec2d(cos(self.body.angle), sin(self.body.angle))
        perp = direction.perpendicular()

        tread_force_max = 8000
        Fl = self.drive[0] * tread_force_max * direction
        Fr = self.drive[1] * tread_force_max * direction

        # Calculate commanded forces
        F = Fr + Fl
        F = Vec2d(F[0], F[1])
        T = Vec2d(0,-half_width)
        T.rotate(self.body.angle)
        T = T.cross(Fr-Fl)

        # Determine if tank is able to turn
        forward_speed = direction.dot(self.body.velocity)
        slip_speed = perp.dot(self.body.velocity)
        if abs(slip_speed) < 2 and abs(T) < 500:
            slip_speed = 0
            T = 0
            self.body.angular_velocity = 0
            self.body.velocity = forward_speed * direction

        # Apply friction forces
        F -= 200*forward_speed*direction + 1000*slip_speed*perp
        T -= 100000*self.body.angular_velocity

        # Synthesize the net force and torque with F1 at CoM and F2 on edge
        F.rotate(-self.body.angle)  # from world to tank frame
        F2 = Vec2d(T/half_width, 0)  # in tank frame
        F1 = F - F2  # in tank frame
        self.body.apply_force_at_local_point(F1, (0, 0))
        self.body.apply_force_at_local_point(F2, (0, -half_width))

        # Apply turret torque by applying mirrored forces on
        # opposite sides of its axis of rotation
        f = 1e4 * self.turret_torque / 2
        self.turret_body.apply_force_at_local_point((0, f), ( 1, 0))
        self.turret_body.apply_force_at_local_point((0,-f), (-1, 0))

    def move(self, dt):
        dtheta = self.body.angular_velocity * dt

        direction = Vec2d(cos(self.body.angle), sin(self.body.angle))
        forward_speed = direction.dot(self.body.velocity)
        dpos = forward_speed * dt

        #self.position = self.body.position
        #self.orientation[0] = self.body.angle*180/pi
        #self.orientation[1] = (self.turret_body.angle -
        #                       self.body.angle)*180/pi

        self.tread_offset += dpos
        self.tread_offset += dtheta * self.body_width/2 * Vec2d(-1,1)

    def info(self):
        return {"position": self.body.position,
                "orientation": [self.body.angle, self.turret_body.angle],
                "is_firing": self.time_to_ready > self.reload_time,
                "color": self.color,
                "velocity": self.body.velocity,
                "spin": self.body.angular_velocity,
                "turret_spin": self.turret_body.angular_velocity,
                "hull": self.hull,
                "battery": self.battery}

    def public(self):
        return {"position": self.body.position,
                "orientation": [self.body.angle, self.turret_body.angle],
                "is_firing": self.time_to_ready > self.reload_time,
                "color": self.color}

    def get_beam(self):
        if self.time_to_ready > self.reload_time:
            barrel = np.array([[Tank.barrel_length, 0]])
            barrel = rotate(barrel, self.turret_body.angle)
            barrel += self.body.position
            return (barrel[0, 0], barrel[0, 1], self.turret_body.angle)
        else:
            return None

    def detect_hit(self, laser):
        """
        Inputs:
            laser: a tuple of origin and angle (x, y, angle)
        Outputs:
            dist: distance from laser origin to impact point, None if no impact
        """
        hitbox = self.get_hitbox()
        # Translate laser to origin
        hitbox -= laser[:2]
        # Rotate laser to x-axis
        hitbox = rotate(hitbox, -laser[2])
        # There is a hit if any edge crosses x-axis at non-negative x
        hitbox = np.vstack((hitbox, hitbox[0, :]))
        dist = []
        for ii in range(4):
            x1 = hitbox[ii, 0]
            y1 = hitbox[ii, 1]
            x2 = hitbox[ii+1, 0]
            y2 = hitbox[ii+1, 1]
            if x1 >= 0 or x2 >= 0:
                # testing against 0 doesn't work in all cases because of
                # rounding errors so we test agianst this theshold instead.
                e = 1e-12
                if abs(y1) < e and abs(y2) < e:
                    """when both points lie on the x axis, the hit occurs at
                    the lower non-negative number, or at 0 if one is negative
                    (which would indicate a tank crash)"""
                    dist.append(max(0, hitbox[ii:ii+1, 0].min()))
                elif (y1 <= e and y2 >= -e) or (y1 >= -e and y2 <= e):
                    dist.append(max(0, x1 - y1*(x2-x1)/(y2-y1)))
        if len(dist) > 0:
            return min(dist)
        else:
            return None
