import numpy as np
from math import pi, degrees, radians
from importlib import import_module
from scipy.integrate import quad


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
    damage = 25.  # HP per sec
    reload_time = 1.

    def __init__(self, control, color, pos=[0., 0.], orient=[0., 0.]):
        self.game = None
        # Load named module and instantiate object of class with same name
        self.control = import_module(control).__dict__[control]()

        """Traditional x (horizontal, positive to the right) and y (vertical,
        positive up) coordinates with origin at lower left corner of arena."""
        self.position = np.array(pos).astype(float)
        """First element is right-handed rotation of body in arena with 0 on
        x-axis and positive angles rotating CCW. Second is similarly defined
        angle between body an dturret, 0 is forward. In degrees."""
        self.orientation = np.array(orient).astype(float)
        """Tank speed, always in the forward direction."""
        self.speed = 0.
        """Rate of change of self.orientation[0]"""
        self.spin = 0.

        self.drive = np.array([0., 0.])  # normalized L, R tread force (in +-1)
        self.tread_offset = np.array([0., 0.])  # for animating treads

        self.turret_torque = 0.  # normalized turret torque (in +-1)
        self.turret_spin = 0.  # turret spin in degrees per sec

        self.shoot = False  # command to fire laser
        self.time_to_ready = self.reload_time
        self.laser_length = Tank.laser_length  # made shorter on hit

        self.hull = 100.
        self.battery = 100.
        self.color = color

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

    def move(self, dt):
        # Modeled After Nutaro, Table 2.1
        m = 100  # kg
        J = (m*4/5)/12*(Tank.body_width**2+Tank.body_length**2)  # kg m**2
        B = Tank.body_width + Tank.tread_width - 2*Tank.tread_overlap

        tread_force_max = 200.  # N
        turret_torque_max = 10.  # N m

        top_speed = 10.  # m/s
        Br = 2*tread_force_max/top_speed  # Rolling friction, N s / m
        top_spin = pi  # rad/sec
        Bl = B*tread_force_max/top_spin  # Turning friction, N m s / rad
        Bl2 = Bl*Tank.tread_length/4  # Lateral friction, N s / m
        Sl = tread_force_max*B/10  # Sticking friction, N m
        # For turret direction
        Jt = (m/5)*Tank.turret_radius**2/2  # kg m**2
        top_spin = pi/4  # rad/sec
        Bt = turret_torque_max/top_spin  # Turning friction, N m s / rad
        St = turret_torque_max/20*0  # Sticking threshold, N m

        # Scale normalized inputs to physical values
        Fl = self.drive[0] * tread_force_max
        Fr = self.drive[1] * tread_force_max
        Ft = self.turret_torque * turret_torque_max

        # Tank only turns if relative tread force can overcome lateral friction
        # Static friction is relevant only when tank is turning slowly enough
        T = (Fr-Fl)*B/2  # torque applied by treads
        if abs(T) < Sl and abs(self.spin) < 2.:
            # Not turning
            w0 = self.spin * pi/180
            w1 = 0
        else:
            w0 = self.spin * pi/180
            dwdt = (T-Bl*w0)/J
            w1 = w0 + dwdt*dt
        self.spin = w1 * 180/pi

        v0 = self.speed
        dvdt = (Fl+Fr-Br*v0)/m
        v1 = v0 + dvdt*dt
        self.speed = v1

        theta0 = self.orientation[0] * pi/180
        dtheta = (w0+w1)/2*dt
        self.orientation[0] += dtheta * 180/pi

        # If a collision has caused the tank to slide sideways,
        # apply drag and redistribute into speed if turning
        self.slide  = 

        r = (v0+v1)/2*dt

        self.position[0] += r * np.cos(theta0+dtheta/2)
        self.position[1] += r * np.sin(theta0+dtheta/2)

        # Calculate each tread rotation
        self.tread_offset[0] += r - dtheta*B/2  # left
        self.tread_offset[1] += r + dtheta*B/2  # right

        # Turret can spin if already spinning or if force can break sticking
        if abs(Ft) > St or self.turret_spin > 2.:
            # Turret can turn
            # Following equations for tank body turning
            # TODO add forces from turning of body
            w0 = radians(self.turret_spin)
            dwdt = (Ft-Bt*w0)/Jt
            self.turret_spin += degrees(dwdt*dt)
            self.orientation[1] += degrees(w0*dt + dwdt*dt/2)
        else:
            self.turret_spin = 0

    def info(self):
        return {"position": self.position,
                "orientation": self.orientation,
                "is_firing": self.time_to_ready > self.reload_time,
                "color": self.color,
                "spin": self.spin,
                "speed": self.speed,
                "turret_spin": self.turret_spin,
                "hull": self.hull,
                "battery": self.battery}

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
            dist: distance from laser origin to impact point, None if no impact
        """
        # Determine hitbox for tank (just use body, not treads, for simplicity)
        u = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])/2
        hitbox = u * [self.body_length, self.body_width]
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
