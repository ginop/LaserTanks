import numpy as np
from math import pi
from importlib import import_module
from scipy.integrate import quad, odeint
from LaserTankUtilities import rotate


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

    def get_hitbox(self):
        # Determine hitbox for tank (just use body, not treads, for simplicity)
        u = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])/2
        hitbox = u * [self.tread_length,
                      self.body_width+2*(self.tread_width-self.tread_overlap)]
        hitbox = rotate(hitbox, self.orientation[0])
        hitbox += self.position
        return hitbox

    def move(self, dt):
        # Modeled After Nutaro, Table 2.1
        m = 0.1  # kg
        J = 5e-4  # kg m**2
        B = Tank.body_width + Tank.tread_width - 2*Tank.tread_overlap
        Br = 1.0  # Rolling friction, N s / m
        Bs = 0.4  # 14.  # Sliding friction, N s / m
        Bl = 0.7  # Turning friction, N m s / rad
        Sl = 0.3*0  # Lateral friction, N m
        # For turret direction
        Jt = 5e-5  # kg m**2
        Bt = 0.02  # Turning friction, N m s / rad
        St = 0.01  # Sticking threshold, N m
        # Scale normalized inputs to physical values
        tread_force_max = 5.  # N
        Fl = self.drive[0] * tread_force_max
        Fr = self.drive[1] * tread_force_max
        turret_torque_max = 0.1  # N m
        Ft = self.turret_torque * turret_torque_max

        def move_ode(states, t):
            # states                derivatives
            # ------                -----------
            # x                     x dot
            # y                     y dot
            # x_dot                 x ddot
            # y_dot                 y ddot
            # body angle            body angle rate
            # body angle rate       body angle accel
            # turret angle          turret angle rate
            # turret angle rate     turret angle accel
            x, y, dx, dy, a1, w1, a2, w2 = states
            v = np.sqrt(dx**2 + dy**2)
            # Tank only turns if relative tread force can overcome lateral
            # friction or if is already turning fast enough
            if abs((Fr-Fl)*B/2) < Sl and abs(w1) < 2.:
                # Not turning
                dv = (Fl+Fr-Br*v)/m
                w1 = 0
                dw1 = 0
            else:
                # Is turning
                dv = (Fl+Fr-(Br+Bs)*v)/m
                dw1 = ((Fl-Fr)*B/2 - Bl*w1)/J
            ddx = dv * np.cos(a1)
            ddy = dv * np.sin(a1)
            # Turret can spin if already spinning or if force can break sticking
            # TODO add forces from turning of body and inertia of turret
            if abs(Ft) > St or w2 > 2.:
                # Turret can turn
                dw2 = (Ft - Bt*w2)/Jt
            else:
                w2 = 0
                dw2 = 0
            return dx, dy, ddx, ddy, w1, dw1, w2, dw2

        x = self.position[0]
        y = self.position[1]
        v = self.speed
        a1 = self.orientation[0] * pi/180
        w1 = self.spin * pi/180
        dx = v * np.cos(a1)
        dy = v * np.sin(a1)
        a2 = self.orientation[1] * pi/180
        w2 = self.turret_spin * pi/180

        x0 = (x, y, dx, dy, a1, w1, a2, w2)
        x1 = odeint(move_ode, x0, [0, dt], rtol=1e-3, atol=1e-2)
        x, y, dx, dy, a1, w1, a2, w2 = x1[1, :]

        # Update position so that get_hitbox returns post ODE positions
        self.position[0] = x
        self.position[1] = y
        self.orientation = [a1 * 180/pi, a2 * 180/pi]

        # TODO implement rotation caused by collisions
        # TODO add friction to wall and obstacle interactions

        # Enforce obstacle collisions
        # The simple method used here examines the straight line between the
        # previous and new positions and, if it intersects an obstacle, removes
        # enough of the perpendicular component of that displacement so that
        # the tank no longer overlaps the obstacle and adjusts the velocity
        # accordingly. In the case where the tank collides with the corner of
        # the obstacle, the tank is moved perpendicular to the intersecting
        # face and is allowed to slide on the corner parallel to that face.
        delta_x = x1[0] - x0[0]
        delta_y = x1[0] - x0[0]

        # Update position for post obstacle collision get_hitbox
        self.position[0] = x
        self.position[1] = y
        self.orientation = [a1 * 180/pi, a2 * 180/pi]

        # Enforce walls
        pts = self.get_hitbox()
        max_x = max(pts[:, 0])
        min_x = min(pts[:, 0])
        max_y = max(pts[:, 1])
        min_y = min(pts[:, 1])
        if min_x <= 0:  # left
            x -= min_x
            if dx <= 0:
                dx = 0.0
        if max_x >= self.game.screen_width:  # right
            x -= max_x - self.game.screen_width
            if dx >= 0:
                dx = 0.0
        if min_y <= 0:  # bottom
            y -= min_y
            if dy <= 0:
                dy = 0.0
        if max_y >= self.game.screen_height:  # top
            y -= max_y - self.game.screen_height
            if dy >= 0:
                dy = 0.0

        v = np.sqrt(dx**2 + dy**2)

        self.position[0] = x
        self.position[1] = y
        self.speed = v
        self.orientation = [a1 * 180/pi, a2 * 180/pi]
        self.spin = w1 * 180/pi
        self.turret_spin = w2 * 180/pi
        # Calculate each tread rotation (approximately)
        self.tread_offset += dt * self.speed
        self.tread_offset[0] -= dt * B*w1/2
        self.tread_offset[1] += dt * B*w1/2

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
