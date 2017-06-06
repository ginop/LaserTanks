import numpy as np
from math import pi
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
    damage = 50.  # HP per sec
    reload_time = 1.

    def __init__(self, control, color, pos=[0., 0.], orient=[0., 0.]):
        self.game = None
        # Load named module and instantiate object of class with same name
        self.control = import_module(control).__dict__[control]()

        self.position = np.array(pos).astype(float)
        self.orientation = np.array(orient).astype(float)
        self.speed = 0.
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
        m = 0.1  # kg
        J = 5e-4  # kg m**2
        B = Tank.body_width + Tank.tread_width - 2*Tank.tread_overlap
        Br = 1.0  # Rolling friction, N s / m
        Bs = 14.*0  # Sliding friction, N s / m
        Bl = 0.7  # Turning friction, N m s / rad
        Sl = 0.3*0  # Lateral friction, N m
        # For turret direction
        Jt = 5e-5  # kg m**2
        Bt = 0.02  # Turning friction, N m s / rad
        St = 0.01*0  # Sticking threshold, N m
        # Scale normalized inputs to physical values
        tread_force_max = 5.  # N
        Fl = self.drive[0] * tread_force_max
        Fr = self.drive[1] * tread_force_max
        turret_torque_max = 0.1  # N m
        Ft = self.turret_torque * turret_torque_max
        # Tank only turns if relative tread force can overcome lateral friction
        # Static friction is relevant only when tank is turning slowly enough
        if abs((Fr-Fl)*B/2) < Sl and abs(self.spin) < 2.:
            # Not turning
            # Solve diff. eq. to get non-linear function for speed, v(t)
            # dv/dt = (Fl+Fr-Br*v)/m
            # dt = m*dv/(Fl+Fr-Br*v)
            # t = -m/Br*log(Fl+Fr-Br*v) + C
            # Using initial condition v(0) = v0
            # C = m/Br*log(Fl+Fr-Br*v0)
            # t = m/Br*log(Fl+Fr-Br*v0) - m/B*log(Fl+Fr-Br*v)
            # t = -m/Br*log((Fl+Fr-Br*v)/(Fl+Fr-Br*v0))
            # v = (Fl+Fr)/Br - (v0-(Fl+Fr)/Br)*exp(-Br*t/m)
            v0 = self.speed
            v1 = (Fr + Fl) / Br  # steady-state speed
            tauv = m/Br
            self.speed = v1 - (v1 - v0) * np.exp(-dt/tauv)
            # Integrate to get path length
            # v = dr/dt = (Fl+Fr)/Br - (v0-(Fl+Fr)/Br)*exp(-Br*t/m)
            # r = t*(Fl+Fr)/Br + m/Br*(v0-(Fl+Fr)/Br)*exp(-Br*t/m) + C
            # Using initial condition r(0) = 0
            # C = -m/Br*(v0-(Fl+Fr)/Br)
            # r = t*(Fl+Fr)/Br + m/Br*(v0-(Fl+Fr)/Br)*(exp(-Br*t/m)-1)
            r = v1*dt - tauv * (v1 - v0) * (1 - np.exp(-dt/tauv))
            # Rotate path length into coordinate frame
            a = self.orientation[0]*pi/180
            self.position[0] += r * np.cos(a)
            self.position[1] += r * np.sin(a)
            self.spin = 0.
            # Calculate each tread rotation
            # vl = vr = v
            # dl = dr = r
            self.tread_offset[0] += r
            self.tread_offset[1] += r
        else:
            # Is turning
            # dv/dt = (Fl+Fr-(Br+Bs)*v)/m
            # Substitute (Br+Bs) for Br in non-turning equations
            # v = (Fl+Fr)/(Br+Bs) -
            #     (v0-(Fl+Fr)/(Br+Bs))*exp(-(Br+Bs)*t/m)
            # r = t*(Fl+Fr)/(Br+Bs) +
            #     m/(Br+Bs)*(v0-(Fl+Fr)/Br)*(exp(-(Br+Bs)*t/m)-1)
            v0 = self.speed
            v1 = (Fr + Fl) / (Br + Bs)
            tauv = m/(Br+Bs)
            self.speed = v1 - (v1 - v0) * np.exp(-dt/tauv)
            r = v1*dt - tauv * (v1 - v0) * (1 - np.exp(-dt/tauv))
            # Solve diff. eq. to get angular rate, w
            # dw/dt = ((Fl-Fr)*B/2 - Bl*w)/J
            # dt = J*dw/(B/2*(Fl-Fr)-Bl*w)
            # t = -J/Bl*log(B*(Fl-Fr)-2*Bl*w) + C
            # Using initial condition w(0) = w0
            # C = J/Bl*log(B*(Fl-Fr)-2*Bl*w0)
            # t = -J/Bl*log((B*(Fl-Fr)-2*Bl*w)/(B*(Fl-Fr)-2*Bl*w0))
            # w = B/2*(Fl-Fr)/Bl-(B/2*(Fl-Fr)/Bl-w0)*exp(-Bl*t/J)
            w1 = B/2*(Fr-Fl)/Bl
            w0 = self.spin * pi/180
            tauw = J/Bl
            self.spin = (w1 - (w1 - w0) * np.exp(-dt/tauw)) * 180/pi
            # Integrate to get change in orientation, a
            # w = da/dt = (B/2*(Fl-Fr)-(B/2*(Fl-Fr)-Bl*w0)*exp(-Bl*t/J))/Bl
            # a = (B/2*(Fl-Fr)*t +
            #     (B/2*(Fl-Fr)/Bl*J-w0*J)*exp(-Bl*t/J))/Bl
            theta0 = self.orientation[0] * pi/180
            dtheta = w1*dt - tauw * (w1 - w0) * (1 - np.exp(-dt/tauw))
            self.orientation[0] += dtheta * 180/pi
            # Calculate for x and y displacement
            # dx/dt = v(t) * cos(a(t)) : differs in convention from Nutaro
            # A0 = (Fl+Fr)/(Br+Bs)
            # A1 = B/2*(Fl-Fr)/Bl
            # dx/dt = (A0 - (v0-A0)*exp(-(Br+Bs)*t/m)) *
            #         cos(A1*t + (A1/Bl*J-w0*J/Bl)*exp(-Bl*t/J))
            # dx/dt may not be analytically integrable
            # but we can solve the problem numerically
            # dy/dt = v(t) * sin(a(t))

            def dxdt(t):
                return ((v1 - (v1-v0)*np.exp(-t/tauv)) *
                        np.cos(theta0 + w1*t -
                               tauw*(w1-w0)*(1-np.exp(-t/tauw))))

            def dydt(t):
                return ((v1 - (v1-v0)*np.exp(-t/tauv)) *
                        np.sin(theta0 + w1*t -
                               tauw*(w1-w0)*(1-np.exp(-t/tauw))))

            x, err = quad(dxdt, 0, dt)
            y, err = quad(dydt, 0, dt)
            self.position[0] += x
            self.position[1] += y

            # Calculate each tread rotation
            self.tread_offset[0] += r - dtheta*B/2
            self.tread_offset[1] += r + dtheta*B/2

        # Turret can spin if already spinning or if force can break sticking
        if abs(Ft) > St or self.turret_spin > 2.:
            # Turret can turn
            # Following equations for tank body turning
            # TODO add forces from turning of body
            w1 = Ft/Bt
            w0 = self.turret_spin * pi/180
            tauw = Jt/Bt
            self.turret_spin = (w1 - (w1 - w0) * np.exp(-dt/tauw)) * 180/pi
            theta0 = self.orientation[1] * pi/180
            self.orientation[1] += (w1*dt - tauw * (w1 - w0) *
                                    (1 - np.exp(-dt/tauw))) * 180/pi

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
