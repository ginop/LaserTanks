import numpy as np
from LaserTankUtilities import rotate

class Obstacle():

    def __init__(self, pos=[0., 0.], width=0.0, height=0.0, angle=0.0):
        self.game = None
        self.position = np.array(pos)
        self.width = width
        self.height = height
        self.angle = angle

        # Determine hitbox for tank (just use body, not treads, for simplicity)
        u = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])/2
        hitbox = u * [self.width, self.height]
        hitbox = rotate(hitbox, self.angle)
        hitbox += self.position
        self.hitbox = hitbox

    def detect_hit(self, laser):
        """
        Inputs:
            laser: a tuple of origin and angle (x, y, angle)
        Outputs:
            dist: distance from laser origin to impact point, None if no impact
        """
        hitbox = self.hitbox
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

    def draw(self):
        self.game.polygon(self.hitbox, (0, 0, 0))
