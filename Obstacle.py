import numpy as np
import pymunk
from pymunk.vec2d import Vec2d

class Obstacle:

    friction = 0.6

    def __init__(self, points):
        self.game = None
        self.points = np.array(points)
        self.body = pymunk.Body(body_type=pymunk.Body.STATIC)
        self.poly = pymunk.Poly(self.body, points, radius=0.1)
        self.poly.friction = Obstacle.friction

    def draw(self):
        self.game.polygon(self.points, (0, 0, 0))

    def detect_hit(self, laser):
        """
        Inputs:
            laser: a tuple of origin and angle (x, y, angle)
        Outputs:
            dist: distance from laser origin to impact point, None if no impact
        """

        start = Vec2d(laser[:2])
        end = start + 99999 * Vec2d([np.cos(laser[2]), np.sin(laser[2])])
        info = self.poly.segment_query(start, end)
        if info.shape:  # it hit something
            return (info.point - start).get_length()
        else:
            return None

    @staticmethod
    def generate_rock(x, y, size, num_points=16, jaggedness=0.2):
        theta = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
        theta += np.random.rand(num_points) * jaggedness * (theta[1] - theta[0])
        r = size * (1 - np.random.rand(num_points) * jaggedness / 2)
        points = r * [x + np.cos(theta), y + np.sin(theta)]
        return Obstacle(points.T.tolist())
