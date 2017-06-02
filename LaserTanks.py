#!python3

import numpy as np
import pygame as pg
from math import pi
# gfxdraw is a submodule (a child directory in the pygame folder) so it must be
# specifically imported. from pygame import * would not include it.
from pygame import gfxdraw
from Tank import Tank


class Game():

    def __init__(self, tanks=[], px=20, screen_width=40, screen_height=40):
        for tank in tanks:
            tank.game = self
        self.tanks = tanks

        self.px = px  # pixels per game unit
        self.screen_width = screen_width  # in game unites
        self.screen_height = screen_height  # in game units

        self.screen = pg.display.set_mode((self.screen_width*self.px,
                                           self.screen_height*self.px))
        pg.display.set_caption("Laser Tanks!")  # The window title
        pg.init()  # must be called before pg.font.SysFont()

    def text(self, str, pos, color, centered=False):
        """
        PyGame label printing function
        Inputs:
            str: the text to print
            pos: the position in game units of top left corner (or centered)
            color: a 3 or 4 tuple, RGB(A), of integers in [0, 255]
            centered: interpret pos as text center, not corner
        """
        pos = np.array(pos)
        pos[1] = self.screen_height - pos[1]
        pos *= self.px
        font = pg.font.SysFont("monospace", 12)
        label = font.render(str, 1, color)
        if centered:
            # TODO: Better text centering
            # Using the Rect doesn't seem to make the text exactly centered
            # It is close, but can we do better?
            r = label.get_rect()
            pos -= [r.width/2., r.height/2.]
        pos = pos.round().astype(int)
        self.screen.blit(label, pos)

    def line(self, points, color):
        """
        PyGame line printing helper function
        Prints antialiased line, handling conversion from game units to pixels
        Inputs:
            points: 2x2 numpy array of line end-points [[x1, y1], [x2, y2]]
            color: a 3 or 4 tuple, RGB(A), of integers in [0, 255]
        """
        points[:, 1] = self.screen_height - points[:, 1]
        pg.draw.aaline(self.screen, color,
                       self.px*points[0, :], self.px*points[1, :])

    def polygon(self, points, color, edge_color=0):
        """
        PyGame polygon printing helper function
        Prints a polygon with an antialiased edge, handling conversion from
        game units to pixels
        Inputs:
            points: 2x2 numpy array of line end-points [[x1, y1], [x2, y2]]
            color: a 3 or 4 tuple, RGB(A), of integers in [0, 255]
            edge_color: a 3 or 4 tuple, RGB(A), of integers in [0, 255]; if
                        ommitted, edge_color matches color
        """
        if edge_color is 0:
            edge_color = color
        # Invert Y coordinate so up is positive (PyGame uses down as positive)
        points[:, 1] = self.screen_height - points[:, 1]
        if color is not None:
            gfxdraw.filled_polygon(self.screen,
                                   (self.px*points).round().astype(int), color)
        if edge_color is not None:
            gfxdraw.aapolygon(self.screen,
                              (self.px*points).round().astype(int), edge_color)

    def circle(self, center, radius, color, edge_color=0):
        """
        PyGame circle printing helper function
        Prints a circle with an antialiased edge, handling conversion from game
        units to pixels
        Inputs:
            center: sequence of x and y coordinates of circle center
            radius: radius of the circle
            color: a 3 or 4 tuple, RGB(A), of integers in [0, 255]
            edge_color: a 3 or 4 tuple, RGB(A), of integers in [0, 255]; if
                        ommitted, edge_color matches color
        """
        if edge_color is 0:
            edge_color = color
        if color is not None:
            gfxdraw.filled_circle(self.screen, int(round(self.px*center[0])),
                                  int(round(self.px *
                                            (self.screen_height-center[1]))),
                                  int(round(self.px*radius)), color)
        if edge_color is not None:
            gfxdraw.aacircle(self.screen, int(round(self.px*center[0])),
                             int(round(self.px *
                                       (self.screen_height-center[1]))),
                             int(round(self.px*radius)), edge_color)

    # TODO: gfxdraw.arc doesn't have a thickness according to the docs and
    #       pg.draw.arc is ugly. Can we do better?
    """def arc(self, center, radius, start, stop, color):
        gfxdraw.arc(self.screen, int(round(self.px * center[0])),
                    int(round(self.px * (self.screen_height - center[1]))),
                    int(round(self.px * radius)), int(round(start*pi/180)),
                    int(round(stop*pi/180)), color)"""

    def run(self):
        clock = pg.time.Clock()
        running = True
        fps = 60
        time = 0

        fps = 60
        while running:

            clock.tick(fps)  # limit fps
            dt = clock.get_time()/1000  # convert msec to sec
            time += dt

            # End loop when window is closed
            for event in pg.event.get():
                # print(event)
                if event.type == pg.QUIT:
                    running = False

            # set the scene
            self.screen.fill((50, 50, 50))
            for x in range(self.screen_width):
                for y in range(self.screen_height):
                    pg.draw.circle(self.screen, (75, 75, 75),
                                   (round(self.px*(x+1/2)),
                                    round(self.px*(y+1/2))), round(self.px/10))

            public_info = [[them.public() for them in self.tanks
                            if them is not me] for me in self.tanks]
            [T.update(dt, time, P) for T, P in zip(self.tanks, public_info)]
            [T.move(dt) for T in self.tanks]
            self.detect_hits(dt)
            [T.draw() for T in self.tanks]
            [T.draw_laser() for T in self.tanks]
            actual_fps = clock.get_fps()
            self.text("FPS: {:.2f}".format(actual_fps),
                      (1/4, self.screen_height-1/4), (255, 255, 255))

            pg.display.flip()

        pg.display.quit()
        pg.quit()
        # quit()

    def detect_hits(self, dt):
        """ For each tank, search for a hit on each other tank. Keep only the
        first (closest) hit."""
        for shooter in self.tanks:
            laser = shooter.get_beam()
            if laser is not None:
                targets = [tank for tank in self.tanks if tank is not shooter]
                dist = []
                hit = []
                for target in targets:
                    x = target.detect_hit(laser)
                    if x is not None:
                        dist.append(x)
                        hit.append(target)
                if len(dist) > 0:
                    laser_length = min(dist)
                    hit = hit[dist.index(laser_length)]
                    hit.hull -= dt*shooter.damage
                    if hit.hull <= 0:
                        self.tanks.remove(hit)
                    shooter.laser_length = laser_length
                else:  # back to default, really-long laser_length
                    shooter.laser_length = Tank.laser_length


if __name__ == "__main__":

    screen_width = 40
    screen_height = 40
    R = Tank('FixedOffset_PIDTracking_Controller', (200, 25, 0),
             [screen_width/4, screen_height/2], [90, 0])
    B = Tank('FixedOffsetController', (0, 50, 255),
             [screen_width*3/4, screen_height/2], [90, 0])
    game = Game(tanks=[R, B], screen_width=screen_width,
                screen_height=screen_height)

    game.run()
