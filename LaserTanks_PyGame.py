import numpy as np
import pygame as pg
# gfxdraw is a submodule (a child directory in the pygame folder) so it must be
# specifically imported. from pygame import * would not include it.
from pygame import gfxdraw
import testControls
from Tank import Tank


class Game():

    px = 20  # pixels per game unit
    screen_width = 40  # in game unites
    screen_height = 40  # in game units

    def __init__(self, tanks=[]):
        for tank in tanks:
            tank.game = self
        self.tanks = tanks
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

    def polygon(self, points, color, edge_color=None):
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
        if edge_color is None:
            edge_color = color
        # Invert Y coordinate so up is positive (PyGame uses down as positive)
        points[:, 1] = self.screen_height - points[:, 1]
        gfxdraw.filled_polygon(self.screen,
                               (self.px*points).round().astype(int), color)
        gfxdraw.aapolygon(self.screen,
                          (self.px*points).round().astype(int), edge_color)

    def circle(self, center, radius, color, edge_color=None):
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
        if edge_color is None:
            edge_color = color
        gfxdraw.filled_circle(self.screen, int(round(self.px*center[0])),
                              int(round(self.px*(self.screen_height-center[1]))),
                              int(round(self.px*radius)), color)
        gfxdraw.aacircle(self.screen, int(round(self.px*center[0])),
                         int(round(self.px*(self.screen_height-center[1]))),
                         int(round(self.px*radius)), edge_color)

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
            [T.draw() for T in self.tanks if T.hull > 0]

            actual_fps = clock.get_fps()
            self.text("FPS: {:.2f}".format(actual_fps),
                      (1/4, self.screen_height-1/4), (255, 255, 255))

            pg.display.flip()

        pg.display.quit()
        pg.quit()
        quit()


if __name__ is "__main__":
    R = Tank(testControls.R, (200, 25, 0), [26., 11.], [-45., -45.])
    G = Tank(testControls.G, (0, 195, 25), [3., 3.], [24., 11.])
    B = Tank(testControls.B, (0, 50, 255), [12., 16.], [95., 15.])
    game = Game(tanks=[R, G, B])
    game.run()
