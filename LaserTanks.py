"""Define the Laser Tanks Game class

"""
from math import pi
import numpy as np  # barely used. replace with something simpler?
import pygame as pg
from pygame import freetype
from pygame import gfxdraw
import pymunk
from Tank import Tank
from threading import Timer, Lock
from time import sleep


def do_nothing(*args, **kwargs):
    """Do nothing.

    A placeholder for function-call hooks in a Game.
    """
    pass


class _Task():
    """Manage fixed rate execution scheduling."""

    def __init__(self, period, callback, *args):
        self.period = period
        self.timer = None
        self.callback = callback
        self.args = args
        self.lock = Lock()

    def run(self):
        with self.lock:
            self.timer = Timer(self.period, self.run)
            self.timer.daemon = True  # Don't keep python running for this task
            self.timer.start()

            self.callback(*self.args)
            """try:
                self.callback(*self.args)
            except:
                print('Task caused error. Terminating now.')
                self.quit()
                raise"""

    def quit(self):
        self.lock.acquire()
        if self.timer is not None:
            self.timer.cancel()

    def __enter__(self, *args, **kwargs):
        self.run()

    def __exit__(self, *args, **kwargs):
        self.quit()


class Game():
    """Manage Laser Tanks game components, simulation, and visualization.

    Properties:
      tanks: a list of Tank objects that will compete in the game
      px: ratio of pixels to in-game length units
      screen_width: arena width in in-game length units
      screen_height: arena height in in-game length units
      real_time: When true, simulation is limited to run in real time.
      fps: the requested update rate (frames per second) of the pygame
           visualization
      dt: the step size of the pymunk simulation
      pre_step, post_step: hooks for callback functions run just before and
                           after each step of the simulation
      draw: visualization flag. Set to false to run simulation without visuals
      max_time: simulation time at which simulation ends if not already done
      end_on_win: flag that determines whether or not game exits when only one
                  tank remains
    """

    def __init__(self, tanks=[], px=2, screen_width=400, screen_height=400,
                 real_time=True, fps=60, dt=0.01, pre_step=do_nothing,
                 post_step=do_nothing, draw=True, max_time=np.inf,
                 end_on_win=True):
        self.px = px  # pixels per game unit
        self.screen_width = screen_width  # in game units
        self.screen_height = screen_height  # in game units
        self.real_time = real_time
        self.fps = fps
        self.dt = dt
        self.pre_step = pre_step
        self.post_step = post_step
        self.draw = draw
        self.max_time = max_time
        self.end_on_win = end_on_win

        # Set up pymunk space and add tank parts
        self.space = pymunk.Space()
        self.tanks = tanks
        for tank in tanks:
            tank.game = self
            self.space.add(tank.body, tank.box, tank.turret_body, tank.turret_circle, tank.pivot, tank.pivot_friction)

        # add walls
        # TODO: put this in a loop or function to reduce duplicate code
        fric = 0.6
        wall_width = 10  # wider (thicker) walls reduce chances of escape

        wall = pymunk.Body(body_type=pymunk.Body.STATIC)
        line = pymunk.Segment(wall,
                              (-wall_width, 0),
                              (-wall_width, self.screen_height),
                              wall_width)
        line.friction = fric
        self.space.add(wall, line)

        wall = pymunk.Body(body_type=pymunk.Body.STATIC)
        line = pymunk.Segment(wall,
                              (0, -wall_width),
                              (self.screen_height, -wall_width),
                              wall_width)
        line.friction = fric
        self.space.add(wall, line)

        wall = pymunk.Body(body_type=pymunk.Body.STATIC)
        line = pymunk.Segment(wall,
                              (0, self.screen_height + wall_width),
                              (self.screen_height, self.screen_height + wall_width),
                              wall_width)
        line.friction = fric
        self.space.add(wall, line)

        wall = pymunk.Body(body_type=pymunk.Body.STATIC)
        line = pymunk.Segment(wall,
                              (self.screen_height + wall_width, 0),
                              (self.screen_height + wall_width, self.screen_height),
                              wall_width)
        line.friction = fric
        self.space.add(wall, line)

        # Initialize game state params
        self.time = 0
        self.runtime = 0
        self.winner = None
        self.running = False
        self.screen = []
        if self.draw:
            # Use DOUBLEBUF to speed up pygame drawing
            self.screen = pg.display.set_mode((self.screen_width * self.px,
                                               self.screen_height * self.px),
                                              pg.DOUBLEBUF | pg.ASYNCBLIT)
            self.screen.set_alpha(None)  # For optimization purposes
            pg.display.set_caption("Laser Tanks!")  # The window title
            pg.init()  # must be called before pg.font.SysFont()
            self.font = freetype.SysFont("monospace", 12)

    def text(self, str, pos, color, centered=False):
        """Print text to PyGame screen.

        Inputs:
            str: the text to print
            pos: the position in game units of top left corner (or center)
            color: a 3 or 4 tuple, RGB(A), of integers in [0, 255]
            centered: interpret pos as text center, not corner
        """
        pos = np.array(pos)
        pos[1] = self.screen_height - pos[1]
        pos *= self.px
        if centered:
            r = self.font.get_rect(str)
            pos -= [r.width / 2., r.height / 2.]
        pos = pos.astype(int)  # cast as int rather than round for speed
        self.font.render_to(self.screen, pos, str, fgcolor=color)

    def line(self, points, color):
        """Print antialiased line, handling conversion from game units to pixels.

        Inputs:
            points: 2x2 numpy array of line end-points [[x1, y1], [x2, y2]]
            color: a 3 or 4 tuple, RGB(A), of integers in [0, 255]
        """
        points[:, 1] = self.screen_height - points[:, 1]
        pg.draw.aaline(self.screen, color,
                       self.px*points[0, :], self.px*points[1, :])

    def polygon(self, points, color, edge_color=0):
        """Print a polygon with an antialiased edge, handling conversion from
        game units to pixels.

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
                                   (self.px*points).astype(int), color)
        if edge_color is not None:
            gfxdraw.aapolygon(self.screen,
                              (self.px*points).astype(int), edge_color)

    def circle(self, center, radius, color, edge_color=0):
        """Print a circle with an antialiased edge, handling conversion from game
        units to pixels.

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
            gfxdraw.filled_circle(self.screen, int(self.px * center[0]),
                                  int(self.px * (self.screen_height - center[1])),
                                  int(self.px * radius), color)
        if edge_color is not None:
            gfxdraw.aacircle(self.screen, int(self.px * center[0]),
                             int(self.px * (self.screen_height - center[1])),
                             int(self.px * radius), edge_color)

    def update_tanks(self):
        # Update time for new simulation epoch
        self.time += self.dt
        # Call hook for pre-step functions
        self.pre_step()
        # Update tanks
        public_info = [[them.public() for them in self.tanks
                        if them is not me] for me in self.tanks]
        [T.update(self.dt, self.time, P) for T, P in zip(self.tanks, public_info)]
        [T.apply_forces() for T in self.tanks]
        self.space.step(self.dt)  # pymunk magic
        [T.move(self.dt) for T in self.tanks]
        self.detect_hits(self.dt)
        # Call hook for post-step functions
        self.post_step()

    def update_visuals(self):
        # set the scene
        self.screen.fill((50, 50, 50))
        # A grid of dots
        for x in range(0, self.screen_width, 10):
            for y in range(0, self.screen_height, 10):
                pg.draw.circle(self.screen, (75, 75, 75),
                               (int(self.px * (x + 1 / 2)),
                                int(self.px * (y + 1 / 2))),
                               int(self.px / 5))

        [T.draw() for T in self.tanks]
        [T.draw_laser() for T in self.tanks]

        str = "Time: {:.2f} ({:.2f}x real time) {:.0f} fps"
        str = str.format(self.time,
                         self.time * 1e3 / (pg.time.get_ticks() - self.starttime),
                         self.clock.get_fps())
        self.text(str, (2, self.screen_height - 2), (255, 255, 255))

        pg.display.flip()
        self.clock.tick()

    def run(self):
        self.clock = pg.time.Clock()

        if self.draw:
            self.screen.fill((50, 50, 50))
            [T.draw() for T in self.tanks]
            pg.display.flip()

        self.starttime = pg.time.get_ticks()

        if self.draw:
            action = self.update_visuals
        else:
            action = do_nothing

        with _Task(1 / self.fps, action):
            self.running = True
            while self.running:

                if self.draw:
                    # End loop when window is closed
                    for event in pg.event.get():
                        # print(event)
                        if event.type == pg.QUIT:
                            print('PyGame received Quit command.')
                            self.running = False

                # If applicable, limit to real-time by skipping rest of loop
                if self.real_time and \
                   (pg.time.get_ticks() - self.starttime) / 1e3 < self.time:
                    sleep(self.dt / 10)
                    continue

                self.update_tanks()

                # Check for timeout
                if self.time > self.max_time:
                    self.running = False

                # Check for winner
                if len(self.tanks) == 1:
                    self.winner = self.tanks[0]
                    if self.end_on_win:
                        self.running = False

    def quit(self):
        pg.display.quit()
        pg.quit()

    def detect_hits(self, dt):
        """Detect lasers hitting targets.

        For each tank, search for a hit on each other tanks. Keep only the
        first (closest) hit. Apply damage to hit tank.
        """
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
                    hit.hull -= dt * shooter.damage
                    if hit.hull <= 0:
                        self.tanks.remove(hit)
                    shooter.laser_length = laser_length
                else:  # back to default, really-long laser_length
                    shooter.laser_length = Tank.laser_length


if __name__ == "__main__":

    screen_width = 400
    screen_height = 400

    R = Tank('Waypoint_PID_Controller', (200, 25, 0),
             [screen_width / 8, screen_height / 2],
             [-pi / 2, -pi / 2])
    # Replace random waypoint generator with corner track waypoint generator
    R.control.waypoint_gen = R.control.waypoint()
    R.control.destination = next(R.control.waypoint_gen)

    B = Tank('Waypoint_PID_Controller', (0, 50, 255),
             [screen_width * 3 / 4, screen_height / 2],
             [3 * pi / 2, 3 * pi / 2])

    game = Game(tanks=[R, B], screen_width=screen_width,
                screen_height=screen_height, real_time=True,
                fps=60, dt=1 / 100, end_on_win=False)

    game.run()
    game.quit()
