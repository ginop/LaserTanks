#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May 31 10:49:49 2017

@author: robertperrotta
"""

import numpy as np
import pygame as pg
from pygame import gfxdraw
from math import pi
import testControls

d2r = pi/180

px = 25  # pixels per game unit
(screen_width, screen_height) = (30, 20)  # arena size in game units

screen = pg.display.set_mode((screen_width*px, screen_height*px))
pg.display.set_caption("Laser Tanks!")
pg.init()
myfont = pg.font.SysFont("monospace", 12)


def rotate(points, angle):
    ca = np.cos(angle*pi/180)
    sa = np.sin(angle*pi/180)
    R = np.array([[[ca, -sa], [sa, ca]]])
    points.shape += (1,)
    points = (R*points).sum(axis=1)
    return points


def text(str, pos, color, centered=False):
    pos = np.array(pos)
    pos[1] = screen_height - pos[1]
    pos *= px
    label = myfont.render(str, 1, color)
    if centered:
        r = label.get_rect()
        pos -= [r.width/2., r.height/2.]
    pos = pos.round().astype(int)
    screen.blit(label, pos)


def line(points, color):
    points[:, 1] = screen_height - points[:, 1]
    pg.draw.aaline(screen, color, px*points[0, :], px*points[1, :])


def polygon(points, color, edge_color=None):
    if edge_color is None:
        edge_color = color
    # Invert Y coordinate so up is positive (PyGame uses down as positive)
    points[:, 1] = screen_height - points[:, 1]
    gfxdraw.filled_polygon(screen, (px*points).round().astype(int), color)
    gfxdraw.aapolygon(screen, (px*points).round().astype(int), edge_color)


def circle(center, radius, color, edge_color=None):
    if edge_color is None:
        edge_color = color
    gfxdraw.filled_circle(screen, int(round(px*center[0])),
                          int(round(px*(screen_height-center[1]))),
                          int(round(px*radius)), color)
    gfxdraw.aacircle(screen, int(round(px*center[0])),
                     int(round(px*(screen_height-center[1]))),
                     int(round(px*radius)), edge_color)


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

    laser_dur = 0.5  # duration of laser shot in seconds
    damage = 100  # HP per sec
    reload_time = 1

    def __init__(self, control, color, pos=[0., 0.], orient=[0., 0.]):
        self.control = control
        self.position = np.array(pos).astype(float)
        self.orientation = np.array(orient)
        self.velocity = np.array([0, 0]).astype(float)
        self.drive = np.array([0., 0.])  # L and R tread speed in units/sec
        self.tread_offset = np.array([0., 0.])  # for animating treads
        self.spin = 0.  # commanded turret spin in degrees per sec
        self.shoot = False  # command to fire laser
        self.time_to_ready = 0.  # counts down from reload_time after each shot
        self.hull = 100
        self.battery = 100
        self.color = color
        self.shield = None

    def draw(self):
        # Specify tank shape centered on origin with 0 rotation (pointed up)
        u = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])/2
        body = u * [Tank.body_width, Tank.body_length]
        tread_r = (u * [Tank.tread_width, Tank.tread_length] +
                   [Tank.body_width/2+Tank.tread_width/2-Tank.tread_overlap, 0])
        tread_l = -tread_r
        barrel = (u * [Tank.barrel_width, Tank.barrel_length] +
                  [0, Tank.barrel_length/2])

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
        polygon(tread_r, self.color)
        polygon(tread_l, self.color)

        # Draw tread lines
        for offset, sign in zip(self.tread_offset, [1, -1]):
            n = 5
            for ii in range(n):
                y = (Tank.tread_length*ii/n + offset) % Tank.tread_length
                y -= Tank.tread_length/2
                pts = np.array([[sign*(Tank.body_width/2+Tank.tread_width-Tank.tread_overlap), y],
                                 [sign*(Tank.body_width/2-Tank.tread_overlap), y]])
                pts = rotate(pts, self.orientation[0])
                pts += self.position
                line(pts, (0, 0, 0))

        polygon(body, (0, 0, 0))
        polygon(barrel, self.color)
        circle(self.position, 0.6, self.color)

        # print HP on tank turret
        text("{}".format(self.hull), self.position, (0, 0, 0), centered=True)

        # draw laser if shooting (indicated by time_to_read above reload_time)
        if self.time_to_ready > self.reload_time:
            laser = (u * [Tank.laser_width, 1000] +
                  [0, Tank.barrel_length + 0.5 + 500])
            laser = rotate(laser, self.orientation[0]+self.orientation[1])
            laser += self.position
            polygon(laser, self.color, (255, 255, 255))


    def update(self, dt, t):
        self.drive, self.spin, self.shoot = self.control(t, Tank)
        
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
            self.position[0] += dt*v*np.sin(a)
            self.position[1] += dt*v*np.cos(a)
        else:
            self.position[0] -= v/w*(np.cos(a+w*dt)-np.cos(a))
            self.position[1] += v/w*(np.sin(a+w*dt)-np.sin(a))
            self.orientation[0] += w*dt*180/pi


R = Tank(testControls.R, (200, 25, 0), [26., 11.], [-45., -45.])
G = Tank(testControls.G, (0, 195, 25), [3., 3.], [24., 11.])
B = Tank(testControls.B, (0, 50, 255), [12., 16.], [95., 15.])

tanks = [R, G, B]

clock = pg.time.Clock()
running = True
fps = 60
time = 0

while running:

    clock.tick(fps)  # limit fps
    dt = clock.get_time()/1000
    time += dt

    # End loop when window is closed
    for event in pg.event.get():
        # print(event)
        if event.type == pg.QUIT:
            running = False

    # set the scene
    screen.fill((50, 50, 50))
    for x in range(screen_width):
        for y in range(screen_height):
            pg.draw.circle(screen, (75, 75, 75),
                           (round(px*(x+1/2)),
                            round(px*(y+1/2))), round(px/10))

    [T.update(dt, time) for T in tanks]
    [T.move(dt) for T in tanks]
    [T.draw() for T in tanks if T.hull > 0]

    actual_fps = clock.get_fps()
    text("FPS: {:.2f}".format(actual_fps),
         (1/4, screen_height-1/4), (255, 255, 255))

    pg.display.flip()

pg.quit()
quit()
