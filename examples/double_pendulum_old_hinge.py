import colorsys
import struct

import mollia_window

import mollia_bullet as mb
from mollia_bullet import transform as t
from _mollia_bullet.renderer import Renderer


def move(x = 0.0, y = 0.0, z = 0.0):
    return mb.transform((x, y, z))


def sphere(radius):
    return {'type': 'sphere', 'radius': radius}


def capsule(radius, height):
    return {'type': 'capsule', 'radius': radius, 'height': height * 2.0}


world = mb.world(gravity=(0.0, 0.01, -10.0))

ball = world.rigid_body(mass=0.0, shape=sphere(0.1), transform=move(z=2.0), group=0, mask=0)
capsule_1 = world.rigid_body(mass=1.0, shape=capsule(0.05, 0.3), transform=move(z=2.3), group=0, mask=0)
capsule_2 = world.rigid_body(mass=1.0, shape=capsule(0.05, 0.3), transform=move(z=2.9), group=0, mask=0)

constraint_1 = world.hinge(ball, capsule_1, pivot=(0.0, 0.0, 2.0), axis=(1.0, 0.0, 0.0))
constraint_2 = world.hinge(capsule_1, capsule_2, pivot=(0.0, 0.0, 2.6), axis=(1.0, 0.0, 0.0))

wnd = mollia_window.main_window()
renderer = Renderer(world)

renderer.set_camera((5.0, 4.0, 3.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))

colors = [colorsys.hls_to_rgb(i / 9.0, 0.5, 0.5) for i in range(64)]
renderer.texture.write(b''.join(struct.pack('BBBB', int(r * 255.0), int(g * 255.0), int(b * 255.0), 255) for r, g, b in colors))

while mollia_window.update():
    world.update()
    renderer.render()
