import colorsys
import struct

import mollia_window

import mollia_bullet as mb
from _mollia_bullet.renderer import Renderer

defaults = dict(friction=0.5, spinning_friction=0.05, rolling_friction=0.05)

world = mb.world(gravity=(0.0, 0.0, -10.0))
ground = world.rigid_body(mass=0.0, shape={'type': 'box', 'size': [20.0, 20.0, 20.0]}, transform=mb.transform((0.0, 0.0, -10.001)), **defaults)

box_shape = world.rigid_body(mass=1.0, shape={'type': 'box', 'size': [0.2, 0.2, 0.2]}, transform=mb.transform((0.0, 0.0, 2.1)), **defaults)
sphere_shape = world.rigid_body(mass=1.0, shape={'type': 'sphere', 'radius': 0.15}, transform=mb.transform((0.1, 0.0, 1.3)), **defaults)
capsule_shape = world.rigid_body(mass=1.0, shape={'type': 'capsule', 'radius': 0.1, 'height': 0.3}, transform=mb.transform((0.0, 0.1, 1.7)), **defaults)
cylinder_shape = world.rigid_body(mass=1.0, shape={'type': 'cylinder', 'radius': 0.2, 'height': 0.05}, transform=mb.transform((0.0, -0.1, 2.3)), **defaults)
cone_shape = world.rigid_body(mass=1.0, shape={'type': 'cone', 'radius': 0.3, 'height': 0.3}, transform=mb.transform((0.2, 0.0, 2.6)), **defaults)
convexhull_shape = world.rigid_body(mass=1.0, shape={
    'type': 'convexhull',
    'vertices': [
        [-0.15, -0.15, -0.15],
        [0.3, 0.0, 0.0],
        [0.0, 0.3, 0.0],
        [0.0, 0.0, 0.3],
    ],
}, transform=mb.transform((0.1, 0.1, 2.8)), **defaults)
multisphere_shape = world.rigid_body(mass=1.0, shape={
    'type': 'multisphere',
    'spheres': [
        [-0.15, -0.15, -0.15, 0.05],
        [0.3, 0.0, 0.0, 0.1],
        [0.0, 0.3, 0.0, 0.05],
        [0.0, 0.0, 0.3, 0.1],
    ],
}, transform=mb.transform((-0.1, 0.0, 3.0)), **defaults)
minkowski_shape = world.rigid_body(mass=1.0, shape={
    'type': 'minkowski',
    'a': {
        'transform': {
            'position': [0.0, 0.0, 0.0],
            'rotation': [0.0, 0.0, 0.0, 1.0],
        },
        'shape': {'type': 'box', 'size': [0.2, 0.2, 0.1]},
    },
    'b': {
        'transform': {
            'position': [0.0, 0.0, 0.0],
            'rotation': [0.0, 0.0, 0.0, 1.0],
        },
        'shape': {'type': 'sphere', 'radius': 0.1},
    },
}, transform=mb.transform((0.0, 0.0, 3.4)), **defaults)
compound_shape = world.rigid_body(mass=1.0, shape={
    'type': 'compound',
    'children': [
        {
            'transform': {
                'position': [0.0, 0.0, 0.0],
                'rotation': [0.0, 0.0, 0.0, 1.0],
            },
            'shape': {'type': 'box', 'size': [0.5, 0.1, 0.1]},
        },
        {
            'transform': {
                'position': [0.0, 0.0, 0.0],
                'rotation': [0.0, 0.0, 0.0, 1.0],
            },
            'shape': {'type': 'box', 'size': [0.1, 0.5, 0.1]},
        },
        {
            'transform': {
                'position': [0.0, 0.0, 0.0],
                'rotation': [0.0, 0.0, 0.0, 1.0],
            },
            'shape': {'type': 'box', 'size': [0.1, 0.1, 0.5]},
        },
    ],
}, transform=mb.transform((0.0, 0.1, 3.4)), **defaults)

wnd = mollia_window.main_window()
renderer = Renderer(world)

renderer.set_camera((5.0, 4.0, 3.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))

colors = [colorsys.hls_to_rgb(i / 9.0, 0.5, 0.5) for i in range(64)]
renderer.texture.write(b''.join(struct.pack('BBBB', int(r * 255.0), int(g * 255.0), int(b * 255.0), 255) for r, g, b in colors))

while mollia_window.update():
    world.update()
    renderer.render()
