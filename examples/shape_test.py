import mollia_bullet

world = mollia_bullet.world()
box_shape = world.rigid_body(shape={'type': 'box', 'size': [1.0, 2.0, 3.0]})
sphere_shape = world.rigid_body(shape={'type': 'sphere', 'radius': 1.0})
capsule_shape = world.rigid_body(shape={'type': 'capsule', 'radius': 1.0, 'height': 2.0})
cylinder_shape = world.rigid_body(shape={'type': 'cylinder', 'radius': 1.0, 'height': 2.0})
cone_shape = world.rigid_body(shape={'type': 'cone', 'radius': 1.0, 'height': 2.0})
convexhull_shape = world.rigid_body(shape={
    'type': 'convexhull',
    'vertices': [
        [-1.0, -1.0, -1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ],
})
multisphere_shape = world.rigid_body(shape={
    'type': 'multisphere',
    'spheres': [
        [-1.0, -1.0, -1.0, 0.4],
        [1.0, 0.0, 0.0, 0.5],
        [0.0, 1.0, 0.0, 0.6],
        [0.0, 0.0, 1.0, 0.7],
    ],
})
minkowski_shape = world.rigid_body(shape={
    'type': 'minkowski',
    'a': {
        'transform': {
            'position': [0.0, 0.0, 0.0],
            'rotation': [0.0, 0.0, 0.0, 1.0],
        },
        'shape': {'type': 'box', 'size': [1.0, 2.0, 3.0]},
    },
    'b': {
        'transform': {
            'position': [0.0, 0.0, 0.0],
            'rotation': [0.0, 0.0, 0.0, 1.0],
        },
        'shape': {'type': 'sphere', 'radius': 0.2},
    },
})
compound_shape = world.rigid_body(shape={
    'type': 'compound',
    'children': [
        {
            'transform': {
                'position': [0.0, 0.0, 0.0],
                'rotation': [0.0, 0.0, 0.0, 1.0],
            },
            'shape': {'type': 'box', 'size': [3.0, 1.0, 1.0]},
        },
        {
            'transform': {
                'position': [0.0, 0.0, 0.0],
                'rotation': [0.0, 0.0, 0.0, 1.0],
            },
            'shape': {'type': 'box', 'size': [1.0, 3.0, 1.0]},
        },
        {
            'transform': {
                'position': [0.0, 0.0, 0.0],
                'rotation': [0.0, 0.0, 0.0, 1.0],
            },
            'shape': {'type': 'box', 'size': [1.0, 1.0, 3.0]},
        },
    ],
})

print(world.shapes())
