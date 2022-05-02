import numpy as np
from scipy.spatial import ConvexHull


def make_hull(points):
    hull = ConvexHull(points)
    vertices = hull.points[hull.simplices]
    normals = np.cross(vertices[:, 0] - vertices[:, 1], vertices[:, 0] - vertices[:, 2])
    sign = np.sign(np.sum(vertices[:, 0] * normals, axis=1))
    vertices[np.where(sign < 0.0)] = vertices[np.where(sign < 0.0), ::-1]
    normals = (normals.T / np.sqrt(np.sum(normals * normals, axis=1)) * sign).T
    mesh = np.zeros((hull.simplices.shape[0] * 3, 9), 'f4')
    mesh[:, 0:3] = vertices.reshape(-1, 3)
    mesh[:, 3:6] = np.repeat(normals, 3, axis=0)
    return mesh


def gen_box(width, length, height):
    points = np.array([[i, j, k] for i in (-0.5, 0.5) for j in (-0.5, 0.5) for k in (-0.5, 0.5)])
    points *= (width, length, height)
    return points


def gen_cylinder(radius, height, res=32):
    x = np.cos(np.linspace(0.0, np.pi * 4.0, res * 2, endpoint=False))
    y = np.sin(np.linspace(0.0, np.pi * 4.0, res * 2, endpoint=False))
    z = np.repeat([-0.5, 0.5], res)
    return np.array([x, y, z]).T * (radius, radius, height)


def gen_cone(radius, height, res=32):
    x = np.cos(np.linspace(0.0, np.pi * 2.0, res + 1))
    y = np.sin(np.linspace(0.0, np.pi * 2.0, res + 1))
    z = np.full(res + 1, -0.5)
    points = np.array([x, y, z]).T
    points[-1] = [0.0, 0.0, 0.5]
    return points * (radius, radius, height)


def gen_capsule(radius, height, res=16):
    sphere = gen_uvsphere(radius, res)
    offset = [0.0, 0.0, height * 0.5]
    return np.concatenate([sphere - offset, sphere + offset])


def gen_sphere(radius, res=100):
    phi = np.pi * (3.0 - np.sqrt(5.0))
    y = 1.0 - (np.arange(res) / (res - 1.0)) * 2.0
    x = np.cos(phi * np.arange(res)) * np.sqrt(1.0 - y * y)
    z = np.sin(phi * np.arange(res)) * np.sqrt(1.0 - y * y)
    return np.array([x, y, z]).T * radius


def gen_uvsphere(radius, res=16):
    h = np.repeat(np.linspace(0.0, np.pi * 2.0, res * 2, endpoint=False), res - 1)
    v = np.tile(np.linspace(0.0, np.pi, res + 1)[1:-1], res * 2)
    ends = [[0.0, 0.0, -1.0], [0.0, 0.0, 1.0]]
    points = np.concatenate([np.array([np.cos(h) * np.sin(v), np.sin(h) * np.sin(v), np.cos(v)]).T, ends])
    return points * radius


def gen_multisphere(points, res=100):
    return np.concatenate([gen_sphere(r, res) + (x, y, z) for x, y, z, r in points])


def gen_minkowski(a, b):
    return np.tile(a.flatten(), b.shape[0]).reshape(-1, 3) + np.tile(b, a.shape[0]).reshape(-1, 3)


def transform_shape(frame, mesh):
    # TODO: implement
    return mesh


def build_shape_points(shape):
    if shape['type'] == 'box':
        return gen_box(*shape['size'])
    if shape['type'] == 'sphere':
        return gen_uvsphere(shape['radius'])
    if shape['type'] == 'capsule':
        return gen_capsule(shape['radius'], shape['height'])
    if shape['type'] == 'cylinder':
        return gen_cylinder(shape['radius'], shape['height'])
    if shape['type'] == 'cone':
        return gen_cone(shape['radius'], shape['height'])
    if shape['type'] == 'convexhull':
        return np.array(shape['vertices'])
    if shape['type'] == 'multisphere':
        return gen_multisphere(shape['spheres'])
    if shape['type'] == 'minkowski':
        return gen_minkowski(build_shape_points(shape['a']['shape']), build_shape_points(shape['b']['shape']))


def build_shape(shape, instance):
    if shape['type'] == 'compound':
        mesh = np.concatenate([transform_shape(np.array(child['transform']), build_shape(child['shape'], instance)) for child in shape['children']])
    elif shape['type'] == 'empty':
        return np.ndarray((0, 9))
    else:
        mesh = make_hull(build_shape_points(shape))
    mesh[:, 6:9] = instance
    return mesh


def build_shapes(shapes):
    return np.concatenate([build_shape(shape, (0.0625 + 1.0 / 8.0 * (i % 8), 0.0625 + 1.0 / 8.0 * (i // 8), i)) for i, shape in enumerate(shapes)]).astype('f4')
