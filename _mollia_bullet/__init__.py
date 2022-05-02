def parse_transform(obj):
    x, y, z = map(float, obj['position'])
    rx, ry, rz, rw = map(float, obj['rotation'])
    return (x, y, z, rx, ry, rz, rw)


def parse_shape(shape):
    if shape is None or shape['type'] == 'empty':
        return (0,)
    if shape['type'] == 'box':
        width, length, height = shape['size']
        return (1, float(width), float(length), float(height))
    if shape['type'] == 'sphere':
        return (2, float(shape['radius']))
    if shape['type'] == 'capsule':
        return (3, float(shape['radius']), float(shape['height']))
    if shape['type'] == 'cylinder':
        return (4, float(shape['radius']), float(shape['height']))
    if shape['type'] == 'cone':
        return (5, float(shape['radius']), float(shape['height']))
    if shape['type'] == 'convexhull':
        return (6, [(float(x), float(y), float(z)) for x, y, z in shape['vertices']])
    if shape['type'] == 'multisphere':
        return (7, [(float(x), float(y), float(z), float(r)) for x, y, z, r in shape['spheres']])
    if shape['type'] == 'minkowski':
        a = parse_transform(shape['a']['transform']), parse_shape(shape['a']['shape'])
        b = parse_transform(shape['b']['transform']), parse_shape(shape['b']['shape'])
        return (8, *a, *b)
    if shape['type'] == 'compound':
        return (9, [(parse_transform(c['transform']), parse_shape(c['shape'])) for c in shape['children']])
    raise ValueError('invalid shape')
