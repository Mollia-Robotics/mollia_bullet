import colorsys

import mollia_bullet
from mollia_bullet.visualization import ColorMeshPreview

world = mollia_bullet.world()
world.gravity = (0.0, 0.0, -10.0)

ground = world.box(0.0, (10.0, 10.0, 10.0), (0.0, 0.0, -10.0), group=1, mask=1)

for i in range(10):
    box = world.box(1.0, (0.25, 0.25, 0.25), (0.0, 0.0, 2.0 + i), group=1, mask=1)
    box.color = color=colorsys.hls_to_rgb(i / 10, 0.5, 0.5)

preview = ColorMeshPreview(world)

for i in range(500):
    world.simulate()
    preview()

preview.wait_to_close()
