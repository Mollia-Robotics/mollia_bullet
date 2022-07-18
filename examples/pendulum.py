import colorsys
import mollia_bullet
from mollia_bullet.visualization import ColorMeshPreview
from itertools import cycle

world = mollia_bullet.world()
preview = ColorMeshPreview(world)

world.gravity = (0.0, 0.0, -10.0)

box_count = 2
ox, oy, oz = [0,0,0]
w, l = 0.1, 0.5
static_box = world.box(0, (w/2,w/2,l/2) , (0,0,0), group = 1, mask = 1)
boxes = [world.box(1, (w/2, w/2, l/2), (ox, oy, oz + (- i - 0.5) * l - 0.5 * w), group = 1, mask = 1) for i in range(box_count)]

for i in range(box_count):    
    boxes[i].color = colorsys.hls_to_rgb(i / 10, 0.5, 0.5)

# stiffness = (50000.0, 0.5, False) #stiffness, damping, on/off
# friction = (5.0, 0.001, 0.15) #linear, rolling, spinning
# boxes[0].stiffness = stiffness
# boxes[1].friction = friction

axis = (1.0, 0.0, 0.0)
h0 = world.hinge(static_box, boxes[0], pivot=(0.0, 0.0, -0.5 * w), axis=axis, ref=static_box, collision=False)
hinges = [world.hinge(box0, box1, pivot=(0.0, 0.0, -0.5*l), axis=axis, ref = box0, collision=False)
                    for box0, box1 in zip(boxes, boxes[1:])]


controller = world.motor_control([h0] + hinges)
vs = cycle([0.3]*1000 + [-0.3]*1000)

while True:

    # boxes[-1].apply_force([1,1,0])
    controller.input_array[:] = [
                [2, next(vs)] for _ in range(box_count)
        ]
    world.simulate()
    preview(up=(0.0, 0.0, -1.0))

preview.wait_to_close()

