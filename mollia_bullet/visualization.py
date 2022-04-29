import struct
import time
from typing import Optional, Tuple

import mollia_window
import numpy as np
import zengl


class ColorMeshPreview:
    def __init__(self, world, size: Optional[Tuple[int, int]] = None):
        if size is None:
            size = (640, 480)

        self.world = world
        self.wnd = mollia_window.preview_window('ColorMeshPreview', size)
        ctx = zengl.context(zengl.loader(headless=True))

        self.size = size
        samples = min(ctx.limits['max_samples'], 4)
        self.image = ctx.image(size, 'rgba8unorm-srgb', samples=samples)
        self.depth = ctx.image(size, 'depth24plus', samples=samples)
        self.resolve = ctx.image(size, 'rgba8unorm-srgb', texture=False)
        self.image.clear_value = (0.95, 0.95, 0.95, 1.0)
        self.aspect = size[0] / size[1]

        self.vertex_buffer = ctx.buffer(size=4 * 1024 * 1024)
        self.vertex_stride = zengl.calcsize('3f 3f 3f')
        self.uniform_buffer = ctx.buffer(size=80)

        self.pipeline = ctx.pipeline(
            vertex_shader='''
                #version 330

                layout (std140) uniform Common {
                    mat4 mvp;
                    vec4 eye;
                };

                layout (location = 0) in vec3 in_vert;
                layout (location = 1) in vec3 in_norm;
                layout (location = 2) in vec3 in_color;

                out vec3 v_vert;
                out vec3 v_norm;
                out vec3 v_color;

                void main() {
                    v_vert = in_vert;
                    v_norm = in_norm;
                    v_color = in_color;
                    gl_Position = mvp * vec4(v_vert, 1.0);
                }
            ''',
            fragment_shader='''
                #version 330

                layout (std140) uniform Common {
                    mat4 mvp;
                    vec4 eye;
                };

                in vec3 v_vert;
                in vec3 v_norm;
                in vec3 v_color;

                layout (location = 0) out vec4 out_color;

                void main() {
                    float lum = dot(normalize(eye.xyz - v_vert), normalize(v_norm)) * 0.5 + 0.5;
                    out_color = vec4(v_color * lum, 1.0);
                }
            ''',
            layout=[
                {
                    'name': 'Common',
                    'binding': 0,
                },
            ],
            resources=[
                {
                    'type': 'uniform_buffer',
                    'binding': 0,
                    'buffer': self.uniform_buffer,
                },
            ],
            framebuffer=[self.image, self.depth],
            topology='triangles',
            cull_face='back',
            vertex_buffers=zengl.bind(self.vertex_buffer, '3f 3f 3f', 0, 1, 2),
        )

    def __call__(self, eye=(4.0, 3.0, 2.0), target=(0.0, 0.0, 0.0), up=(0.0, 0.0, 1.0), fov=45.0):
        if not self.wnd.visible:
            return

        mesh = self.world.main_group.color_mesh()
        camera = zengl.camera(eye, target, up, aspect=self.aspect, fov=fov)
        self.uniform_buffer.write(struct.pack('=64s3f', camera, *eye))
        self.vertex_buffer.write(mesh)
        self.pipeline.vertex_count = len(mesh) // self.vertex_stride
        self.image.clear()
        self.depth.clear()
        self.pipeline.render()
        self.image.blit(self.resolve)
        self.wnd.update(np.ndarray((self.size[1], self.size[0], 4), 'u1', self.resolve.read())[::-1, :, 0:3])

    def wait_to_close(self):
        while self.wnd.visible:
            time.sleep(0.1)
