import struct

import mollia_window
import numpy as np
import zengl

from _mollia_bullet.shapes import build_shapes


def kernel(s):
    x = np.arange(-s, s + 1)
    y = np.exp(-x * x / (s * s / 4))
    y /= y.sum()
    v = ', '.join(f'{t:.8f}' for t in y)
    return f'const int N = {s * 2 + 1};\nfloat coeff[N] = float[]({v});'


class Blur:
    def __init__(self, ctx: zengl.Context, image):
        self.temp = ctx.image(image.size, 'rgba8unorm')
        self.output = ctx.image(image.size, 'rgba8unorm')

        ctx.includes['blur_kernel'] = kernel(11)

        self.blur_x = ctx.pipeline(
            vertex_shader='''
                #version 330

                vec2 positions[3] = vec2[](
                    vec2(-1.0, -1.0),
                    vec2(3.0, -1.0),
                    vec2(-1.0, 3.0)
                );

                void main() {
                    gl_Position = vec4(positions[gl_VertexID], 0.0, 1.0);
                }
            ''',
            fragment_shader='''
                #version 330

                uniform sampler2D Texture;

                layout (location = 0) out vec4 out_color;

                #include "blur_kernel"

                void main() {
                    vec3 color = vec3(0.0, 0.0, 0.0);
                    for (int i = 0; i < N; ++i) {
                        color += texelFetch(Texture, ivec2(gl_FragCoord.xy) + ivec2(i - N / 2, 0), 0).rgb * coeff[i];
                    }
                    out_color = vec4(color, 1.0);
                }
            ''',
            layout=[
                {
                    'name': 'Texture',
                    'binding': 0,
                },
            ],
            resources=[
                {
                    'type': 'sampler',
                    'binding': 0,
                    'image': image,
                },
            ],
            framebuffer=[self.temp],
            topology='triangles',
            vertex_count=3,
        )

        self.blur_y = ctx.pipeline(
            vertex_shader='''
                #version 330

                vec2 positions[3] = vec2[](
                    vec2(-1.0, -1.0),
                    vec2(3.0, -1.0),
                    vec2(-1.0, 3.0)
                );

                void main() {
                    gl_Position = vec4(positions[gl_VertexID], 0.0, 1.0);
                }
            ''',
            fragment_shader='''
                #version 330

                uniform sampler2D Texture;

                layout (location = 0) out vec4 out_color;

                #include "blur_kernel"

                void main() {
                    vec3 color = vec3(0.0, 0.0, 0.0);
                    for (int i = 0; i < N; ++i) {
                        color += texelFetch(Texture, ivec2(gl_FragCoord.xy) + ivec2(0, i - N / 2), 0).rgb * coeff[i];
                    }
                    out_color = vec4(color, 1.0);
                }
            ''',
            layout=[
                {
                    'name': 'Texture',
                    'binding': 0,
                },
            ],
            resources=[
                {
                    'type': 'sampler',
                    'binding': 0,
                    'image': self.temp,
                },
            ],
            framebuffer=[self.output],
            topology='triangles',
            vertex_count=3,
        )

    def render(self):
        self.blur_x.render()
        self.blur_y.render()


class Plane:
    def __init__(self, ctx: zengl.Context, ubo, image, shadow, framebuffer):
        self.pipeline = ctx.pipeline(
            vertex_shader='''
                #version 330

                #include "main_uniform_buffer"

                vec3 vertices[4] = vec3[](
                    vec3(-1000.0, -1000.0, 0.0),
                    vec3(-1000.0, 1000.0, 0.0),
                    vec3(1000.0, -1000.0, 0.0),
                    vec3(1000.0, 1000.0, 0.0)
                );

                out vec3 v_vertex;

                void main() {
                    v_vertex = vertices[gl_VertexID];
                    gl_Position = mvp * vec4(vertices[gl_VertexID], 1.0);
                }
            ''',
            fragment_shader='''
                #version 330

                #include "main_uniform_buffer"

                uniform sampler2D Texture;
                uniform sampler2D Shadow;

                in vec3 v_vertex;

                layout (location = 0) out vec4 out_color;

                void main() {
                    float gray = 0.3 + float((int(v_vertex.x + 1000.0) + int(v_vertex.y + 1000.0)) % 2) * 0.05;
                    gray = mix(gray, 0.3, smoothstep(10.0, 30.0, distance(eye.xy, v_vertex.xy)));
                    vec3 color = texelFetch(Texture, ivec2(gl_FragCoord.xy), 0).rgb * gray;
                    float luminance = dot(color, vec3(0.2125, 0.7154, 0.0721));
                    out_color = vec4(color, 1.0);
                    // out_color = vec4(vec3(luminance), 1.0);

                    vec4 tmp = lmvp * vec4(v_vertex, 1.0);
                    vec3 shadow_coord = tmp.xyz / tmp.w * 0.5 + 0.5;
                    if (shadow_coord.z > 0.0 && shadow_coord.x > 0.0 && shadow_coord.x < 1.0 &&  shadow_coord.y > 0.0 && shadow_coord.y < 1.0) {
                        float shadow = texture(Shadow, shadow_coord.xy).r;
                        float dist = distance(light.xyz, v_vertex);
                        if (shadow < dist - 1e-3) {
                            out_color.rgb *= 0.5;
                        }
                    }
                }
            ''',
            layout=[
                {
                    'name': 'MainUniformBuffer',
                    'binding': 0,
                },
                {
                    'name': 'Texture',
                    'binding': 0,
                },
                {
                    'name': 'Shadow',
                    'binding': 1,
                },
            ],
            resources=[
                {
                    'type': 'uniform_buffer',
                    'binding': 0,
                    'buffer': ubo,
                },
                {
                    'type': 'sampler',
                    'binding': 0,
                    'image': image,
                },
                {
                    'type': 'sampler',
                    'binding': 1,
                    'image': shadow,
                },
            ],
            framebuffer=framebuffer,
            topology='triangle_strip',
            vertex_count=4,
        )

    def render(self):
        self.pipeline.render()


class BonesData:
    def __init__(self, ctx, obj):
        self.obj = obj
        self.bones_buffer = ctx.buffer(size=4096)
        shapes = obj.shapes()
        self.vertex_buffer = ctx.buffer(build_shapes(shapes))
        self.vertex_count = self.vertex_buffer.size // zengl.calcsize('3f 3f 2f 1f')

    def render(self):
        self.bones_buffer.write(self.obj.frame())


class Bones:
    def __init__(self, ctx: zengl.Context, ubo, texture, shadow, framebuffer, data: BonesData, reflect):
        self.data = data
        ctx.includes['reflect'] = f'const bool reflect = {str(reflect).lower()};'
        self.pipeline = ctx.pipeline(
            vertex_shader='''
                #version 330

                #include "main_uniform_buffer"
                #include "reflect"

                layout (std140) uniform UniformBufferBones {
                    vec4 Bones[256];
                };

                layout (location = 0) in vec3 in_vertex;
                layout (location = 1) in vec3 in_normal;
                layout (location = 2) in vec2 in_texcoord;
                layout (location = 3) in float in_bone;

                out vec3 v_vertex;
                out vec3 v_normal;
                out vec2 v_texcoord;

                vec3 qtransform(vec4 q, vec3 v) {
                    return v + 2.0 * cross(cross(v, q.xyz) - q.w * v, q.xyz);
                }

                void main() {
                    vec4 position = Bones[int(in_bone) * 2 + 0];
                    vec4 rotation = Bones[int(in_bone) * 2 + 1];
                    v_vertex = position.xyz + qtransform(rotation, in_vertex);
                    v_normal = qtransform(rotation, in_normal);
                    v_texcoord = in_texcoord;
                    if (reflect) {
                        v_vertex.z = -v_vertex.z;
                        v_normal.z = -v_normal.z;
                    }
                    gl_Position = mvp * vec4(v_vertex, 1.0);
                }
            ''',
            fragment_shader='''
                #version 330

                #include "main_uniform_buffer"
                #include "reflect"

                uniform sampler2D Texture;
                uniform sampler2D Shadow;

                in vec3 v_vertex;
                in vec3 v_normal;
                in vec2 v_texcoord;

                layout (location = 0) out vec4 out_color;

                void main() {
                    if (reflect && v_vertex.z > 0.0) {
                        discard;
                    }

                    vec3 v_color = texture(Texture, v_texcoord).rgb;

                    float ambient = 0.05;
                    float facing = 0.1;
                    float shininess = 32.0;
                    float light_power = 10.0;

                    vec3 light_dir = light.xyz - v_vertex;
                    float light_distance = length(light_dir);
                    light_distance = light_distance * light_distance;
                    light_dir = normalize(light_dir);

                    vec3 view_dir = normalize(eye.xyz - v_vertex);
                    float facing_view_dot = max(dot(view_dir, v_normal), 0.0);
                    vec3 color = v_color * ambient + v_color * facing_view_dot * facing;

                    vec4 tmp = lmvp * vec4(v_vertex, 1.0);
                    vec3 shadow_coord = tmp.xyz / tmp.w * 0.5 + 0.5;
                    if (shadow_coord.z > 0.0 && shadow_coord.x > 0.0 && shadow_coord.x < 1.0 &&  shadow_coord.y > 0.0 && shadow_coord.y < 1.0) {
                        float shadow = texture(Shadow, shadow_coord.xy).r;
                        float dist = distance(light.xyz, v_vertex);
                        if (shadow + 1e-3 > dist) {
                            float lambertian = max(dot(light_dir, v_normal), 0.0);
                            float specular = 0.0;

                            if (lambertian > 0.0) {
                                vec3 half_dir = normalize(light_dir + view_dir);
                                float spec_angle = max(dot(half_dir, v_normal), 0.0);
                                specular = pow(spec_angle, shininess);
                            }

                            color += v_color * lambertian * light_power / light_distance + specular * light_power / light_distance;
                        }
                    }

                    out_color = vec4(color, 1.0);
                }
            ''',
            layout=[
                {
                    'name': 'MainUniformBuffer',
                    'binding': 0,
                },
                {
                    'name': 'UniformBufferBones',
                    'binding': 1,
                },
                {
                    'name': 'Texture',
                    'binding': 0,
                },
                {
                    'name': 'Shadow',
                    'binding': 1,
                },
            ],
            resources=[
                {
                    'type': 'uniform_buffer',
                    'binding': 0,
                    'buffer': ubo,
                },
                {
                    'type': 'uniform_buffer',
                    'binding': 1,
                    'buffer': self.data.bones_buffer,
                },
                {
                    'type': 'sampler',
                    'binding': 0,
                    'image': texture,
                },
                {
                    'type': 'sampler',
                    'binding': 1,
                    'image': shadow,
                },
            ],
            framebuffer=framebuffer,
            topology='triangles',
            cull_face='front' if reflect else 'back',
            vertex_buffers=zengl.bind(self.data.vertex_buffer, '3f 3f 2f 1f', 0, 1, 2, 3),
        )

    def render(self):
        self.pipeline.vertex_count = self.data.vertex_count
        self.pipeline.render()


class BonesShadow:
    def __init__(self, ctx: zengl.Context, ubo, texture, framebuffer, data: BonesData, reflect):
        self.data = data
        ctx.includes['reflect'] = f'const bool reflect = {str(reflect).lower()};'
        self.pipeline = ctx.pipeline(
            vertex_shader='''
                #version 330

                #include "main_uniform_buffer"
                #include "reflect"

                layout (std140) uniform UniformBufferBones {
                    vec4 Bones[256];
                };

                layout (location = 0) in vec3 in_vertex;
                layout (location = 1) in float in_bone;

                out vec3 v_vertex;

                vec3 qtransform(vec4 q, vec3 v) {
                    return v + 2.0 * cross(cross(v, q.xyz) - q.w * v, q.xyz);
                }

                void main() {
                    vec4 position = Bones[int(in_bone) * 2 + 0];
                    vec4 rotation = Bones[int(in_bone) * 2 + 1];
                    v_vertex = position.xyz + qtransform(rotation, in_vertex);
                    if (reflect) {
                        v_vertex.z = -v_vertex.z;
                    }
                    gl_Position = lmvp * vec4(v_vertex, 1.0);
                }
            ''',
            fragment_shader='''
                #version 330

                #include "main_uniform_buffer"
                #include "reflect"

                in vec3 v_vertex;

                layout (location = 0) out float out_distance;

                void main() {
                    if (reflect && v_vertex.z > 0.0) {
                        discard;
                    }
                    out_distance = distance(v_vertex, light.xyz);
                }
            ''',
            layout=[
                {
                    'name': 'MainUniformBuffer',
                    'binding': 0,
                },
                {
                    'name': 'UniformBufferBones',
                    'binding': 1,
                },
            ],
            resources=[
                {
                    'type': 'uniform_buffer',
                    'binding': 0,
                    'buffer': ubo,
                },
                {
                    'type': 'uniform_buffer',
                    'binding': 1,
                    'buffer': self.data.bones_buffer,
                },
            ],
            framebuffer=framebuffer,
            topology='triangles',
            cull_face='front' if reflect else 'back',
            vertex_buffers=zengl.bind(self.data.vertex_buffer, '3f 3f 2f 1f', 0, -1, -1, 1),
        )

    def render(self):
        self.pipeline.vertex_count = self.data.vertex_count
        self.pipeline.render()


class Renderer:
    def __init__(self, world):
        self.wnd = mollia_window.wnd
        self.ctx = zengl.context()
        samples = min(self.ctx.limits['max_samples'], 4)

        self.reflection_image = self.ctx.image(self.wnd.size, 'rgba8unorm-srgb')
        self.reflection_depth = self.ctx.image(self.wnd.size, 'depth24plus')

        self.shadow_image = self.ctx.image((1024, 1024), 'r32float')
        self.shadow_depth = self.ctx.image((1024, 1024), 'depth24plus')

        self.image = self.ctx.image(self.wnd.size, 'rgba8unorm-srgb', samples=samples)
        self.depth = self.ctx.image(self.wnd.size, 'depth24plus', samples=samples)
        self.texture = self.ctx.image((8, 8), 'rgba8snorm', b'\xff\xff\xff\xff' * 64)

        self.uniform_buffer = self.ctx.buffer(size=256)
        self.ubo_struct = struct.Struct('=64s64s3f4x3f4x')
        self.ubo_data = bytearray(256)
        self.ctx.includes['main_uniform_buffer'] = '''
            layout (std140) uniform MainUniformBuffer {
                mat4 mvp;
                mat4 lmvp;
                vec4 eye;
                vec4 light;
            };
        '''
        self.blur = Blur(self.ctx, self.reflection_image)
        self.bones_data = BonesData(self.ctx, world)
        self.pipelines = [
            self.bones_data,
            BonesShadow(self.ctx, self.uniform_buffer, self.texture, [self.shadow_image, self.shadow_depth], self.bones_data, False),
            Bones(self.ctx, self.uniform_buffer, self.texture, self.shadow_image, [self.reflection_image, self.reflection_depth], self.bones_data, True),
            self.blur,
            Plane(self.ctx, self.uniform_buffer, self.blur.output, self.shadow_image, [self.image, self.depth]),
            Bones(self.ctx, self.uniform_buffer, self.texture, self.shadow_image, [self.image, self.depth], self.bones_data, False),
        ]

    def set_camera(self, eye, target, up, fov=45.0):
        light = (-2.0, 2.0, 3.0)
        camera = zengl.camera(eye, target, up, fov=fov, aspect=self.wnd.ratio)
        light_camera = zengl.camera(light, (2.0, 0.0, 0.0), up, fov=70.0, aspect=1.0, near=0.1, far=20.0)
        self.ubo_struct.pack_into(self.ubo_data, 0, camera, light_camera, *eye, *light)

    def render(self):
        self.uniform_buffer.write(self.ubo_data)

        self.shadow_image.clear_value = 1e6
        self.shadow_image.clear()
        self.shadow_depth.clear()

        self.reflection_image.clear_value = (1.0, 1.0, 1.0, 1.0)
        self.reflection_image.clear()
        self.reflection_depth.clear()

        self.image.clear_value = (1.0, 1.0, 1.0, 1.0)
        self.image.clear()
        self.depth.clear()

        for pipeline in self.pipelines:
            pipeline.render()

        self.image.blit()
