#version 460


layout(location = 0) in vec2 in_mapping;
layout(location = 1) in vec3 in_position;
layout(location = 2) in vec3 in_normal;
layout(location = 3) in vec2 in_uv;

layout(location = 0) out vec3 position;
layout(location = 1) out vec3 normal;
layout(location = 2) out vec2 uv;

layout(binding = 0) uniform UniformBufferObject {
    float scale;
} ubo;

void main()
{
    gl_Position = vec4((in_mapping * 2 - 1) * ubo.scale, 0.0, 1.0);
    position = in_position;
    normal = in_normal;
    uv = in_uv;
}
