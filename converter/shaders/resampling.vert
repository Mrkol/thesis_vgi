#version 450
#extension GL_ARB_separate_shader_objects : enable


layout(location = 0) in vec2 inMapping;
layout(location = 1) in vec3 inPosition;

layout(location = 0) out vec3 position;

layout(binding = 0) uniform UniformBufferObject {
    float scale;
} ubo;

void main()
{
    gl_Position = vec4((inMapping * 2 - 1) * ubo.scale, 0.0, 1.0);
    position = vec3(inPosition);
}
