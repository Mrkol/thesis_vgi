#version 450
#extension GL_ARB_separate_shader_objects : enable


layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 uv;

layout(location = 0) out vec4 out1;
layout(location = 1) out vec4 out2;


void main()
{
    out1 = vec4(position, uv.x);
    out2 = vec4(uv.y, normal);
}
