#version 450
#extension GL_ARB_separate_shader_objects : enable


layout(location = 0) in vec3 position;

layout(location = 0) out vec4 outColor;


void main()
{
    outColor = vec4(.5, .5, .5, 0);
}
