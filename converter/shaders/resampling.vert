#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(location = 0) in dvec2 inMapping;
layout(location = 2) in dvec3 inPosition;

layout(location = 0) out dvec3 position;


void main()
{
    gl_Position = dvec4(inMapping, 0.0, 0.0);
    position = inPosition;
}
