#version 460

layout(location = 0) in vec3 color;
layout(location = 1) flat in uint mip;

layout(location = 0) out vec4 frag_color;


void main()
{
    frag_color = vec4(floor(color * mip) / mip, 0);
}
