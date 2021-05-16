#version 460


// INPUTS

layout (triangles) in;

layout(location = 0) in vec3 color[3];
layout(location = 1) in uint mip[3];


// OUTPUTS

layout(line_strip, max_vertices = 3) out;

layout(location = 0) out vec3 out_color;
layout(location = 1) flat out uint out_mip;


void main()
{
    for(int i = 0; i < 4; i++)
    {
        gl_Position = gl_in[i % 3].gl_Position;

        out_color = color[i % 3];
        out_mip = mip[i % 3];

        EmitVertex();
    }

    EndPrimitive();
}
