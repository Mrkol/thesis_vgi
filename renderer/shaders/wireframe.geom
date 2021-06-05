#version 460


// INPUTS

layout (triangles) in;

layout(location = 0) in vec3 position[3];
layout(location = 1) in vec3 normal[3];
layout(location = 2) in vec2 uv[3];


// OUTPUTS

layout(line_strip, max_vertices = 3) out;

layout(location = 0) out vec3 out_position;
layout(location = 1) out vec3 out_normal;
layout(location = 2) out vec2 out_uv;

void main()
{
    for(int i = 0; i < 4; i++)
    {
        gl_Position = gl_in[i % 3].gl_Position;

        out_position = position[i % 3];
        out_normal = normal[i % 3];
        out_uv = uv[i % 3];

        EmitVertex();
    }

    EndPrimitive();
}
