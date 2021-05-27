#version 460


// INPUTS

layout(location = 0) in vec4 instance_side_mip[];
layout(location = 1) in float instance_mip[];
layout(location = 2) in vec2 instance_param_space_offset[];
layout(location = 3) in float instance_param_space_size[];

layout(location = 4) in uint instance_index[];


// OUTPUTS

layout(vertices = 4) out;

layout(location = 0) out float mip[];
layout(location = 1) out vec2 param_space_offset[];
layout(location = 2) out float param_space_size[];

layout(location = 3) out uint index[];


void main()
{
    if (gl_InvocationID == 0)
    {
        gl_TessLevelInner[0] = instance_mip[0];
        gl_TessLevelInner[1] = instance_mip[0];

        gl_TessLevelOuter[0] = instance_side_mip[0].w;
        gl_TessLevelOuter[1] = instance_side_mip[0].x;
        gl_TessLevelOuter[2] = instance_side_mip[0].y;
        gl_TessLevelOuter[3] = instance_side_mip[0].z;
    }

    gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;

    mip[gl_InvocationID] = instance_mip[gl_InvocationID];
    param_space_offset[gl_InvocationID] = instance_param_space_offset[gl_InvocationID];
    param_space_size[gl_InvocationID] = instance_param_space_size[gl_InvocationID];

    index[gl_InvocationID] = instance_index[gl_InvocationID];
}
