#version 460


// INPUTS

layout(location = 0) in vec4 instance_side_mip;
layout(location = 1) in float instance_mip;
layout(location = 2) in vec2 instance_param_space_offset;
layout(location = 3) in float instance_param_space_size;
layout(location = 4) in uint patch_index;


// OUTPUTS

layout(location = 0) out vec4 out_instance_side_mip;
layout(location = 1) out float out_instance_mip;
layout(location = 2) out vec2 out_instance_param_space_offset;
layout(location = 3) out float out_instance_param_space_size;

layout(location = 4) out uint out_index;

vec2 positions[4] = vec2[](
    vec2(0.0, 0.0),
    vec2(0.0, 1.0),
    vec2(1.0, 1.0),
    vec2(1.0, 0.0)
);

void main() {
    gl_Position = vec4(positions[gl_VertexIndex], 0.0, 1.0);

    out_instance_side_mip = instance_side_mip;
    out_instance_mip = instance_mip;
    out_instance_param_space_offset = instance_param_space_offset;
    out_instance_param_space_size = instance_param_space_size;

    out_index = patch_index;
}
