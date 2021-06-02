#version 460


// INPUTS

layout(location = 0) in vec4 position;
layout(location = 1) in vec4 normal;


// OUTPUTS

layout(location = 0) out vec3 out_position;
layout(location = 1) out vec3 out_normal;
layout(location = 2) out vec2 out_uv;


// UNIFORMS

struct DirectionalLight
{
    vec4 direction;
    vec4 diffuse;
    vec4 specular;
};

layout(set = 0, binding = 0) uniform GlobalUBO {
    mat4 view;
    mat4 proj;
    DirectionalLight sun;
} global_ubo;

layout(set = 1, binding = 0) uniform ObjectUBO {
    mat4 model;
    mat4 normal;
} object_ubo;


void main() {
    vec4 screenspace_position = global_ubo.view * object_ubo.model * vec4(position.xyz, 1.0);

    out_position = screenspace_position.xyz / screenspace_position.w;
    out_normal = mat3(object_ubo.normal) * normal.xyz;
    out_uv = vec2(position.w, normal.w);

    gl_Position = global_ubo.proj * screenspace_position;
}
