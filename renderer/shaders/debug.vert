#version 460


layout(location = 0) in vec3 inPosition;

layout(location = 0) out vec3 color;

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
} object_ubo;


void main()
{
    gl_Position = global_ubo.proj * global_ubo.view * object_ubo.model * vec4(inPosition, 1);
    color = ((gl_VertexIndex / 2) & 1) == 0 ? vec3(0.5, 0, 0) : vec3(0, 0.5, 0);
}
