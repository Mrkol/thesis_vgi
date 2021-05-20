#version 460

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 uv;

layout(location = 0) out vec4 frag_color;

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

layout(set = 1, binding = 3) uniform sampler2DArray maps;

mat3 calculate_tspace()
{
    vec3 Q1 = dFdx(position);
    vec3 Q2 = dFdy(position);
    vec2 st1 = dFdx(uv);
    vec2 st2 = dFdy(uv);

    return mat3(
        normalize(Q1*st2.t - Q2*st1.t),
        normalize(-Q1*st2.s + Q2*st1.s),
        normalize(normal));
}


void main()
{
    vec3 albedo = texture(maps, vec3(uv, 0)).rgb;
    float roughness = texture(maps, vec3(uv, 1)).r;

    vec3 actual_normal = normalize(normal);
    vec3 sun_direction = normalize(mat3(global_ubo.view) * -global_ubo.sun.direction.xyz);
    vec3 eye_direction = normalize(-position);

    float L = max(dot(actual_normal, sun_direction), 0);

    float S = max(dot(reflect(-sun_direction, actual_normal), eye_direction), 0);

    vec3 color =
        // Ambient
        albedo * vec3(0.05, 0.05, 0.05)
        // Diffues
        + albedo * global_ubo.sun.diffuse.rgb * L
        // specular
        + 0.01 / roughness * global_ubo.sun.specular.rgb * pow(S, 3);

    frag_color = vec4(color, 1);
}
