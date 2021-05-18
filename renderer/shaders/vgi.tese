#version 460


// INPUTS

layout(quads, equal_spacing, cw) in;

layout(location = 0) in float instance_mip[];
layout(location = 1) in vec2 instance_param_space_offset[];
layout(location = 2) in float instance_param_space_size[];
layout(location = 3) in uint instance_index[];


// OUTPUTS

layout(location = 0) out vec3 color;
layout(location = 1) flat out uint out_mip;


// UNIFORMS

layout(set = 0, binding = 0) uniform GlobalUBO {
    mat4 view;
    mat4 proj;
} global_ubo;

layout(set = 1, binding = 0) uniform ObjectUBO {
    mat4 model;
    uint cache_side_size;
    uint min_mip;
    uint mip_level_count;
} object_ubo;

layout(set = 1, binding = 1) uniform sampler2DArray geometry_image;

layout(std430, set = 1, binding = 2) readonly buffer ObjectSBO {
    // AFAIK we can't be more specific about how data is layed out here :(
    uint indirection_tables[];
} object_sbo;

uint calc_indirection_tables_size(uint count)
{
    return ((1u << (2*count)) - 1)/3;
}

uint read_indirection(uint idx, uint size, uint x, uint y)
{
    return object_sbo.indirection_tables[
        idx * calc_indirection_tables_size(object_ubo.mip_level_count)
        + calc_indirection_tables_size(size)
        + y * (1u << size)
        + x
    ];
}

const uint PAGE_NONE = ~uint(0);

struct Pixel
{
    vec3 position;
    vec2 uv;
    vec3 normal;
};

/**
 * page -- page index (row-major)
 * in_page_uv -- normalized coords of requested data
 */
Pixel get_from_cache(uint page, vec2 in_page_uv)
{
    vec2 page_in_cache_uv = vec2(
        float(page % object_ubo.cache_side_size),
        float(page / object_ubo.cache_side_size));

    float psz = float((1 << object_ubo.min_mip) + 1);

    vec2 uv = (page_in_cache_uv + (in_page_uv * (psz - 1) + 0.5) / psz) / float(object_ubo.cache_side_size);

    vec4 a = texture(geometry_image, vec3(uv, 0));
    vec4 b = texture(geometry_image, vec3(uv, 1));
    return Pixel(a.xyz, vec2(a.w, b.x), b.yzw);
}

Pixel read_virtual_texture(uint idx, uint mip, vec2 uv, uvec2 page_offset)
{
    uint page_count = (1u << mip) / (1u << object_ubo.min_mip);

    // Pick the leftest toppest page ignoring page overlap
    uvec2 page_uv = uvec2(uv * page_count);
    // Out of bounds on bottom edge and right edge are handled via page_offset
    // Tessellated grid edge picking the wrong page is handled via page_offset as well
    // offset is only applied at page boundary
    page_uv -= page_offset * uvec2(1 - (uv * page_count - page_uv));

    uint page = PAGE_NONE;

    while (mip >= object_ubo.min_mip)
    {
        page = read_indirection(idx,
            mip - object_ubo.min_mip,
            page_uv.x, page_uv.y
        );

        if (page != PAGE_NONE)
        {
            break;
        }

        page_uv /= 2;
        page_count /= 2;
        --mip;
    }

    vec2 in_page_uv = uv * page_count - page_uv;

    return get_from_cache(page, in_page_uv);
}

void main()
{
    color = vec3(gl_TessCoord.xy, 1);
    out_mip = uint(instance_mip[0]);


    Pixel p = read_virtual_texture(
        instance_index[0],
        uint(clamp(log2(instance_mip[0] / instance_param_space_size[0]),
            object_ubo.min_mip, object_ubo.min_mip + object_ubo.mip_level_count - 1)),
        instance_param_space_offset[0] + gl_TessCoord.xy * instance_param_space_size[0],
        uvec2(gl_TessCoord.xy)
    );

    gl_Position = global_ubo.proj * global_ubo.view * object_ubo.model * vec4(p.position, 1.0);
}
