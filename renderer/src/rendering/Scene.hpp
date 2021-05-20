#pragma once

#include <memory>
#include <deque>
#include <unordered_map>
#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.h>

#include "ResourceManager.hpp"
#include "Camera.hpp"
#include "data_primitives/RingBuffer.hpp"
#include "data_primitives/DescriptorSetRing.hpp"


class SceneObjectBase;
class SceneObjectType;

struct PipelineCreationInfo
{
    vk::RenderPass render_pass;
    vk::Extent2D extent;
};

enum class ViewMode : uint32_t
{
    Normal,
    Wireframe,
    SIZE
};

// vec4 as vulkan seems to hate vec3
struct DirectionalLight
{
    Eigen::Vector4f direction;
    Eigen::Vector4f diffuse;
    Eigen::Vector4f specular;
};

class Scene
{
public:
    Scene(IResourceManager* irm, PipelineCreationInfo info);

    void recreate_pipelines(PipelineCreationInfo info);

    void reload_shaders();

    void add_object(std::unique_ptr<SceneObjectBase> object);

    void tick(float delta_seconds);
    void record_pre_commands(vk::CommandBuffer cb);
    void record_commands(vk::CommandBuffer cb);

    Camera* debug_get_camera() { return &camera; }

public:
    ViewMode view_mode = ViewMode::Normal;

private:
    IResourceManager* resource_manager;

    PipelineCreationInfo pipeline_creation_info;

    vk::UniqueDescriptorSetLayout global_descriptor_set_layout;

    RingBuffer global_uniform_buffer;
    DescriptorSetRing global_uniforms;

    struct PerTypeInfo
    {
        std::unique_ptr<SceneObjectType> type;
        vk::UniquePipeline pipeline;
        std::vector<SceneObjectBase*> instances;
    };

    std::unordered_map<std::string_view, PerTypeInfo> object_types;
    std::vector<std::unique_ptr<SceneObjectBase>> scene_objects;

    Camera camera;

    DirectionalLight sun{};
};
