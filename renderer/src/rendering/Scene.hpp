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
#include "ViewMode.hpp"


class SceneObjectBase;
class SceneObjectType;

struct PipelineCreationInfo
{
    vk::RenderPass render_pass;
    vk::Extent2D extent;
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

    void add_object(std::unique_ptr<SceneObjectBase> object);

    void tick(float delta_seconds);
    void record_pre_commands(vk::CommandBuffer cb);
    void record_commands(vk::CommandBuffer cb);

    Camera* debug_get_camera() { return &camera_; }

private:
    IResourceManager* resource_manager_;

    PipelineCreationInfo pipeline_creation_info_;

    vk::UniqueDescriptorSetLayout global_descriptor_set_layout_;

    RingBuffer global_uniform_buffer_;
    DescriptorSetRing global_uniforms_;

    struct PerTypeInfo
    {
        std::unique_ptr<SceneObjectType> type;
        vk::UniquePipeline pipeline;
        std::vector<SceneObjectBase*> instances;
    };

    std::unordered_map<std::string_view, PerTypeInfo> object_types_;
    std::vector<std::unique_ptr<SceneObjectBase>> scene_objects_;

    Camera camera_;

    DirectionalLight sun_{};
    ViewMode view_mode_;
};
