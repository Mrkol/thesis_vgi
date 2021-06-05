#pragma once

#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.h>

#include "ResourceManager.hpp"
#include "ViewMode.hpp"



class SceneObjectTypeFactory;
class SceneObjectType;


struct TickInfo
{
    const Eigen::Matrix4f& view;
    const Eigen::Matrix4f& proj;
    vk::Extent2D resolution;
};

class SceneObjectBase
{
    friend class Scene;

public:
    virtual void on_type_object_available(SceneObjectType& type) = 0;

    virtual void tick(float delta_seconds, TickInfo tick_info) {};

    virtual void record_pre_commands(vk::CommandBuffer cb) {};

    virtual void record_commands(vk::CommandBuffer cb) = 0;

    [[nodiscard]] virtual const SceneObjectTypeFactory& get_scene_object_type_factory() const = 0;

    /**
     * Object will die on the next tick
     */
    void kill() { alive_ = false; }
    [[nodiscard]] bool is_alive() const { return alive_; }

    virtual ~SceneObjectBase() = default;

private:
    bool alive_{true};
    bool enabled_{true};
};

class SceneObjectType
{
public:
    explicit SceneObjectType(IResourceManager* irm) : resource_manager_(irm) {};

    virtual void tick(float delta_seconds, TickInfo tick_info) {};

    virtual void record_commands(vk::CommandBuffer cb) {};

    struct PipelineCreateInfo
    {
        vk::Extent2D viewport_extents;
        vk::DescriptorSetLayout scene_descriptor_set_layout;
        vk::RenderPass render_pass;
        ViewMode mode;
    };

    virtual vk::UniquePipeline create_pipeline(PipelineCreateInfo info) = 0;
    vk::PipelineLayout get_pipeline_layout() const { return pipeline_layout_.get(); }
    IResourceManager* get_resource_manager() const { return resource_manager_; }
    vk::DescriptorSetLayout get_instance_descriptor_set_layout() const { return instance_descriptor_set_layout_.get(); }

    virtual ~SceneObjectType() = default;

protected:
    IResourceManager* resource_manager_;
    vk::UniquePipelineLayout pipeline_layout_;
    vk::UniqueDescriptorSetLayout instance_descriptor_set_layout_;
};

class SceneObjectTypeFactory
{
public:
    [[nodiscard]] virtual std::unique_ptr<SceneObjectType> create(IResourceManager* irm) const = 0;

    [[nodiscard]] virtual std::string_view get_name() const = 0;

    virtual ~SceneObjectTypeFactory() = default;
};

/**
 * Factory definitions always look the same, hence the convenience macro
 */
#define MAKE_SCENE_OBJECT_TYPE_FACTORY(type) \
class type##Factory : public SceneObjectTypeFactory \
{ \
public: \
    std::unique_ptr<SceneObjectType> create(IResourceManager* irm) const override \
        { return std::make_unique<type>(irm); }\
    constexpr static const char* NAME = #type;\
    std::string_view get_name() const override { return {NAME}; }\
} 
