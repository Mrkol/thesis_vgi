#pragma once

#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.h>
#include "ResourceManager.hpp"


static constexpr uint32_t OBJECT_DESCRIPTOR_SET_BINDING = 1;

class SceneObjectTypeFactory;
class SceneObjectType;

class SceneObjectBase
{
public:
    virtual void on_type_object_available(SceneObjectType& type) = 0;

    virtual void tick() {};

    virtual void record_pre_commands(vk::CommandBuffer cb) {};

    virtual void record_commands(vk::CommandBuffer cb) = 0;

    [[nodiscard]] virtual const SceneObjectTypeFactory& get_scene_object_type_factory() const = 0;

    /**
     * Object will die on the next tick
     */
    void kill() { alive = false; }
    [[nodiscard]] bool is_alive() const { return alive; }

    virtual ~SceneObjectBase() = default;

private:
    bool alive{true};
};

class SceneObjectType
{
public:
    explicit SceneObjectType(IResourceManager* irm) : resource_manager(irm) {};

    virtual void tick() {};

    virtual void record_commands(vk::CommandBuffer cb) {};

    virtual void reload_shaders() {};

    struct PipelineCreateInfo
    {
        vk::Extent2D viewport_extents;
        vk::DescriptorSetLayout scene_descriptor_set_layout;
        vk::RenderPass render_pass;
    };

    virtual vk::UniquePipeline create_pipeline(PipelineCreateInfo info) = 0;
    vk::PipelineLayout get_pipeline_layout() const { return pipeline_layout.get(); }
    IResourceManager* get_resource_manager() const { return resource_manager; }
    vk::DescriptorSetLayout get_type_descriptor_set_layout() const { return type_descriptor_set_layout.get(); }
    vk::DescriptorSetLayout get_instance_descriptor_set_layout() const { return instance_descriptor_set_layout.get(); }

    virtual ~SceneObjectType() = default;

protected:
    IResourceManager* resource_manager;
    vk::UniquePipelineLayout pipeline_layout;
    vk::UniqueDescriptorSetLayout type_descriptor_set_layout;
    vk::UniqueDescriptorSetLayout instance_descriptor_set_layout;
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
