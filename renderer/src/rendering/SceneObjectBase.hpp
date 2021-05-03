#pragma once

#include <vulkan/vulkan.hpp>


static constexpr uint32_t OBJECT_DESCRIPTOR_SET_BINDING = 1;

class SceneObjectTypeFactory;

class SceneObjectBase
{
public:
    virtual void update_descriptor_sets(vk::DescriptorSet) = 0;

    virtual void render(vk::CommandBuffer) = 0;

    virtual const SceneObjectTypeFactory& get_scene_object_type_factory() const = 0;

    virtual ~SceneObjectBase() = default;

};

struct SceneObjectTypeCreateInfo
{
    vk::Device device;
    std::size_t swap_chain_size;
    vk::Extent2D viewport_extents;
    vk::DescriptorSetLayout global_descriptor_set_layout;
    vk::DescriptorPool& descriptor_pool;
    vk::RenderPass render_pass;
};

class SceneObjectType
{
public:
    virtual void begin_rendering(vk::CommandBuffer cb);

    const vk::PipelineLayout& get_pipeline_layout() const { return pipeline_layout.get(); };

    virtual ~SceneObjectType() = default;

protected:
    vk::UniquePipelineLayout pipeline_layout;
    vk::UniquePipeline pipeline;
    vk::UniqueDescriptorSetLayout instance_descriptor_set_layout;
};

class SceneObjectTypeFactory
{
public:
    virtual std::unique_ptr<SceneObjectType> create(SceneObjectTypeCreateInfo info) const = 0;

    virtual std::string_view get_name() const = 0;

    virtual ~SceneObjectTypeFactory() = default;
};

/**
 * Factory definitions always look the same, hence the convenience macro
 */
#define MAKE_SCENE_OBJECT_TYPE_FACTORY(type) \
class type##Factory : public SceneObjectTypeFactory \
{ \
public: \
    std::unique_ptr<SceneObjectType> create(SceneObjectTypeCreateInfo info) const override \
        { return std::make_unique<type>(std::move(info)); }\
    constexpr static const char* NAME = #type;\
    std::string_view get_name() const override { return {NAME}; }\
} 
