#pragma once

#include <Eigen/Geometry>

#include "Tangible.hpp"
#include "../data_primitives/RingBuffer.hpp"
#include "../data_primitives/DescriptorSetRing.hpp"


class GridSceneObjectType;

class GridSceneObject : public TangibleSceneObject
{
public:
    explicit GridSceneObject(std::size_t size);

    void on_type_object_available(SceneObjectType& type) override;

    void tick(float delta_seconds, TickInfo tick_info) override;

    void record_commands(vk::CommandBuffer cb) override;
    [[nodiscard]] const SceneObjectTypeFactory& get_scene_object_type_factory() const override;

    ~GridSceneObject() override;

private:
    std::size_t size_;
    GridSceneObjectType* our_type_{nullptr};

    vk::Buffer vbo_;
    RingBuffer uniform_buffer_;
    DescriptorSetRing uniforms_;
};

class GridSceneObjectType : public SceneObjectType
{
public:
    explicit GridSceneObjectType(IResourceManager* irm);

    vk::UniquePipeline create_pipeline(PipelineCreateInfo info) override;

    vk::Buffer acquire_vbo(std::size_t grid_size);
    void release_vbo(std::size_t grid_size);

private:
    ShaderPtr vertex_shader_;
    ShaderPtr fragment_shader_;


    struct PerSizeData
    {
        std::size_t users{0};
        UniqueVmaBuffer buffer;
    };
    std::unordered_map<std::size_t, PerSizeData> buffers_for_sizes_;
};

MAKE_SCENE_OBJECT_TYPE_FACTORY(GridSceneObjectType);
