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
    std::size_t size;
    GridSceneObjectType* our_type{nullptr};

    vk::Buffer vbo;
    RingBuffer uniform_buffer;
    DescriptorSetRing uniforms;
};

class GridSceneObjectType : public SceneObjectType
{
public:
    explicit GridSceneObjectType(IResourceManager* irm);

    vk::UniquePipeline create_pipeline(PipelineCreateInfo info) override;

    vk::Buffer acquire_vbo(std::size_t grid_size);
    void release_vbo(std::size_t grid_size);

private:
    std::vector<std::byte> vertex_shader;
    std::vector<std::byte> fragment_shader;


    struct PerSizeData
    {
        std::size_t users{0};
        UniqueVmaBuffer buffer;
    };
    std::unordered_map<std::size_t, PerSizeData> buffers_for_sizes;
};

MAKE_SCENE_OBJECT_TYPE_FACTORY(GridSceneObjectType);
