#pragma once

#include <filesystem>

#include "Tangible.hpp"
#include "../../UniqueStbImage.hpp"


class StaticMesh : public TangibleSceneObject
{

public:
    explicit StaticMesh(const std::filesystem::path& folder);

    void on_type_object_available(SceneObjectType& type) override;

    void tick(float delta_seconds, TickInfo tick_info) override;

    void record_commands(vk::CommandBuffer cb) override;

    [[nodiscard]] const SceneObjectTypeFactory& get_scene_object_type_factory() const override;

private:
    static constexpr std::size_t TEXTURE_MAP_COUNT = 2;
    std::array<UniqueStbImage, TEXTURE_MAP_COUNT> texture_maps;

    class VMeshSceneObjectType* our_type{nullptr};

    UniqueVmaBuffer vbo;
    RingBuffer ubo;

    DescriptorSetRing descriptors;
};

class StaticMeshSceneObjectType : public SceneObjectType
{
public:
    explicit StaticMeshSceneObjectType(class IResourceManager* irm);

    vk::UniquePipeline create_pipeline(PipelineCreateInfo info) override;
    void reload_shaders() override;

private:
    void load_shaders();

private:
    std::vector<std::byte> vertex_shader;
    std::vector<std::byte> fragment_shader;
};

MAKE_SCENE_OBJECT_TYPE_FACTORY(StaticMeshSceneObjectType);