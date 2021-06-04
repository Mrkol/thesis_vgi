#pragma once

#include <filesystem>

#include "Tangible.hpp"
#include "../../UniqueStbImage.hpp"


// UV is packed into position.w and normal.w
struct Vertex
{
    Eigen::Vector4f position;
    Eigen::Vector4f normal;

    bool operator==(const Vertex& other) const = default;
};

class StaticMesh : public TangibleSceneObject
{
public:
    explicit StaticMesh(const std::filesystem::path& model);

    void on_type_object_available(SceneObjectType& type) override;

    void tick(float delta_seconds, TickInfo tick_info) override;

    void record_commands(vk::CommandBuffer cb) override;

    [[nodiscard]] const SceneObjectTypeFactory& get_scene_object_type_factory() const override;

private:
    static constexpr std::size_t TEXTURE_MAP_COUNT = 2;
    std::array<UniqueStbImage, TEXTURE_MAP_COUNT> texture_maps_;

    std::vector<uint32_t> indices_;
    std::vector<Vertex> attributes_;
    std::vector<uint32_t> lod_offsets_;

    std::size_t current_lod_{0};

    class StaticMeshSceneObjectType* our_type_{nullptr};

    UniqueVmaBuffer vbo_;
    UniqueVmaBuffer ibo_;
    RingBuffer ubo_;
    UniqueVmaImage textures_;
    vk::UniqueImageView textures_view_;
    vk::UniqueSampler sampler_;

    DescriptorSetRing descriptors_;
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
    std::vector<std::byte> vertex_shader_;
    std::vector<std::byte> fragment_shader_;
};

MAKE_SCENE_OBJECT_TYPE_FACTORY(StaticMeshSceneObjectType);
