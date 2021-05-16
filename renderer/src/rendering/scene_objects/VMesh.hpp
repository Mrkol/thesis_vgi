#pragma once

#include <filesystem>

#include "Tangible.hpp"
#include "../vgi/HierarchicalAtlas.hpp"
#include "../data_primitives/VirtualTextureSet.hpp"
#include "../data_primitives/RingBuffer.hpp"
#include "../data_primitives/DescriptorSetRing.hpp"


class VMesh : public TangibleSceneObject
{
public:
    explicit VMesh(const std::filesystem::path& folder);

    void record_pre_commands(vk::CommandBuffer cb) override;

    void on_type_object_available(SceneObjectType& type) override;

    void tick() override;

    void record_commands(vk::CommandBuffer cb) override;

    [[nodiscard]] const SceneObjectTypeFactory& get_scene_object_type_factory() const override;

private:
    HierarchicalAtlas atlas;
    HierarchyCut current_cut;
    class VMeshSceneObjectType* our_type{nullptr};

    UniqueVmaBuffer vbo;
    RingBuffer ubo;
    VirtualTextureSet vgis;

    DescriptorSetRing descriptors;
};

class VMeshSceneObjectType : public SceneObjectType
{
public:
    explicit VMeshSceneObjectType(IResourceManager* irm);

    vk::UniquePipeline create_pipeline(PipelineCreateInfo info) override;
    void reload_shaders() override;

private:
    void load_shaders();

private:
    std::vector<std::byte> vertex_shader;
    std::vector<std::byte> fragment_shader;
    std::vector<std::byte> tess_ctrl_shader;
    std::vector<std::byte> tess_eval_shader;
    std::vector<std::byte> geom_shader;
};

MAKE_SCENE_OBJECT_TYPE_FACTORY(VMeshSceneObjectType);
