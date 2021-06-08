#pragma once

#include <filesystem>

#include "Tangible.hpp"
#include "../vgi/HierarchicalAtlas.hpp"
#include "../data_primitives/VirtualTextureSet.hpp"
#include "../data_primitives/RingBuffer.hpp"
#include "../data_primitives/DescriptorSetRing.hpp"
#include "../../UniqueStbImage.hpp"


class VMesh : public TangibleSceneObject
{
public:
    explicit VMesh(const std::filesystem::path& folder);

    void record_pre_commands(vk::CommandBuffer cb) override;

    void on_type_object_available(SceneObjectType& type) override;

    void tick(float delta_seconds, TickInfo tick_info) override;

    void record_commands(vk::CommandBuffer cb) override;

    [[nodiscard]] const SceneObjectTypeFactory& get_scene_object_type_factory() const override;

private:
    static constexpr std::size_t TEXTURE_MAP_COUNT = 2;
    std::array<std::filesystem::path, TEXTURE_MAP_COUNT> texture_maps_;

    // TODO: Move the atlas to the type object
    HierarchicalAtlas atlas_;
    HierarchyCut current_cut_;
    class VMeshSceneObjectType* our_type_{nullptr};

    RingBuffer vbo_;
    RingBuffer ubo_;
    VirtualTextureSet vgis_;
    TexturePtr texture_;
    vk::UniqueSampler sampler_;

    DescriptorSetRing descriptors_;
};

class VMeshSceneObjectType : public SceneObjectType
{
public:
    explicit VMeshSceneObjectType(IResourceManager* irm);

    vk::UniquePipeline create_pipeline(PipelineCreateInfo info) override;

private:
    ShaderPtr vertex_shader_;
    ShaderPtr tess_ctrl_shader_;
    ShaderPtr tess_eval_shader_;
    ShaderPtr geom_shader_;
    ShaderPtr fragment_shader_;
};

MAKE_SCENE_OBJECT_TYPE_FACTORY(VMeshSceneObjectType);
