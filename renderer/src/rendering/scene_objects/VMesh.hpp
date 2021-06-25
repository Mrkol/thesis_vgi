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
    explicit VMesh(std::filesystem::path  folder);

    void on_type_object_available(SceneObjectType& type) override;

    void tick(float delta_seconds, TickInfo tick_info) override;

    void record_commands(vk::CommandBuffer cb) override;

    [[nodiscard]] const SceneObjectTypeFactory& get_scene_object_type_factory() const override;

private:
    std::filesystem::path folder_;

    HierarchyCut current_cut_;
    class VMeshSceneObjectType* our_type_{nullptr};

    std::shared_ptr<class Model> model_;

    RingBuffer vbo_;
    RingBuffer ubo_;

    DescriptorSetRing descriptors_;
};

struct Model
{
    HierarchicalAtlas atlas;
    VirtualTextureSet vgis;
    TexturePtr texture;
    vk::UniqueSampler sampler;
};

class VMeshSceneObjectType : public SceneObjectType
{
public:
    explicit VMeshSceneObjectType(IResourceManager* irm);

    void post_tick(float delta_seconds, TickInfo tick_info) override;
    void record_pre_commands(vk::CommandBuffer cb) override;

    [[nodiscard]] std::shared_ptr<Model> get_model(const std::filesystem::path& path);

    vk::UniquePipeline create_pipeline(PipelineCreateInfo info) override;

private:
    ShaderPtr vertex_shader_;
    ShaderPtr tess_ctrl_shader_;
    ShaderPtr tess_eval_shader_;
    ShaderPtr geom_shader_;
    ShaderPtr fragment_shader_;

    std::unordered_map<std::string, std::weak_ptr<Model>> models_;
};

MAKE_SCENE_OBJECT_TYPE_FACTORY(VMeshSceneObjectType);
