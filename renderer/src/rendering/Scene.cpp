#include "Scene.hpp"

#include "SceneObjectBase.hpp"
#include "scene_objects/Grid.hpp"
#include "scene_objects/VMesh.hpp"
#include "../EigenHelpers.hpp"
#include "data_primitives/RingBuffer.hpp"


struct GlobalUBO
{
    Eigen::Matrix4f view;
    Eigen::Matrix4f proj;
};

static_assert(sizeof(GlobalUBO) == sizeof(float) * (4 * 4) * 2);

Scene::Scene(IResourceManager* irm, PipelineCreationInfo info)
    : resource_manager{irm}
    , pipeline_creation_info{info}
    , global_uniform_buffer{irm->create_ubo(sizeof(GlobalUBO))}
{
    vk::DescriptorSetLayoutBinding binding{
        /* binding */ 0,
        vk::DescriptorType::eUniformBuffer,
        /* descriptor count */ 1,
        vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eTessellationEvaluation,
        nullptr
    };

    global_descriptor_set_layout = irm->get_device().createDescriptorSetLayoutUnique(vk::DescriptorSetLayoutCreateInfo{
        {}, 1, &binding
    });

    global_uniforms = irm->create_descriptor_set_ring(global_descriptor_set_layout.get());
    global_uniforms.write_ubo(global_uniform_buffer, 0);

    add_object(std::make_unique<GridSceneObject>(10));
    auto vmesh = std::make_unique<VMesh>("../converter/resampled");
    vmesh->scale.setConstant(0.01f);
    vmesh->rotation = Eigen::AngleAxisf(-EIGEN_PI/2, Eigen::Vector3f::UnitX());
    add_object(std::move(vmesh));
}

void Scene::recreate_pipelines(PipelineCreationInfo info)
{
    pipeline_creation_info = info;

    for (auto&[_, type_data] : object_types)
    {
        type_data.pipeline = type_data.type->create_pipeline(
            {info.extent, global_descriptor_set_layout.get(), info.render_pass});
    }
}

void Scene::reload_shaders()
{
    for (auto&[name, info] : object_types)
    {
        info.type->reload_shaders();
    }
}

void Scene::tick(float delta_seconds)
{
    GlobalUBO ubo{
        camera.view(),
        perspective(static_cast<float>(EIGEN_PI/2),
            static_cast<float>(pipeline_creation_info.extent.width)
                /static_cast<float>(pipeline_creation_info.extent.height),
            .01f, 10.f)
    };
    global_uniform_buffer.write_next(std::span{reinterpret_cast<std::byte*>(&ubo), sizeof(ubo)});


    for (auto& kv : object_types)
    {
        auto&[type, _, instances] = kv.second;
        type->tick(delta_seconds, {ubo.view, ubo.proj, pipeline_creation_info.extent});
        for (auto instance : instances)
        {
            instance->tick(delta_seconds, {ubo.view, ubo.proj, pipeline_creation_info.extent});
        }

        std::erase_if(instances, std::not1(std::mem_fn(&SceneObjectBase::is_alive)));
    }

    std::erase_if(scene_objects, [](const auto& uptr) { return !uptr->is_alive(); });

    std::erase_if(object_types, [](const auto& kv) { return kv.second.instances.empty(); });
}

void Scene::record_pre_commands(vk::CommandBuffer cb)
{
    for (auto& object : scene_objects)
    {
        object->record_pre_commands(cb);
    }
}

void Scene::record_commands(vk::CommandBuffer cb)
{
    for (auto& kv : object_types)
    {
        auto&[type, pipeline, instances] = kv.second;

        cb.bindPipeline(vk::PipelineBindPoint::eGraphics, pipeline.get());
        cb.bindDescriptorSets(vk::PipelineBindPoint::eGraphics, type->get_pipeline_layout(), 0,
            {global_uniforms.read_next()}, {});


        for (auto& instance : instances)
        {
            instance->record_commands(cb);
        }
    }
}

void Scene::add_object(std::unique_ptr<SceneObjectBase> object)
{
    auto& factory = object->get_scene_object_type_factory();
    auto type_name = factory.get_name();

    auto it = object_types.find(type_name);

    if (it == object_types.end())
    {
        PerTypeInfo info{
            factory.create(resource_manager)
        };

        info.pipeline = info.type->create_pipeline(
            {pipeline_creation_info.extent, global_descriptor_set_layout.get(), pipeline_creation_info.render_pass});

        it = object_types.emplace(type_name, std::move(info)).first;
    }

    it->second.instances.push_back(object.get());

    scene_objects.emplace_back(std::move(object))
        ->on_type_object_available(*it->second.type);
}
