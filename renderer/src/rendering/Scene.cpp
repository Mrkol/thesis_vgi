#include "Scene.hpp"

#include <functional>
#include <imgui.h>

#include "SceneObjectBase.hpp"
#include "scene_objects/Grid.hpp"
#include "scene_objects/VMesh.hpp"
#include "scene_objects/StaticMesh.hpp"
#include "../EigenHelpers.hpp"
#include "data_primitives/RingBuffer.hpp"


struct GlobalUBO
{
    Eigen::Matrix4f view;
    Eigen::Matrix4f proj;
    DirectionalLight sun;
};


Scene::Scene(IResourceManager* irm, PipelineCreationInfo info)
    : resource_manager_{irm}
    , pipeline_creation_info_{info}
    , global_uniform_buffer_{irm->create_ubo(sizeof(GlobalUBO))}
{
    vk::DescriptorSetLayoutBinding binding{
        /* binding */ 0,
        vk::DescriptorType::eUniformBuffer,
        /* descriptor count */ 1,
        vk::ShaderStageFlagBits::eVertex
            | vk::ShaderStageFlagBits::eTessellationEvaluation
            | vk::ShaderStageFlagBits::eFragment,
        nullptr
    };

    global_descriptor_set_layout_ = irm->get_device().createDescriptorSetLayoutUnique(vk::DescriptorSetLayoutCreateInfo{
        {}, 1, &binding
    });

    global_uniforms_ = irm->create_descriptor_set_ring(global_descriptor_set_layout_.get());
    global_uniforms_.write_ubo(global_uniform_buffer_, 0);

    {
        auto grid = std::make_unique<GridSceneObject>(16);
        grid->scale.setConstant(16);
        add_object(std::move(grid));
    }

//    {
//        auto vmesh = std::make_unique<VMesh>("../../models/sphere");
//        add_object(std::move(vmesh));
//    }

    {
        auto vmesh = std::make_unique<VMesh>("../../models/rock_assembly_rough");
        vmesh->scale.setConstant(0.02f);
        vmesh->rotation = Eigen::AngleAxisf(-EIGEN_PI/2, Eigen::Vector3f::UnitX());
        add_object(std::move(vmesh));
    }

    {
        auto mesh = std::make_unique<StaticMesh>("../../models/rock_assembly_rough/sjzbj_LOD0");
        mesh->enabled_ = false;
        mesh->scale.setConstant(0.02f);
        mesh->rotation = Eigen::AngleAxisf(-EIGEN_PI/2, Eigen::Vector3f::UnitX());
        add_object(std::move(mesh));
    }

    {
        auto mesh = std::make_unique<StaticMesh>("../../models/rock_assembly_rough/sjzbj_High");
        mesh->enabled_ = false;
        mesh->scale.setConstant(0.02f);
        mesh->rotation = Eigen::AngleAxisf(-EIGEN_PI/2, Eigen::Vector3f::UnitX());
        add_object(std::move(mesh));
    }

//    {
//        auto mesh = std::make_unique<StaticMesh>("../../models/rock_cliffs_old/uchwaffda_LOD0");
//        mesh->scale.setConstant(0.02f);
//        mesh->position << 10, 0, -5;
//        mesh->rotation = Eigen::AngleAxisf(EIGEN_PI, Eigen::Vector3f::UnitZ())
//            * Eigen::AngleAxisf(-EIGEN_PI/2, Eigen::Vector3f::UnitX());
//        add_object(std::move(mesh));
//    }
//    {
//        auto vmesh = std::make_unique<VMesh>("../../models/rock_cliffs_old");
//        vmesh->scale.setConstant(0.02f);
//        vmesh->position << -10, 0, -5;
//        vmesh->rotation = Eigen::AngleAxisf(EIGEN_PI, Eigen::Vector3f::UnitZ())
//            * Eigen::AngleAxisf(-EIGEN_PI/2, Eigen::Vector3f::UnitX());
//        add_object(std::move(vmesh));
//    }
//
//    {
//        auto vmesh = std::make_unique<VMesh>("../../models/nature_snow_old");
//        vmesh->scale.setConstant(0.01f);
//        vmesh->position << -10, 0, 0;
//        vmesh->rotation = Eigen::AngleAxisf(-EIGEN_PI/2, Eigen::Vector3f::UnitX());
//        add_object(std::move(vmesh));
//    }
//
//    {
//        auto vmesh = std::make_unique<VMesh>("../../models/rock_cliffs_old");
//        vmesh->scale.setConstant(0.02f);
//        vmesh->position << 10, 0, 0;
//        vmesh->rotation = Eigen::AngleAxisf(static_cast<float>(-EIGEN_PI/2), Eigen::Vector3f::UnitX());
//        add_object(std::move(vmesh));
//    }


    sun_.direction << -1, 0, 1, 0;
    sun_.diffuse << 1, 1, 1, 0;
    sun_.specular << 1, 1, 1, 0;
}

void Scene::recreate_pipelines(PipelineCreationInfo info)
{
    resource_manager_->get_device().waitIdle();

    pipeline_creation_info_ = info;

    for (auto&[_, type_data] : object_types_)
    {
        type_data.pipeline = type_data.type->create_pipeline(
            {info.extent, global_descriptor_set_layout_.get(), info.render_pass, view_mode_});
    }
}

void Scene::tick(float delta_seconds)
{
    ImGui::Begin("Scene");

    {
        static std::array view_modes { "Normal", "Wireframe" };

        if (ImGui::BeginCombo("##combo", view_modes[static_cast<std::size_t>(view_mode_)])) // The second parameter is the label previewed before opening the combo.
        {
            for (std::size_t i = 0; i < view_modes.size(); ++i)
            {
                bool is_selected = static_cast<std::size_t>(view_mode_) == i;
                if (ImGui::Selectable(view_modes[i], is_selected))
                {
                    view_mode_ = static_cast<ViewMode>(i);
                    recreate_pipelines(pipeline_creation_info_);
                }
                if (is_selected)
                {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
    }


    GlobalUBO ubo{
        camera_.view(),
        perspective(static_cast<float>(EIGEN_PI/2),
            static_cast<float>(pipeline_creation_info_.extent.width)
                /static_cast<float>(pipeline_creation_info_.extent.height),
            .001f, 100.f),
        sun_
    };
    global_uniform_buffer_.write_next(std::span{reinterpret_cast<std::byte*>(&ubo), sizeof(ubo)});

    int object_id{0};
    for (auto& kv : object_types_)
    {
        auto&[type, _, instances] = kv.second;
        type->tick(delta_seconds, {ubo.view, ubo.proj, pipeline_creation_info_.extent});
        for (auto instance : instances)
        {
            ImGui::PushID(object_id++);
            ImGui::Text("%s", std::string(instance->get_scene_object_type_factory().get_name()).c_str());
            ImGui::SameLine(ImGui::GetWindowWidth() - 30);
            ImGui::Checkbox("", &instance->enabled_);

            if (instance->enabled_)
            {
                instance->tick(delta_seconds, {ubo.view, ubo.proj, pipeline_creation_info_.extent});
            }

            ImGui::PopID();
        }

        std::erase_if(instances, std::not_fn(std::mem_fn(&SceneObjectBase::is_alive)));
    }

    std::erase_if(scene_objects_, [](const auto& uptr) { return !uptr->is_alive(); });

    std::erase_if(object_types_, [](const auto& kv) { return kv.second.instances.empty(); });

    ImGui::End();
}

void Scene::record_pre_commands(vk::CommandBuffer cb)
{
    for (auto& object : scene_objects_)
    {
        if (object->enabled_)
        {
            object->record_pre_commands(cb);
        }
    }
}

void Scene::record_commands(vk::CommandBuffer cb)
{
    auto global_uniforms_descriptor_set = global_uniforms_.read_next();


    for (auto& kv : object_types_)
    {
        auto&[type, pipeline, instances] = kv.second;

        cb.bindPipeline(vk::PipelineBindPoint::eGraphics, pipeline.get());
        cb.bindDescriptorSets(vk::PipelineBindPoint::eGraphics, type->get_pipeline_layout(), 0,
            {global_uniforms_descriptor_set}, {});


        for (auto& instance : instances)
        {
            if (instance->enabled_)
            {
                instance->record_commands(cb);
            }
        }
    }
}

void Scene::add_object(std::unique_ptr<SceneObjectBase> object)
{
    auto& factory = object->get_scene_object_type_factory();
    auto type_name = factory.get_name();

    auto it = object_types_.find(type_name);

    if (it == object_types_.end())
    {
        PerTypeInfo info{
            factory.create(resource_manager_)
        };

        info.pipeline = info.type->create_pipeline(
            {pipeline_creation_info_.extent, global_descriptor_set_layout_.get(), pipeline_creation_info_.render_pass});

        it = object_types_.emplace(type_name, std::move(info)).first;
    }

    it->second.instances.push_back(object.get());

    scene_objects_.emplace_back(std::move(object))
        ->on_type_object_available(*it->second.type);
}
