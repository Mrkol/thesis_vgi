#include "VMesh.hpp"

#include <stb_image.h>
#include <imgui.h>
#include <map>

#include <VkHelpers.hpp>
#include <Utility.hpp>
#include <utility>



static constexpr std::size_t CACHE_SIZE = 256;

struct PerInstanceData
{
    Eigen::Vector4f side_mips;
    float mip;
    Eigen::Vector2f param_space_offset;
    float param_space_size;
    uint32_t patch_index;
};

struct UBO
{
    Eigen::Matrix4f model;
    Eigen::Matrix4f normal;
    uint32_t cache_side_pages;
    uint32_t min_mip;
    uint32_t mip_level_count;
};

VMesh::VMesh(std::filesystem::path  folder)
    : folder_{std::move(folder)}
{
}

const SceneObjectTypeFactory& VMesh::get_scene_object_type_factory() const
{
    static VMeshSceneObjectTypeFactory factory;
    return factory;
}

void VMesh::on_type_object_available(SceneObjectType& type)
{
    our_type_ = dynamic_cast<VMeshSceneObjectType*>(&type);

    auto irm = type.get_resource_manager();

    model_ = our_type_->get_model(folder_);

    vbo_ = irm->create_dynamic_vbo(sizeof(PerInstanceData)
        * (1ull << model_->atlas.get_hierarchy_depth()) * model_->atlas.get_patch_count());

    ubo_ = irm->create_ubo(sizeof(UBO));
    descriptors_ = irm->create_descriptor_set_ring(type.get_instance_descriptor_set_layout());
    descriptors_.write_ubo(ubo_, 0);


    vk::DescriptorImageInfo descriptor_image_info{
        model_->vgis.sampler(), model_->vgis.view(), vk::ImageLayout::eShaderReadOnlyOptimal
    };

    descriptors_.write_all(std::vector(descriptors_.size(), vk::WriteDescriptorSet{
        {}, 2, 0,
        1, vk::DescriptorType::eCombinedImageSampler,
        &descriptor_image_info, nullptr, nullptr
    }));

    descriptors_.write_sbo(model_->vgis.get_indirection_table_sbo(), 3);

    vk::DescriptorImageInfo info{
        model_->sampler.get(), model_->texture->get(), vk::ImageLayout::eShaderReadOnlyOptimal
    };
    descriptors_.write_all(std::vector(descriptors_.size(),
        vk::WriteDescriptorSet{
            {}, 1, 0,
            1, vk::DescriptorType::eCombinedImageSampler, &info, nullptr, nullptr
        }
    ));
}

void VMesh::tick(float delta_seconds, TickInfo tick_info)
{
    Eigen::Matrix4f model_mat =
        ( rotation
        * Eigen::Translation3f(position)
        * Eigen::AlignedScaling3f(scale)
        ).matrix();

    // DON'T TOUCH THIS, EIGEN WILL CRASH
    Eigen::Matrix4f normal_mat = (tick_info.view * model_mat).inverse().transpose();

    UBO raw_ubo{
        model_mat,
        normal_mat,
        CACHE_SIZE,
        static_cast<uint32_t>(model_->atlas.get_min_mip()),
        static_cast<uint32_t>(model_->vgis.get_mip_level_count()),
    };
    ubo_.write_next({reinterpret_cast<std::byte*>(&raw_ubo), sizeof(raw_ubo)});

    {
        current_cut_ = model_->atlas.default_cut();

        auto nodes = current_cut_.get_nodes();

        static constexpr float TARGET_POLYGONS_PER_PIXEL = 1.f/64.f;

        auto wanted_mip =
            [&raw_ubo, &tick_info](NodeHandle node)
            {
                auto ss_size = float(tick_info.resolution.width * tick_info.resolution.height) *
                    node->projected_screenspace_area(raw_ubo.model, tick_info.view, tick_info.proj)
                        * TARGET_POLYGONS_PER_PIXEL;
                // Might be negative, so clamp first
                return std::size_t(std::clamp(std::log2(ss_size) / 2.f,
                    1.f, float(node->max_tessellation)));
            };

        auto priority =
            [&wanted_mip](NodeHandle node) -> int64_t
            {
                if (!node.has_children())
                {
                    return -std::numeric_limits<int64_t>::max();
                }
                int64_t saved = 1ll << (2*wanted_mip(node));
                for (std::size_t i = 0; i < 4; ++i)
                {
                    saved -= 1ll << (2*wanted_mip(node.child(i)));
                }
                return saved;
            };

        uint64_t polygon_limit = model_->vgis.cache_size_pixels();
        uint64_t total_polygons = 0;
        std::multimap<int64_t, NodeHandle, std::greater<>> queue;
        for (auto node : nodes)
        {
            auto mip = wanted_mip(node);
            current_cut_.set_mip(node, mip);
            queue.emplace(priority(node), node);
            total_polygons += 1ull << (2*mip);
        }

        const std::size_t node_limit = nodes.size() * 3;

        while (current_cut_.size() < node_limit
//            && total_polygons > polygon_limit
            && !queue.empty() && queue.begin()->first > 0)
        {
            auto node = queue.begin()->second;
            total_polygons -= queue.begin()->first;
            queue.erase(queue.begin());
            current_cut_.split(node);
            if (node.has_children())
            {
                for (std::size_t i = 0; i < 4; ++i)
                {
                    auto child = node.child(i);
                    current_cut_.set_mip(child, wanted_mip(child));
                    queue.emplace(priority(child), child);
                }
            }
        }
    }

    // TODO: remove, debug
//    ImGui::Begin("VMesh");
//    std::size_t kek = 0;
//    for (auto&[node, data] : current_cut_)
//    {
//        ImGui::PushID(std::to_string(kek).c_str());
//
//        if (ImGui::Button("X", ImVec2{20, 20}))
//        {
//            current_cut_.split(node);
//            ImGui::PopID();
//            break;
//        }
//
//
//        ImGui::SameLine();
//
//        std::size_t mip = data.mip;
//        ImGui::SliderScalar((std::to_string(kek++) + "(" + std::to_string(data.patch_idx) + ")").c_str(),
//            ImGuiDataType_U64, &mip,
//            &node->min_tessellation, &node->max_tessellation);
//        current_cut_.set_mip(node, mip);
//
//        ImGui::PopID();
//    }
//    ImGui::End();

    for (auto&[node, data] : current_cut_)
    {
        auto avail_gi = model_->vgis.bump_region(
            data.patch_idx,
            std::clamp(std::size_t(float(data.mip) - std::log2(node->size)),
                model_->atlas.get_min_mip(), model_->atlas.get_min_mip() + model_->vgis.get_mip_level_count() - 1),
            node->offset.x(),
            node->offset.y(),
            node->size
        );

        auto avail_mip = std::clamp(
            std::size_t(float(avail_gi) + std::log2(node->size)),
            std::size_t{1}, node->max_tessellation);

        if (avail_mip < data.mip)
        {
            current_cut_.set_mip(node, avail_mip);
        }
    }

    current_cut_.recalculate_side_mips();

    {
        auto pidata = reinterpret_cast<PerInstanceData*>(vbo_.get_current().data());

        auto mip_transform = [](std::size_t mip) { return static_cast<float>(1ul << mip); };

        for (auto&[node, data] : current_cut_)
        {
            pidata->mip = mip_transform(data.mip);
            pidata->side_mips.x() = mip_transform(data.side_mip[0]);
            pidata->side_mips.y() = mip_transform(data.side_mip[1]);
            pidata->side_mips.z() = mip_transform(data.side_mip[2]);
            pidata->side_mips.w() = mip_transform(data.side_mip[3]);

            pidata->param_space_offset.x() = node->offset.x();
            pidata->param_space_offset.y() = node->offset.y();
            pidata->param_space_size = node->size;

            pidata->patch_index = static_cast<uint32_t>(data.patch_idx);

            ++pidata;
        }
    }
}

void VMesh::record_commands(vk::CommandBuffer cb)
{
    vk::DeviceSize offset{vbo_.current_offset()};
    auto buf = vbo_.get();
    cb.bindVertexBuffers(0, 1, &buf, &offset);
    vbo_.next();

    std::array sets{descriptors_.read_next()};
    cb.bindDescriptorSets(vk::PipelineBindPoint::eGraphics, our_type_->get_pipeline_layout(), 1,
        static_cast<uint32_t>(sets.size()), sets.data(), 0, nullptr);

    cb.draw(4, static_cast<uint32_t>(current_cut_.size()), 0, 0);
}

VMeshSceneObjectType::VMeshSceneObjectType(IResourceManager* irm)
    : SceneObjectType(irm)
    , vertex_shader_{irm->get_shader("vgi.vert")}
    , tess_ctrl_shader_{irm->get_shader("vgi.tesc")}
    , tess_eval_shader_{irm->get_shader("vgi.tese")}
    , geom_shader_{irm->get_shader("wireframe.geom")}
    , fragment_shader_{irm->get_shader("phong.frag")}
{
}

void VMeshSceneObjectType::post_tick(float delta_seconds, TickInfo tick_info)
{
    for (auto&[key, model] : models_)
    {
        if (auto ptr = model.lock())
        {
            ptr->vgis.tick();
        }
    }
}

void VMeshSceneObjectType::record_pre_commands(vk::CommandBuffer cb)
{
    for (auto&[key, model] : models_)
    {
        if (auto ptr = model.lock())
        {
            ptr->vgis.record_commands(cb);
        }
    }
}

vk::UniquePipeline VMeshSceneObjectType::create_pipeline(SceneObjectType::PipelineCreateInfo info)
{
    auto device = resource_manager_->get_device();

    std::vector shader_stages{
        vk::PipelineShaderStageCreateInfo
            {{}, vk::ShaderStageFlagBits::eVertex, vertex_shader_->get(), "main"},
        vk::PipelineShaderStageCreateInfo
            {{}, vk::ShaderStageFlagBits::eTessellationControl, tess_ctrl_shader_->get(), "main"},
        vk::PipelineShaderStageCreateInfo
            {{}, vk::ShaderStageFlagBits::eTessellationEvaluation, tess_eval_shader_->get(), "main"},
        vk::PipelineShaderStageCreateInfo
            {{}, vk::ShaderStageFlagBits::eFragment, fragment_shader_->get(), "main"}
    };

    if (info.mode == ViewMode::Wireframe)
    {
        shader_stages.push_back(
            vk::PipelineShaderStageCreateInfo
                {{}, vk::ShaderStageFlagBits::eGeometry, geom_shader_->get(), "main"});
    }

    vk::VertexInputBindingDescription vertex_input_binding_description
        {0, sizeof(PerInstanceData), vk::VertexInputRate::eInstance};

    std::array vertex_input_attribute_descriptions{
        vk::VertexInputAttributeDescription
            {/* location */ 0, /* binding */ 0, vk::Format::eR32G32B32A32Sfloat, offsetof(PerInstanceData, side_mips)},
        vk::VertexInputAttributeDescription
            {/* location */ 1, /* binding */ 0, vk::Format::eR32Sfloat, offsetof(PerInstanceData, mip)},
        vk::VertexInputAttributeDescription
            {/* location */ 2, /* binding */ 0, vk::Format::eR32G32Sfloat, offsetof(PerInstanceData, param_space_offset)},
        vk::VertexInputAttributeDescription
            {/* location */ 3, /* binding */ 0, vk::Format::eR32Sfloat, offsetof(PerInstanceData, param_space_size)},
        vk::VertexInputAttributeDescription
            {/* location */ 4, /* binding */ 0, vk::Format::eR32Uint, offsetof(PerInstanceData, patch_index)}
    };

    vk::PipelineVertexInputStateCreateInfo vertex_input_info{
        {},
        1, &vertex_input_binding_description,
        static_cast<uint32_t>(vertex_input_attribute_descriptions.size()), vertex_input_attribute_descriptions.data()
    };

    vk::PipelineInputAssemblyStateCreateInfo input_assembly_info{
        {}, vk::PrimitiveTopology::ePatchList, false
    };

    vk::Viewport viewport{
        0, 0,
        static_cast<float>(info.viewport_extents.width), static_cast<float>(info.viewport_extents.height),
        0, 1
    };
    vk::Rect2D scissor{
        {0, 0},
        {info.viewport_extents.width, info.viewport_extents.height}
    };

    vk::PipelineViewportStateCreateInfo viewport_info{{}, 1, &viewport, 1, &scissor};

    vk::PipelineTessellationStateCreateInfo tesselation_info{
        {}, 4
    };

    vk::PipelineRasterizationStateCreateInfo rasterizer_info{
        {},
        /* depth clamp */ false,
        /* rasterizer discard */ false,
        vk::PolygonMode::eFill, vk::CullModeFlagBits::eBack, vk::FrontFace::eClockwise,
        /* depth bias */ false, 0, 0, 0,
        /* line width */ 1
    };

    vk::PipelineMultisampleStateCreateInfo multisample_info{
        {},
        vk::SampleCountFlagBits::e1,
        /* sample shading */ false, 1.f,
        /* sample mask */ nullptr,
        /* alpha to coverage */ false,
        /* alpha to one */ false
    };

    vk::PipelineColorBlendAttachmentState color_blend_attachment_state
        {false,
         vk::BlendFactor::eOne, vk::BlendFactor::eZero, vk::BlendOp::eAdd,
         vk::BlendFactor::eOne, vk::BlendFactor::eZero, vk::BlendOp::eAdd,
         vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG
             | vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA};

    vk::PipelineColorBlendStateCreateInfo color_blend_state_create_info
        {{}, false, vk::LogicOp::eCopy, 1, &color_blend_attachment_state, {0, 0, 0, 0}};

    vk::PipelineDepthStencilStateCreateInfo depth_stencil_info{
        {},
        /* enable depth test */  true,
        /* enable depth write */ true,
        /* comparator */vk::CompareOp::eLess,
        /* enable depth bounds test */ false,
        /* enable stencil test */ false,
    };

    std::array object_set_bindings{
        vk::DescriptorSetLayoutBinding{
            /* binding */ 0,
            vk::DescriptorType::eUniformBuffer,
            /* descriptor count */ 1,
            vk::ShaderStageFlagBits::eTessellationEvaluation,
            nullptr
        },
        vk::DescriptorSetLayoutBinding{
            /* binding */ 1,
            vk::DescriptorType::eCombinedImageSampler,
            /* descriptor count */ 1,
            vk::ShaderStageFlagBits::eFragment,
            nullptr
        },
        vk::DescriptorSetLayoutBinding{
            /* binding */ 2,
            vk::DescriptorType::eCombinedImageSampler,
            /* descriptor count */ 1,
            vk::ShaderStageFlagBits::eTessellationEvaluation,
            nullptr
        },
        vk::DescriptorSetLayoutBinding{
            /* binding */ 3,
            vk::DescriptorType::eStorageBuffer,
            /* descriptor count */ 1,
            vk::ShaderStageFlagBits::eTessellationEvaluation,
            nullptr
        },
    };

    instance_descriptor_set_layout_ = device.createDescriptorSetLayoutUnique(vk::DescriptorSetLayoutCreateInfo{
        {}, static_cast<uint32_t>(object_set_bindings.size()), object_set_bindings.data()
    });

    std::array desc_set_layouts{
        info.scene_descriptor_set_layout,
        instance_descriptor_set_layout_.get()
    };

    pipeline_layout_ = device.createPipelineLayoutUnique(vk::PipelineLayoutCreateInfo{
        {},
        static_cast<uint32_t>(desc_set_layouts.size()), desc_set_layouts.data(),
        /* push constant ranges */ 0, nullptr
    });

    return device.createGraphicsPipelineUnique({}, vk::GraphicsPipelineCreateInfo{
        {}, static_cast<uint32_t>(shader_stages.size()), shader_stages.data(),
        &vertex_input_info,
        &input_assembly_info,
        &tesselation_info,
        &viewport_info,
        &rasterizer_info,
        &multisample_info,
        &depth_stencil_info,
        &color_blend_state_create_info,
        nullptr,
        pipeline_layout_.get(),
        info.render_pass,
        0,
        {},
        -1
    }).value;
}

std::shared_ptr<Model> VMeshSceneObjectType::get_model(const std::filesystem::path& path)
{
    auto& weak_ptr = models_[path.string()];
    auto result = weak_ptr.lock();

    if (result == nullptr)
    {
        Model model{HierarchicalAtlas(path / "resampled")};

        std::vector<std::vector<const std::byte*>> gis;
        for (std::size_t i = 0; i < model.atlas.get_patch_count(); ++i)
        {
            auto& current = model.atlas.get_gis(i);
            gis.emplace_back();

            for (auto& gi : current)
            {
                gis.back().emplace_back(reinterpret_cast<const std::byte*>(gi.get_data()));
            }
        }

        model.vgis =
            resource_manager_->create_svt(CACHE_SIZE, model.atlas.get_patch_count(),
                vk::Format::eR32G32B32A32Sfloat, 2, model.atlas.get_min_mip(), std::move(gis));

        model.texture = resource_manager_->get_texture(std::array{path / "albedo.jpg", path / "specular.jpg"});

        model.sampler = resource_manager_->get_device().createSamplerUnique(vk::SamplerCreateInfo{
            {}, vk::Filter::eLinear, vk::Filter::eLinear,
            vk::SamplerMipmapMode::eLinear,
            vk::SamplerAddressMode::eClampToEdge,
            vk::SamplerAddressMode::eClampToEdge,
            vk::SamplerAddressMode::eClampToEdge,
            0, false, 1, false, vk::CompareOp::eAlways, 0, 0, vk::BorderColor::eIntOpaqueBlack, false
        });



        result = std::make_shared<Model>(std::move(model));
        weak_ptr = result;
    }

    return result;
}
