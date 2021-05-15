#include "VMesh.hpp"

#include <VkHelpers.hpp>
#include <imgui.h>


static constexpr std::size_t CACHE_SIZE = 64;

struct __attribute__ ((packed)) PerInstanceData
{
    Eigen::Vector4f side_mips;
    float mip;
    Eigen::Vector2f param_space_offset;
    float param_space_size;
    uint32_t patch_index;
};

static_assert(sizeof(PerInstanceData) == 8*sizeof(float) + sizeof(uint32_t));

struct UBO
{
    Eigen::Matrix4f model;
    uint32_t cache_side_pages;
    uint32_t min_mip;
    uint32_t mip_level_count;
};

VMesh::VMesh(const std::filesystem::path& folder)
    : atlas(folder)
    , current_cut(atlas.default_cut())
{
}

const SceneObjectTypeFactory& VMesh::get_scene_object_type_factory() const
{
    static VMeshSceneObjectTypeFactory factory;
    return factory;
}

void VMesh::on_type_object_available(SceneObjectType& type)
{
    our_type = dynamic_cast<VMeshSceneObjectType*>(&type);

    auto irm = type.get_resource_manager();

    vbo = irm->create_vbo(sizeof(PerInstanceData)
        * (1 << atlas.get_hierarchy_depth()) * atlas.get_patch_count());
    vbo.map();

    ubo = irm->create_ubo(sizeof(UBO));
    descriptors = irm->create_descriptor_set_ring(type.get_instance_descriptor_set_layout());
    descriptors.write_ubo(ubo, 0);

    std::vector<std::vector<const std::byte*>> gis;
    for (std::size_t i = 0; i < atlas.get_patch_count(); ++i)
    {
        auto& current = atlas.get_gis(i);
        gis.emplace_back();

        for (auto& gi : current)
        {
            gis.back().emplace_back(reinterpret_cast<const std::byte*>(gi.get_data()));
        }
    }

    vgis = irm->create_svt(CACHE_SIZE, atlas.get_patch_count(),
        vk::Format::eR32G32B32Sfloat, atlas.get_min_mip(), std::move(gis));

    vk::DescriptorImageInfo descriptor_image_info{
        vgis.sampler(), vgis.view(), vk::ImageLayout::eShaderReadOnlyOptimal
    };

    descriptors.write_all(std::vector(descriptors.size(), vk::WriteDescriptorSet{
        {}, 1, 0,
        1, vk::DescriptorType::eCombinedImageSampler,
        &descriptor_image_info, nullptr, nullptr
    }));

    descriptors.write_sbo(vgis.get_indirection_table_sbo(), 2);
}

void VMesh::tick()
{
    // TODO: optimize the cut

    auto nodes_to_render = current_cut.dump();

    // Keeps the order stable for easier debug
    // TODO: remove
    std::sort(nodes_to_render.begin(), nodes_to_render.end(),
        [](const auto& p, const auto& q)
        {
            return p.second.patch_idx < q.second.patch_idx;
        });


    // TODO: remove, debug
    ImGui::Begin("VMesh");
    std::size_t kek = 0;
    for (auto&[node, data] : nodes_to_render)
    {
        ImGui::PushID(std::to_string(kek).c_str());

        if (ImGui::Button("X", ImVec2{20, 20}))
        {
            current_cut.split(node);
        }
        ImGui::SameLine();

        std::size_t mip = data.mip;
        ImGui::SliderScalar((std::to_string(kek++) + "(" + std::to_string(data.patch_idx) + ")").c_str(),
            ImGuiDataType_U64, &mip,
            &node->min_tessellation, &node->max_tessellation);
        current_cut.set_mip(node, mip);

        ImGui::PopID();
    }
    ImGui::End();

    for (auto&[node, data] : nodes_to_render)
    {
        vgis.bump_region(
            data.patch_idx,
            std::clamp(std::size_t(std::log2(float(1 << data.mip) / node->size)),
                atlas.get_min_mip(), atlas.get_min_mip() + vgis.get_mip_level_count() - 1),
            node->offset.x(),
            node->offset.y(),
            node->size
        );
    }

    {
        UBO raw_ubo{
            ( rotation
                * Eigen::Translation3f(position)
                * Eigen::AlignedScaling3f(scale)
            ).matrix(),
            CACHE_SIZE,
            static_cast<uint32_t>(atlas.get_min_mip()),
            static_cast<uint32_t>(vgis.get_mip_level_count()),
        };
        ubo.write_next({reinterpret_cast<std::byte*>(&raw_ubo), sizeof(raw_ubo)});
    }

    {
        auto pidata = reinterpret_cast<PerInstanceData*>(vbo.data());

        auto mip_transform = [](std::size_t mip) { return static_cast<float>(1 << mip); };

        for (auto&[node, data] : nodes_to_render)
        {
            auto m = mip_transform(data.mip);
            pidata->mip = m;
            // TODO: proper neighbor lookup
            pidata->side_mips.x() = m;
            pidata->side_mips.y() = m;
            pidata->side_mips.z() = m;
            pidata->side_mips.w() = m;

            pidata->param_space_offset.x() = node->offset.x();
            pidata->param_space_offset.y() = node->offset.y();
            pidata->param_space_size = node->size;

            pidata->patch_index = data.patch_idx;

            ++pidata;
        }
    }

    vgis.tick();
}

void VMesh::record_pre_commands(vk::CommandBuffer cb)
{
    vgis.record_commands(cb);
}

void VMesh::record_commands(vk::CommandBuffer cb)
{
    std::array offsets{vk::DeviceSize{0}};
    auto buf = vbo.get();
    cb.bindVertexBuffers(0, 1, &buf, offsets.data());

    std::array sets{descriptors.read_next()};
    cb.bindDescriptorSets(vk::PipelineBindPoint::eGraphics, our_type->get_pipeline_layout(), 1,
        static_cast<uint32_t>(sets.size()), sets.data(), 0, nullptr);

    cb.draw(4, current_cut.size(), 0, 0);
}

VMeshSceneObjectType::VMeshSceneObjectType(IResourceManager* irm)
    : SceneObjectType(irm)
{
    load_shaders();
}

vk::UniquePipeline VMeshSceneObjectType::create_pipeline(SceneObjectType::PipelineCreateInfo info)
{
    auto device = resource_manager->get_device();
    auto vert_module = device.createShaderModuleUnique(vk::ShaderModuleCreateInfo{
        {}, vertex_shader.size(), reinterpret_cast<const uint32_t*>(vertex_shader.data())
    });

    auto frag_module = device.createShaderModuleUnique(vk::ShaderModuleCreateInfo{
        {}, fragment_shader.size(), reinterpret_cast<const uint32_t*>(fragment_shader.data())
    });

    auto tess_ctrl_module = device.createShaderModuleUnique(vk::ShaderModuleCreateInfo{
        {}, tess_ctrl_shader.size(), reinterpret_cast<const uint32_t*>(tess_ctrl_shader.data())
    });

    auto tess_eval_module = device.createShaderModuleUnique(vk::ShaderModuleCreateInfo{
        {}, tess_eval_shader.size(), reinterpret_cast<const uint32_t*>(tess_eval_shader.data())
    });

    std::array shader_stages{
        vk::PipelineShaderStageCreateInfo
            {{}, vk::ShaderStageFlagBits::eVertex, vert_module.get(), "main"},
        vk::PipelineShaderStageCreateInfo
            {{}, vk::ShaderStageFlagBits::eFragment, frag_module.get(), "main"},
        vk::PipelineShaderStageCreateInfo
            {{}, vk::ShaderStageFlagBits::eTessellationControl, tess_ctrl_module.get(), "main"},
        vk::PipelineShaderStageCreateInfo
            {{}, vk::ShaderStageFlagBits::eTessellationEvaluation, tess_eval_module.get(), "main"}
    };

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
        vk::PolygonMode::eFill, vk::CullModeFlagBits::eNone, vk::FrontFace::eClockwise,
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
            vk::ShaderStageFlagBits::eTessellationEvaluation,
            nullptr
        },
        vk::DescriptorSetLayoutBinding{
            /* binding */ 2,
            vk::DescriptorType::eStorageBuffer,
            /* descriptor count */ 1,
            vk::ShaderStageFlagBits::eTessellationEvaluation,
            nullptr
        },
    };

    instance_descriptor_set_layout = device.createDescriptorSetLayoutUnique(vk::DescriptorSetLayoutCreateInfo{
        {}, static_cast<uint32_t>(object_set_bindings.size()), object_set_bindings.data()
    });

    std::array desc_set_layouts{
        info.scene_descriptor_set_layout,
        instance_descriptor_set_layout.get()
    };

    pipeline_layout = device.createPipelineLayoutUnique(vk::PipelineLayoutCreateInfo{
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
        pipeline_layout.get(),
        info.render_pass,
        0,
        {},
        -1
    });
}

void VMeshSceneObjectType::reload_shaders()
{
    load_shaders();
}

void VMeshSceneObjectType::load_shaders()
{
    vertex_shader = VkHelpers::read_shader("vgi.vert");
    fragment_shader = VkHelpers::read_shader("vgi.frag");
    tess_ctrl_shader = VkHelpers::read_shader("vgi.tesc");
    tess_eval_shader = VkHelpers::read_shader("vgi.tese");
}
