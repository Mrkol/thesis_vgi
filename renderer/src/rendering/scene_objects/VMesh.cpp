#include "VMesh.hpp"

#include <stb_image.h>
#include <imgui.h>
#include <map>

#include <VkHelpers.hpp>
#include <Utility.hpp>



static constexpr std::size_t CACHE_SIZE = 128;

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

VMesh::VMesh(const std::filesystem::path& folder)
    : texture_maps_{UniqueStbImage{folder / "albedo.jpg"}, UniqueStbImage{folder / "specular.jpg"}}
    , atlas_(folder / "resampled")
    , current_cut_(atlas_.default_cut())
{
    for (auto& map : texture_maps_)
    {
        AD_HOC_ASSERT(map.width() == texture_maps_[0].width() && map.height() == texture_maps_[0].height(), "Oops");
    }
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

    vbo_ = irm->create_dynamic_vbo(sizeof(PerInstanceData)
        * (1ull << atlas_.get_hierarchy_depth()) * atlas_.get_patch_count());

    ubo_ = irm->create_ubo(sizeof(UBO));
    descriptors_ = irm->create_descriptor_set_ring(type.get_instance_descriptor_set_layout());
    descriptors_.write_ubo(ubo_, 0);

    std::vector<std::vector<const std::byte*>> gis;
    for (std::size_t i = 0; i < atlas_.get_patch_count(); ++i)
    {
        auto& current = atlas_.get_gis(i);
        gis.emplace_back();

        for (auto& gi : current)
        {
            gis.back().emplace_back(reinterpret_cast<const std::byte*>(gi.get_data()));
        }
    }

    vgis_ = irm->create_svt(CACHE_SIZE, atlas_.get_patch_count(),
        vk::Format::eR32G32B32A32Sfloat, 2, atlas_.get_min_mip(), std::move(gis));

    vk::DescriptorImageInfo descriptor_image_info{
        vgis_.sampler(), vgis_.view(), vk::ImageLayout::eShaderReadOnlyOptimal
    };

    descriptors_.write_all(std::vector(descriptors_.size(), vk::WriteDescriptorSet{
        {}, 1, 0,
        1, vk::DescriptorType::eCombinedImageSampler,
        &descriptor_image_info, nullptr, nullptr
    }));

    descriptors_.write_sbo(vgis_.get_indirection_table_sbo(), 2);





    textures_ = irm->create_texture(
        {static_cast<uint32_t>(texture_maps_[0].width()), static_cast<uint32_t>(texture_maps_[0].height())},
        texture_maps_.size());


    auto cb = irm->begin_single_time_commands();

    textures_.transfer_layout(cb.get(), vk::ImageLayout::eUndefined, vk::ImageLayout::eTransferDstOptimal,
        {}, vk::AccessFlagBits::eTransferWrite,
        vk::PipelineStageFlagBits::eTopOfPipe, vk::PipelineStageFlagBits::eTransfer);


    std::size_t staging_size = TEXTURE_MAP_COUNT * texture_maps_[0].width() * texture_maps_[0].height() * 4;
    auto staging = irm->create_staging_buffer(staging_size);

    auto mapped = staging.map();

    // TODO: make not fubar
    for (auto& map : texture_maps_)
    {
        std::size_t size = map.width() * map.height() * 4;
        std::memcpy(mapped, map.data(), size);
        mapped += size;
    }

    staging.unmap();

    std::array<vk::BufferImageCopy, TEXTURE_MAP_COUNT> copy_ops;
    std::size_t curr_offset = 0;
    for (std::size_t i = 0; i < TEXTURE_MAP_COUNT; ++i)
    {
        copy_ops[i] = vk::BufferImageCopy{
            static_cast<uint32_t>(curr_offset),
            static_cast<uint32_t>(texture_maps_[i].width()), static_cast<uint32_t>(texture_maps_[i].height()),
            vk::ImageSubresourceLayers{
                vk::ImageAspectFlagBits::eColor,
                0, static_cast<uint32_t>(i), 1
            },
            vk::Offset3D{0, 0, 0},
            vk::Extent3D{
                static_cast<uint32_t>(texture_maps_[i].width()),
                static_cast<uint32_t>(texture_maps_[i].height()),
                1
            }
        };
        curr_offset += texture_maps_[i].width() * texture_maps_[i].height() * 4;
    }

    cb->copyBufferToImage(staging.get(), textures_.get(), vk::ImageLayout::eTransferDstOptimal, copy_ops);

    textures_.transfer_layout(cb.get(), vk::ImageLayout::eTransferDstOptimal, vk::ImageLayout::eShaderReadOnlyOptimal,
        vk::AccessFlagBits::eTransferWrite, vk::AccessFlagBits::eShaderRead,
        vk::PipelineStageFlagBits::eTransfer, vk::PipelineStageFlagBits::eFragmentShader);

    irm->finish_single_time_commands(std::move(cb));

    sampler_ = irm->get_device().createSamplerUnique(vk::SamplerCreateInfo{
        {}, vk::Filter::eLinear, vk::Filter::eLinear,
        vk::SamplerMipmapMode::eLinear,
        vk::SamplerAddressMode::eClampToEdge,
        vk::SamplerAddressMode::eClampToEdge,
        vk::SamplerAddressMode::eClampToEdge,
        0, false, 1, false, vk::CompareOp::eAlways, 0, 0, vk::BorderColor::eIntOpaqueBlack, false
    });

    textures_view_ = irm->get_device().createImageViewUnique(vk::ImageViewCreateInfo{
        {}, textures_.get(), vk::ImageViewType::e2DArray, vk::Format::eR8G8B8A8Srgb,
        vk::ComponentMapping{},
        vk::ImageSubresourceRange{
            vk::ImageAspectFlagBits::eColor,
            0, 1, 0, TEXTURE_MAP_COUNT
        }
    });

    vk::DescriptorImageInfo info{
        sampler_.get(), textures_view_.get(), vk::ImageLayout::eShaderReadOnlyOptimal
    };
    descriptors_.write_all(std::vector(descriptors_.size(),
        vk::WriteDescriptorSet{
            {}, 3, 0,
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
        static_cast<uint32_t>(atlas_.get_min_mip()),
        static_cast<uint32_t>(vgis_.get_mip_level_count()),
    };
    ubo_.write_next({reinterpret_cast<std::byte*>(&raw_ubo), sizeof(raw_ubo)});

    {
        current_cut_ = atlas_.default_cut();

        auto nodes = current_cut_.get_nodes();

        static constexpr float TARGET_POLYGONS_PER_PIXEL = 1.f/64.f;

        auto wanted_mip =
            [&raw_ubo, &tick_info](NodeHandle node)
            {
                auto ss_size = float(tick_info.resolution.width * tick_info.resolution.height) *
                    node->projected_screenspace_area(raw_ubo.model, tick_info.view, tick_info.proj)
                        * TARGET_POLYGONS_PER_PIXEL;
                return std::size_t(std::clamp(std::log2(ss_size) / 2.f,
                    float(node->min_tessellation), float(node->max_tessellation)));
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

        uint64_t polygon_limit = vgis_.cache_size_pixels();
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
        auto avail_mip = vgis_.bump_region(
            data.patch_idx,
            std::clamp(std::size_t(float(data.mip) - std::log2(node->size)),
                atlas_.get_min_mip(), atlas_.get_min_mip() + vgis_.get_mip_level_count() - 1),
            node->offset.x(),
            node->offset.y(),
            node->size
        );

        current_cut_.set_mip(node,
            std::clamp(std::size_t(float(avail_mip) + std::log2(node->size)),
                node->min_tessellation, node->max_tessellation));
    }

    current_cut_.recalculate_side_mips();

    {
        auto pidata = reinterpret_cast<PerInstanceData*>(vbo_.get_current().data());

        auto mip_transform = [](std::size_t mip) { return static_cast<float>(1 << mip); };

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

    vgis_.tick();
}

void VMesh::record_pre_commands(vk::CommandBuffer cb)
{
    vgis_.record_commands(cb);
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
{
    load_shaders();
}

vk::UniquePipeline VMeshSceneObjectType::create_pipeline(SceneObjectType::PipelineCreateInfo info)
{
    auto device = resource_manager_->get_device();
    auto vert_module = device.createShaderModuleUnique(vk::ShaderModuleCreateInfo{
        {}, vertex_shader_.size(), reinterpret_cast<const uint32_t*>(vertex_shader_.data())
    });

    auto frag_module = device.createShaderModuleUnique(vk::ShaderModuleCreateInfo{
        {}, fragment_shader_.size(), reinterpret_cast<const uint32_t*>(fragment_shader_.data())
    });

    auto tess_ctrl_module = device.createShaderModuleUnique(vk::ShaderModuleCreateInfo{
        {}, tess_ctrl_shader_.size(), reinterpret_cast<const uint32_t*>(tess_ctrl_shader_.data())
    });

    auto tess_eval_module = device.createShaderModuleUnique(vk::ShaderModuleCreateInfo{
        {}, tess_eval_shader_.size(), reinterpret_cast<const uint32_t*>(tess_eval_shader_.data())
    });

    // TODO: proper viewmode system for wireframes and lighting toggles
//    auto geom_module = device.createShaderModuleUnique(vk::ShaderModuleCreateInfo{
//        {}, geom_shader_.size(), reinterpret_cast<const uint32_t*>(geom_shader_.data())
//    });

    std::array shader_stages{
        vk::PipelineShaderStageCreateInfo
            {{}, vk::ShaderStageFlagBits::eVertex, vert_module.get(), "main"},
        vk::PipelineShaderStageCreateInfo
            {{}, vk::ShaderStageFlagBits::eFragment, frag_module.get(), "main"},
        vk::PipelineShaderStageCreateInfo
            {{}, vk::ShaderStageFlagBits::eTessellationControl, tess_ctrl_module.get(), "main"},
        vk::PipelineShaderStageCreateInfo
            {{}, vk::ShaderStageFlagBits::eTessellationEvaluation, tess_eval_module.get(), "main"},
//        vk::PipelineShaderStageCreateInfo
//            {{}, vk::ShaderStageFlagBits::eGeometry, geom_module.get(), "main"}
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
        vk::DescriptorSetLayoutBinding{
            /* binding */ 3,
            vk::DescriptorType::eCombinedImageSampler,
            /* descriptor count */ 1,
            vk::ShaderStageFlagBits::eFragment,
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

void VMeshSceneObjectType::reload_shaders()
{
    load_shaders();
}

void VMeshSceneObjectType::load_shaders()
{
    vertex_shader_ = VkHelpers::read_shader("vgi.vert");
    fragment_shader_ = VkHelpers::read_shader("vgi.frag");
    tess_ctrl_shader_ = VkHelpers::read_shader("vgi.tesc");
    tess_eval_shader_ = VkHelpers::read_shader("vgi.tese");
    geom_shader_ = VkHelpers::read_shader("vgi.geom");
}
