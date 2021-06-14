#include "StaticMesh.hpp"

#include <tiny_obj_loader.h>

#include "VkHelpers.hpp"
#include "TupleHash.hpp"
#include "Utility.hpp"


namespace std
{

template<>
struct hash<Vertex>
{
    std::size_t operator()(const Vertex& vertex) const
    {
        return std::hash<std::tuple<float,float,float,float,float,float,float,float>>{}(
            std::make_tuple(vertex.position.x(), vertex.position.y(), vertex.position.z(), vertex.position.w(),
                vertex.normal.x(), vertex.normal.y(), vertex.normal.z(), vertex.normal.w())
            );
    }
};

}

struct UBO
{
    Eigen::Matrix4f model;
    Eigen::Matrix4f normal;
};

StaticMesh::StaticMesh(const std::filesystem::path& model)
    : texture_maps_{model.parent_path() / "albedo.jpg", model.parent_path() / "specular.jpg"}
{
    std::vector<std::filesystem::path> lod_paths;

    if (auto filename_string = model.filename().string();
        filename_string.ends_with("_LOD0"))
    {
        filename_string = filename_string.substr(0, filename_string.size() - 1);
        auto lod_file = model.parent_path() / (filename_string + std::to_string(0) + ".obj");
        std::size_t i = 0;
        while (exists(lod_file))
        {
            lod_paths.push_back(lod_file);
            lod_file = model.parent_path() / (filename_string + std::to_string(i++) + ".obj");
        }
    }
    else
    {
        lod_paths.push_back(model.parent_path() / (filename_string + ".obj"));
    }

    bb_min_.setConstant(std::numeric_limits<float>::max());
    bb_max_.setConstant(-std::numeric_limits<float>::max());

    std::unordered_map<Vertex, uint32_t> unique_verts;
    for (const auto& lod : lod_paths)
    {
        lod_offsets_.push_back(uint32_t(indices_.size()));

        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
        std::string warn, err;

        if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, lod.string().c_str()))
        {
            throw std::runtime_error(warn + err);
        }

        for (const auto& shape : shapes)
        {
            for (const auto& index : shape.mesh.indices)
            {
                Vertex vertex{
                    Eigen::Vector4f(
                        attrib.vertices[3 * index.vertex_index + 0],
                        attrib.vertices[3 * index.vertex_index + 1],
                        attrib.vertices[3 * index.vertex_index + 2],
                        attrib.texcoords[2 * index.texcoord_index + 0]),
                    Eigen::Vector4f(
                        attrib.normals[3 * index.normal_index + 0],
                        attrib.normals[3 * index.normal_index + 1],
                        attrib.normals[3 * index.normal_index + 2],
                        1.f - attrib.texcoords[2 * index.texcoord_index + 1])
                };

                bb_min_ = bb_min_.cwiseMin(vertex.position.block<3, 1>(0, 0)).eval();
                bb_max_ = bb_max_.cwiseMax(vertex.position.block<3, 1>(0, 0)).eval();

                auto it = unique_verts.find(vertex);
                if (it == unique_verts.end()) {
                    it = unique_verts.emplace(vertex, static_cast<uint32_t>(attributes_.size())).first;
                    attributes_.push_back(vertex);
                }

                indices_.push_back(it->second);
            }
        }
    }
    lod_offsets_.push_back(uint32_t(indices_.size()));
}

void StaticMesh::on_type_object_available(SceneObjectType& type)
{
    our_type_ = dynamic_cast<StaticMeshSceneObjectType*>(&type);

    auto irm = type.get_resource_manager();

    vbo_ = irm->create_vbo(attributes_.size() * sizeof(attributes_[0]));

    {
        auto mapped =  vbo_.map();
        std::memcpy(mapped, attributes_.data(), attributes_.size() * sizeof(attributes_[0]));
        vbo_.unmap();
    }

    ibo_ = irm->create_ibo(indices_.size() * sizeof(indices_[0]));

    {
        auto mapped = ibo_.map();
        std::memcpy(mapped, indices_.data(), indices_.size() * sizeof(indices_[0]));
        ibo_.unmap();
    }

    ubo_ = irm->create_ubo(sizeof(UBO));
    descriptors_ = irm->create_descriptor_set_ring(type.get_instance_descriptor_set_layout());
    descriptors_.write_ubo(ubo_, 0);


    texture_ = irm->get_texture(texture_maps_);

    sampler_ = irm->get_device().createSamplerUnique(vk::SamplerCreateInfo{
        {}, vk::Filter::eLinear, vk::Filter::eLinear,
        vk::SamplerMipmapMode::eLinear,
        vk::SamplerAddressMode::eClampToEdge,
        vk::SamplerAddressMode::eClampToEdge,
        vk::SamplerAddressMode::eClampToEdge,
        0, false, 1, false, vk::CompareOp::eAlways, 0, 0, vk::BorderColor::eIntOpaqueBlack, false
    });

    vk::DescriptorImageInfo info{
        sampler_.get(), texture_->get(), vk::ImageLayout::eShaderReadOnlyOptimal
    };
    descriptors_.write_all(std::vector(descriptors_.size(),
        vk::WriteDescriptorSet{
            {}, 1, 0,
            1, vk::DescriptorType::eCombinedImageSampler, &info, nullptr, nullptr
        }
    ));
}

void StaticMesh::tick(float delta_seconds, TickInfo tick_info)
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
        normal_mat
    };

    auto dist = (tick_info.view * model_mat * Eigen::Vector3f::Zero().homogeneous()).norm();
    auto size = std::cbrt((model_mat * (bb_max_ - bb_min_).homogeneous()).cwiseAbs().prod());
    current_lod_ = std::clamp(
        std::size_t(dist * size / 100 * float(lod_offsets_.size() - 1)),
        std::size_t{0}, lod_offsets_.size() - 2);

    ubo_.write_next({reinterpret_cast<std::byte*>(&raw_ubo), sizeof(raw_ubo)});
}

void StaticMesh::record_commands(vk::CommandBuffer cb)
{
    vk::DeviceSize offset{0};
    auto buf = vbo_.get();
    cb.bindVertexBuffers(0, 1, &buf, &offset);

    std::array sets{descriptors_.read_next()};
    cb.bindDescriptorSets(vk::PipelineBindPoint::eGraphics, our_type_->get_pipeline_layout(), 1,
        static_cast<uint32_t>(sets.size()), sets.data(), 0, nullptr);

    cb.bindIndexBuffer(ibo_.get(), lod_offsets_[current_lod_] * sizeof(indices_[0]), vk::IndexType::eUint32);
    cb.drawIndexed(lod_offsets_[current_lod_ + 1] - lod_offsets_[current_lod_], 1, 0, 0, 0);
}

const SceneObjectTypeFactory& StaticMesh::get_scene_object_type_factory() const
{
    static StaticMeshSceneObjectTypeFactory factory;
    return factory;
}

StaticMeshSceneObjectType::StaticMeshSceneObjectType(class IResourceManager* irm)
    : SceneObjectType(irm)
    , vertex_shader_{irm->get_shader("static.vert")}
    , fragment_shader_{irm->get_shader("phong.frag")}
    , geom_shader_{irm->get_shader("wireframe.geom")}
{
}

vk::UniquePipeline StaticMeshSceneObjectType::create_pipeline(SceneObjectType::PipelineCreateInfo info)
{
    auto device = resource_manager_->get_device();

    std::vector shader_stages{
        vk::PipelineShaderStageCreateInfo
            {{}, vk::ShaderStageFlagBits::eVertex, vertex_shader_->get(), "main"},
        vk::PipelineShaderStageCreateInfo
            {{}, vk::ShaderStageFlagBits::eFragment, fragment_shader_->get(), "main"},
    };

    if (info.mode == ViewMode::Wireframe)
    {
        shader_stages.push_back(
            vk::PipelineShaderStageCreateInfo
                {{}, vk::ShaderStageFlagBits::eGeometry, geom_shader_->get(), "main"});
    }

    vk::VertexInputBindingDescription vertex_input_binding_description
        {0, sizeof(Vertex), vk::VertexInputRate::eVertex};

    std::array vertex_input_attribute_descriptions{
        vk::VertexInputAttributeDescription
            {/* location */ 0, /* binding */ 0, vk::Format::eR32G32B32A32Sfloat, offsetof(Vertex, position)},
        vk::VertexInputAttributeDescription
            {/* location */ 1, /* binding */ 0, vk::Format::eR32G32B32A32Sfloat, offsetof(Vertex, normal)}
    };

    vk::PipelineVertexInputStateCreateInfo vertex_input_info{
        {},
        1, &vertex_input_binding_description,
        static_cast<uint32_t>(vertex_input_attribute_descriptions.size()), vertex_input_attribute_descriptions.data()
    };

    vk::PipelineInputAssemblyStateCreateInfo input_assembly_info{
        {}, vk::PrimitiveTopology::eTriangleList, false
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

    vk::PipelineRasterizationStateCreateInfo rasterizer_info{
        {},
        /* depth clamp */ false,
        /* rasterizer discard */ false,
        vk::PolygonMode::eFill, vk::CullModeFlagBits::eFront, vk::FrontFace::eClockwise,
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

    vk::PipelineColorBlendAttachmentState color_blend_attachment_state{
        false,
        vk::BlendFactor::eOne, vk::BlendFactor::eZero, vk::BlendOp::eAdd,
        vk::BlendFactor::eOne, vk::BlendFactor::eZero, vk::BlendOp::eAdd,
        vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG
            | vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA
    };

    vk::PipelineColorBlendStateCreateInfo color_blend_state_create_info
        {{}, false, vk::LogicOp::eCopy, 1, &color_blend_attachment_state, {0, 0, 0, 0}};

    vk::PipelineDepthStencilStateCreateInfo depth_stencil_info{
        {},
        /* enable depth test */  true,
        /* enable depth write */ true,
        /* comparator */ vk::CompareOp::eLess,
        /* enable depth bounds test */ false,
        /* enable stencil test */ false,
    };

    std::array object_set_bindings{
        vk::DescriptorSetLayoutBinding{
            /* binding */ 0,
            vk::DescriptorType::eUniformBuffer,
            /* descriptor count */ 1,
            vk::ShaderStageFlagBits::eVertex,
            nullptr
        },
        vk::DescriptorSetLayoutBinding{
            /* binding */ 1,
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
        nullptr,
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
