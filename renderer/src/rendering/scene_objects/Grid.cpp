#include "Grid.hpp"

#include <Eigen/Dense>

#include "VkHelpers.hpp"


struct Vertex
{
    Eigen::Vector3f data;
};

struct UBO
{
    Eigen::Matrix4f model;
};

static_assert(sizeof(Vertex) == 4*3);


std::size_t grid_size_to_vert_count(std::size_t size)
{
    return 4*(size + 1);
}

GridSceneObject::GridSceneObject(std::size_t size)
    : size_(size)
{
}

void GridSceneObject::on_type_object_available(SceneObjectType& type)
{
    // Type is guaranteed to be ours
    our_type_ = dynamic_cast<GridSceneObjectType*>(&type);

    // No need to release the old one, as the entire type object was recreated if this was called
    vbo_ = our_type_->acquire_vbo(size_);
    uniform_buffer_ = our_type_->get_resource_manager()->create_ubo(sizeof(UBO));
    uniforms_ =
        our_type_->get_resource_manager()->create_descriptor_set_ring(our_type_->get_instance_descriptor_set_layout());
    uniforms_.write_ubo(uniform_buffer_, 0);
}

void GridSceneObject::tick(float delta_seconds, TickInfo tick_info)
{
//    Eigen::Quaternionf q = rotation * Eigen::AngleAxisf(0.0001f, Eigen::Vector3f::UnitZ());
//    rotation = q;

    UBO ubo{
        ( rotation
        * Eigen::Translation3f(position)
        * Eigen::AlignedScaling3f(scale)
        ).matrix()
    };
    uniform_buffer_.write_next({reinterpret_cast<std::byte*>(&ubo), sizeof(ubo)});
}

void GridSceneObject::record_commands(vk::CommandBuffer cb)
{
    std::array offsets{vk::DeviceSize{0}};
    cb.bindVertexBuffers(0, 1, &vbo_, offsets.data());
    std::array sets{uniforms_.read_next()};
    cb.bindDescriptorSets(vk::PipelineBindPoint::eGraphics, our_type_->get_pipeline_layout(), 1,
        static_cast<uint32_t>(sets.size()), sets.data(), 0, nullptr);
    cb.draw(static_cast<uint32_t>(grid_size_to_vert_count(size_)), 1, 0, 0);
}

const SceneObjectTypeFactory& GridSceneObject::get_scene_object_type_factory() const
{
    static GridSceneObjectTypeFactory factory;
    return factory;
}

GridSceneObject::~GridSceneObject()
{
    if (our_type_)
    {
        our_type_->release_vbo(size_);
    }
}

GridSceneObjectType::GridSceneObjectType(IResourceManager* irm)
    : SceneObjectType(irm)
    , vertex_shader_{VkHelpers::read_shader("debug.vert")}
    , fragment_shader_{VkHelpers::read_shader("debug.frag")}
{
}

vk::UniquePipeline GridSceneObjectType::create_pipeline(PipelineCreateInfo info)
{
    auto device = resource_manager_->get_device();
    auto vert_module = device.createShaderModuleUnique(vk::ShaderModuleCreateInfo{
        {},
        vertex_shader_.size(), reinterpret_cast<const uint32_t*>(vertex_shader_.data())
    });

    auto frag_module = device.createShaderModuleUnique(vk::ShaderModuleCreateInfo{
        {},
        fragment_shader_.size(), reinterpret_cast<const uint32_t*>(fragment_shader_.data())
    });

    std::array<vk::PipelineShaderStageCreateInfo, 2> shader_stages{
        vk::PipelineShaderStageCreateInfo{{}, vk::ShaderStageFlagBits::eVertex, vert_module.get(), "main"},
        vk::PipelineShaderStageCreateInfo{{}, vk::ShaderStageFlagBits::eFragment, frag_module.get(), "main"}};

    vk::VertexInputBindingDescription vertex_input_binding_description
        {0, sizeof(Vertex), vk::VertexInputRate::eVertex};

    std::array vertex_input_attribute_descriptions{
        vk::VertexInputAttributeDescription
            {/* location */ 0, /* binding */ 0, vk::Format::eR32G32B32Sfloat, 0}
    };

    vk::PipelineVertexInputStateCreateInfo vertex_input_info{
        {},
        1, &vertex_input_binding_description,
        static_cast<uint32_t>(vertex_input_attribute_descriptions.size()), vertex_input_attribute_descriptions.data()
    };

    vk::PipelineInputAssemblyStateCreateInfo input_assembly_info{
        {}, vk::PrimitiveTopology::eLineList, false
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

    vk::DescriptorSetLayoutBinding object_set{
        /* binding */ 0,
        vk::DescriptorType::eUniformBuffer,
        /* descriptor count */ 1,
        vk::ShaderStageFlagBits::eVertex,
        nullptr
    };

    instance_descriptor_set_layout_ = device.createDescriptorSetLayoutUnique(vk::DescriptorSetLayoutCreateInfo{
        {}, 1, &object_set
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

void fill_grid_vbo(UniqueVmaBuffer& buffer, std::size_t grid_size)
{
    auto data = reinterpret_cast<Vertex*>(buffer.map());

    for (std::size_t i = 0; i <= grid_size; ++i)
    {
        auto percentage = 2.f * static_cast<float>(i) / static_cast<float>(grid_size) - 1.f;

        data++->data = {percentage, -1, 0};
        data++->data = {percentage,  1, 0};
        data++->data = {-1, percentage, 0};
        data++->data = { 1, percentage, 0};
    }

    buffer.unmap();
}

vk::Buffer GridSceneObjectType::acquire_vbo(std::size_t grid_size)
{
    auto it = buffers_for_sizes_.find(grid_size);

    if (it == buffers_for_sizes_.end())
    {
        it = buffers_for_sizes_.emplace(grid_size,
            PerSizeData{
                0,
                resource_manager_->create_vbo(sizeof(Vertex) * grid_size_to_vert_count(grid_size))
            }).first;

        fill_grid_vbo(it->second.buffer, grid_size);
    }

    ++it->second.users;
    return it->second.buffer.get();
}

void GridSceneObjectType::release_vbo(std::size_t grid_size)
{
    auto it = buffers_for_sizes_.find(grid_size);
    --it->second.users;

    if (it->second.users == 0)
    {
        buffers_for_sizes_.erase(it);
    }
}
