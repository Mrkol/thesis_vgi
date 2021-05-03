#include "Grid.hpp"

#include "VkHelpers.hpp"


struct Vertex
{
    float x, y, z;
};

void GridSceneObject::update_descriptor_sets(vk::DescriptorSet)
{
}

void GridSceneObject::render(vk::CommandBuffer)
{
}

const SceneObjectTypeFactory& GridSceneObject::get_scene_object_type_factory() const
{
    static GridSceneObjectTypeFactory factory;
    return factory;
}

GridSceneObjectType::GridSceneObjectType(SceneObjectTypeCreateInfo info)
{
    auto vert_source = VkHelpers::read_shader("debug.vert");
    auto vert_module = info.device.createShaderModuleUnique(vk::ShaderModuleCreateInfo{
        {},
        vert_source.size(), reinterpret_cast<const uint32_t*>(vert_source .data())
    });

    auto frag_source = VkHelpers::read_shader("debug.frag");
    auto frag_module = info.device.createShaderModuleUnique(vk::ShaderModuleCreateInfo{
        {},
        frag_source.size(), reinterpret_cast<const uint32_t*>(frag_source.data())
    });

    std::array<vk::PipelineShaderStageCreateInfo, 2> shader_stages{
        vk::PipelineShaderStageCreateInfo{{}, vk::ShaderStageFlagBits::eVertex, vert_module.get(), "grid"},
        vk::PipelineShaderStageCreateInfo{{}, vk::ShaderStageFlagBits::eFragment, frag_module.get(), "grid"}};
    
    vk::VertexInputBindingDescription vertex_input_binding_description
        {0, sizeof(Vertex), vk::VertexInputRate::eVertex};

    std::array vertex_input_attribute_descriptions{
        vk::VertexInputAttributeDescription
            {/* location */ 1, /* binding */ 0, vk::Format::eR32G32B32Sfloat, 0}
    };

    vk::PipelineVertexInputStateCreateInfo vertex_input_info{
        {},
        1, &vertex_input_binding_description,
        static_cast<uint32_t>(vertex_input_attribute_descriptions.size()), vertex_input_attribute_descriptions.data()
    };

    vk::PipelineInputAssemblyStateCreateInfo triangle_input_assembly_info{
        {}, vk::PrimitiveTopology::eTriangleList, false
    };

    vk::Viewport viewport{0, 0,
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

    vk::DescriptorSetLayoutBinding object_set{
        /* binding */ OBJECT_DESCRIPTOR_SET_BINDING ,
                      vk::DescriptorType::eUniformBuffer,
        /* descriptor count */ 1,
                      vk::ShaderStageFlagBits::eVertex,
                      nullptr
    };

    instance_descriptor_set_layout = info.device.createDescriptorSetLayoutUnique(vk::DescriptorSetLayoutCreateInfo{
        {}, 1, &object_set
    });

    std::array desc_set_layouts{
        info.global_descriptor_set_layout,
        instance_descriptor_set_layout.get()
    };

    pipeline_layout = info.device.createPipelineLayoutUnique(vk::PipelineLayoutCreateInfo{
        {},
        static_cast<uint32_t>(desc_set_layouts.size()), desc_set_layouts.data(),
        /* push constant ranges */ 0, nullptr
    });

    
    vk::PipelineInputAssemblyStateCreateInfo line_input_assembly_info{
        {}, vk::PrimitiveTopology::eLineStrip, false
    };

    std::array dyn_states {vk::DynamicState::eViewport};
    vk::PipelineDynamicStateCreateInfo dynamic_state_create_info{
        {}, static_cast<uint32_t>(dyn_states.size()), dyn_states.data()
    };

    pipeline = info.device.createGraphicsPipelineUnique({}, vk::GraphicsPipelineCreateInfo{
        {}, 2, shader_stages.data(),
        &vertex_input_info,
        &line_input_assembly_info,
        nullptr,
        &viewport_info,
        &rasterizer_info,
        &multisample_info,
        nullptr,
        &color_blend_state_create_info,
        nullptr, //&dynamic_state_create_info, TODO: fix
        pipeline_layout.get(),
        info.render_pass,
        0,
        {},
        -1
    });
}
