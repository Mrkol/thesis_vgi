#include "SceneObjectBase.hpp"

void SceneObjectType::begin_rendering(vk::CommandBuffer cb)
{
    cb.bindPipeline(vk::PipelineBindPoint::eGraphics, pipeline.get());
}
