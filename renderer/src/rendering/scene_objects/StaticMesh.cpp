#include "StaticMesh.hpp"

StaticMesh::StaticMesh(const std::filesystem::path& folder)
{

}

void StaticMesh::on_type_object_available(SceneObjectType& type)
{

}

void StaticMesh::tick(float delta_seconds, TickInfo tick_info)
{
    SceneObjectBase::tick(delta_seconds, tick_info);
}

void StaticMesh::record_commands(vk::CommandBuffer cb)
{

}

const SceneObjectTypeFactory& StaticMesh::get_scene_object_type_factory() const
{
    static StaticMeshSceneObjectTypeFactory factory;
    return factory;
}

StaticMeshSceneObjectType::StaticMeshSceneObjectType(class IResourceManager* irm) : SceneObjectType(irm)
{

}

vk::UniquePipeline StaticMeshSceneObjectType::create_pipeline(SceneObjectType::PipelineCreateInfo info)
{
    return vk::UniquePipeline();
}

void StaticMeshSceneObjectType::reload_shaders()
{
    SceneObjectType::reload_shaders();
}

void StaticMeshSceneObjectType::load_shaders()
{

}
