#include "Scene.hpp"

#include "SceneObjectBase.hpp"


void Scene::add_object(std::unique_ptr<SceneObjectBase> object)
{
    auto& factory = object->get_scene_object_type_factory();
    auto type_name = factory.get_name();

    if (!object_types.contains(type_name))
    {
        //object_types.insert(type_name, factory.create({}));
    }

    scene_objects.push_back(object.release());
}
