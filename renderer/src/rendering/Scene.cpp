#include "Scene.hpp"

#include "SceneObjectBase.hpp"


void Scene::add_object(std::unique_ptr<SceneObjectBase> object)
{
    scene_objects.push_back(object.release());
}
