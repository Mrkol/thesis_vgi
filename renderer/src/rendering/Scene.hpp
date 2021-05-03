#pragma once

#include <memory>
#include <deque>
#include <unordered_map>


class SceneObjectBase;
class SceneObjectType;

class Scene
{
public:

    void add_object(std::unique_ptr<SceneObjectBase> object);

private:
    std::deque<SceneObjectBase*> scene_objects;
    std::unordered_map<std::string_view, SceneObjectType*> object_types;
};
