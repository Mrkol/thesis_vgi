#pragma once

#include <memory>
#include <deque>


class SceneObjectBase;

class Scene
{
public:

    void add_object(std::unique_ptr<SceneObjectBase> object);

private:
    std::deque<SceneObjectBase*> scene_objects;
};
