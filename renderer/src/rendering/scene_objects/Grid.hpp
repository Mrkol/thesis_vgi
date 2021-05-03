#pragma once

#include "../SceneObjectBase.hpp"

class GridSceneObject : public SceneObjectBase
{
public:
    void update_descriptor_sets(vk::DescriptorSet) override;
    void render(vk::CommandBuffer) override;
    const SceneObjectTypeFactory& get_scene_object_type_factory() const override;

private:
};

class GridSceneObjectType : public SceneObjectType
{
public:
    explicit GridSceneObjectType(SceneObjectTypeCreateInfo info);
};

MAKE_SCENE_OBJECT_TYPE_FACTORY(GridSceneObjectType);
