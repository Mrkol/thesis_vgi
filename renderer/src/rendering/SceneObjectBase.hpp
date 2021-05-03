#pragma once


class SceneObjectBase
{
public:
    virtual void Render() = 0;

    virtual ~SceneObjectBase() = default;
};
