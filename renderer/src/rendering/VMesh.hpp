#pragma once

#include <filesystem>

#include "SceneObjectBase.hpp"


class VMesh : public SceneObjectBase
{
public:
    explicit VMesh(const std::filesystem::path& folder);

    void Render() override;
private:
};
