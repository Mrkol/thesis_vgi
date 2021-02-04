#pragma once

#include <filesystem>

namespace Gridify
{

struct Arguments
{
    std::filesystem::path input_file;
    std::filesystem::path output_directory;
};

struct Result
{
    std::filesystem::path output_file;
};

std::vector<Result> gridify(Arguments args);

}
