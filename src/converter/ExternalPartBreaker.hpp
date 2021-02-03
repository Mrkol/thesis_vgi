#pragma once

#include <filesystem>

namespace ExternalPartBreaker
{

struct Arguments
{
    std::filesystem::path input_file;
    std::filesystem::path output_directory;
};

struct Result
{
    std::filesystem::path output_file;
    // vm[i] is the index of ith vertex in output_file inside of input_file
    std::vector<size_t> vertex_mapping;
    size_t poly_count;
};

std::vector<Result> break_into_parts(Arguments args);

}
