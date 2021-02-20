#include <cxxopts.hpp>

#include "ToPlainConverters.hpp"
#include "Gridify.hpp"
#include "InCoreClustering.hpp"


int main(int argc, char** argv)
{
    cxxopts::Options options("VGI Converter", "Converts .obj files to VGI format");

    options.add_options()
        ("i,input", "Input obj file", cxxopts::value<std::string>())
        ("o,output", "Output directory", cxxopts::value<std::string>())
        ;

    auto parsed = options.parse(argc, argv);

    std::filesystem::path input = parsed["i"].as<std::string>();
    std::filesystem::path output = parsed["o"].as<std::string>();

    auto workdir = output / "work";
    create_directory(workdir);

    auto plainfile = workdir / "plain";

    obj_to_plain(input, plainfile, workdir);

    auto cellsdir = workdir / "cells";
    create_directory(cellsdir);

    gridify(plainfile, cellsdir);

    std::vector<Patch> all_patches;
    for (const auto& entry : std::filesystem::directory_iterator{cellsdir})
    {
        auto[patches, ids] = cluster(entry.path());
        break;
    }





    return 0;
}
