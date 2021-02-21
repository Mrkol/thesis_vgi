#include <cxxopts.hpp>

#include "ToPlainConverters.hpp"
#include "Gridify.hpp"
#include "InCoreClustering.hpp"
#include "OutOfCoreClustering.hpp"


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

    std::vector<ClusteringData> datas;
    // TODO: threadpool for running this in parallel
    for (const auto& entry : std::filesystem::directory_iterator{cellsdir})
    {
        datas.push_back(incore_cluster(entry.path(), 1024*10));
    }

    auto[patches, graph_vertices] = outofcore_cluster(datas);


    return 0;
}
