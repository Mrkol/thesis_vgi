#include <cxxopts.hpp>
#include <fstream>

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

    std::vector<std::filesystem::path> cells;
    // order is indeterminate for directory_iterator, so we need to save the paths
    for (const auto& entry : std::filesystem::directory_iterator{cellsdir})
    {
        cells.push_back(entry.path());
    }

    std::vector<ClusteringData> datas;
    datas.reserve(cells.size());
    // TODO: threadpool for running this in parallel
    for (const auto& cell : cells)
    {
        datas.push_back(incore_cluster(cell, 100*1024*1024 / cells.size()));
    }

    // After this the mapping of outofcore cluster is correct
    {
        std::filesystem::remove(plainfile);
        std::ofstream plain_rewrite{plainfile, std::ios_base::app};
        for (const auto& cell : cells)
        {
            std::ifstream in{cell};
            plain_rewrite << in.rdbuf();
        }
    }

    auto[patches, graph_vertices, mapping] = outofcore_cluster(datas);

    // TODO: remove, temporary debug thing.
    {
        std::ifstream plain{plainfile};
        std::ofstream clustered{workdir / "clustered"};

        ThickTriangle tri{};
        std::size_t idx = 0;
        while (plain.read(reinterpret_cast<char*>(&tri), sizeof(tri)))
        {
            auto cluster_id = mapping[idx];
            tri.a.u = tri.b.u = tri.c.u = cluster_id;
            tri.a.v = tri.b.v = tri.c.v = patches.size();
            tri.a.w = tri.b.w = tri.c.w = 0;
            clustered.write(reinterpret_cast<char*>(&tri), sizeof(tri));
            ++idx;
        }
    }

    return 0;
}
