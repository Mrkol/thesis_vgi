#include <cxxopts.hpp>
#include <fstream>

#include "common/StaticThreadPool.hpp"
#include "common/ScopedTimer.hpp"

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

    {
        ScopedTimer timer{"plainification"};
        obj_to_plain(input, plainfile, workdir);
    }

    auto cellsdir = workdir / "cells";
    create_directory(cellsdir);

    {
        ScopedTimer timer{"gridify"};
        gridify(plainfile, cellsdir);
    }

    std::vector<std::filesystem::path> cells;
    // order is indeterminate for directory_iterator, so we need to save the paths
    for (const auto& entry : std::filesystem::directory_iterator{cellsdir})
    {
        cells.push_back(entry.path());
    }

    std::vector<ClusteringData> datas;
    datas.resize(cells.size());

    std::size_t memory_limit = 100 * 1024 * 1024 / cells.size();
    {
        ScopedTimer timer("incore clustering");
        StaticThreadPool pool;
        for (std::size_t i = 0; i < cells.size(); ++i)
        {
            pool.submit([&datas, &cells, memory_limit, i]()
                {
                    datas[i] = incore_cluster(cells[i], memory_limit);
                });
        }
    }

    // After this the mapping of outofcore cluster is correct
    {
        ScopedTimer timer("cell concatenation");
        std::filesystem::remove(plainfile);
        std::ofstream plain_rewrite{plainfile, std::ios_base::app | std::ios_base::binary};
        for (const auto& cell : cells)
        {
            std::ifstream in{cell, std::ios_base::binary};
            plain_rewrite << in.rdbuf();
        }
    }

    ClusteringData result_data;
    {
        ScopedTimer timer("out of core clustering");
        result_data  = outofcore_cluster(std::move(datas));
    }

    // TODO: remove, temporary debug thing.
    {
        ScopedTimer timer("debug output");

        std::ifstream plain{plainfile, std::ios_base::binary};
        std::ofstream clustered{workdir / "clustered", std::ios_base::binary};

        ThickTriangle tri{};
        std::size_t idx = 0;
        while (plain.read(reinterpret_cast<char*>(&tri), sizeof(tri)))
        {
            auto cluster_id = result_data.accumulated_mapping[idx];
            tri.a.u = tri.b.u = tri.c.u = static_cast<float>(cluster_id);
            tri.a.v = tri.b.v = tri.c.v = static_cast<float>(result_data.patches.size());
            tri.a.w = tri.b.w = tri.c.w = 0;
            clustered.write(reinterpret_cast<char*>(&tri), sizeof(tri));
            ++idx;
        }
    }

    return 0;
}
