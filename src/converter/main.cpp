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
        ("clustering-error-threshold", "Error threshold for stopping clustering",
            cxxopts::value<FloatingNumber>()->default_value("150"))
        ("clustering-planarity-weight", "Weight for planarity error in the clustering metric",
            cxxopts::value<FloatingNumber>()->default_value("1"))
        ("clustering-orientation-weight", "Weight for orientation error in the clustering metric",
            cxxopts::value<FloatingNumber>()->default_value("0"))
        ("clustering-compactness-weight", "Weight for compactness error in the clustering metric",
            cxxopts::value<FloatingNumber>()->default_value("1"))
        ("clustering-max-memory", "Memory limit for clustering (in bytes)",
            cxxopts::value<std::size_t>()->default_value("104857600"))
        ("h,help", "Print usage")
        ;

    auto parsed = options.parse(argc, argv);


    if (parsed.count("help"))
    {
        std::cout << options.help() << std::endl;
        return 0;
    }

    std::filesystem::path input = parsed["i"].as<std::string>();
    std::filesystem::path output = parsed["o"].as<std::string>();
    FloatingNumber clustering_error_threshold = parsed["clustering-error-threshold"].as<FloatingNumber>();
    ClusteringMetricConfig clustering_metric_config
    {
        parsed["clustering-planarity-weight"].as<FloatingNumber>(),
        parsed["clustering-orientation-weight"].as<FloatingNumber>(),
        parsed["clustering-compactness-weight"].as<FloatingNumber>()
    };

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

    std::size_t memory_limit = parsed["clustering-max-memory"].as<std::size_t>() / cells.size();
    {
        ScopedTimer timer("incore clustering");
        StaticThreadPool pool;
        for (std::size_t i = 0; i < cells.size(); ++i)
        {
            pool.submit([&datas, &cells, memory_limit, i, clustering_metric_config, clustering_error_threshold]()
                {
                    datas[i] = incore_cluster(cells[i],
                        clustering_metric_config, memory_limit, clustering_error_threshold, 0.3);
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
        result_data  = outofcore_cluster(std::move(datas), clustering_metric_config, clustering_error_threshold);
    }

    // TODO: remove, temporary debug thing.
    {
        ScopedTimer timer("debug output");

        std::ifstream plain{plainfile, std::ios_base::binary};
        std::ofstream clustered{workdir / "clustered", std::ios_base::binary};

        std::size_t fp_size = sizeof(FloatingNumber);

        static_assert(sizeof(fp_size) == 8);
        clustered.write(reinterpret_cast<char*>(&fp_size), sizeof(fp_size));

        ThickTriangle tri{};
        std::size_t idx = 0;
        while (plain.read(reinterpret_cast<char*>(&tri), sizeof(tri)))
        {
            auto cluster_id = result_data.accumulated_mapping[idx];
            tri.a.u = tri.b.u = tri.c.u = static_cast<FloatingNumber>(cluster_id);
            tri.a.v = tri.b.v = tri.c.v = static_cast<FloatingNumber>(result_data.patches.size());
            tri.a.w = tri.b.w = tri.c.w = 0;
            clustered.write(reinterpret_cast<char*>(&tri), sizeof(tri));
            ++idx;
        }
    }

    return 0;
}
