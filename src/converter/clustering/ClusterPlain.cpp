#include "ClusterPlain.hpp"

#include <fstream>

#include "Gridify.hpp"
#include "InCoreClustering.hpp"
#include "OutOfCoreClustering.hpp"
#include "Quadrangulate.hpp"
#include "common/ScopedTimer.hpp"
#include "common/StaticThreadPool.hpp"


void debug_output(const std::filesystem::path& folder, const std::filesystem::path& output);

void cluster_plain(const std::filesystem::path& plainfile, const std::filesystem::path& workdir,
    ClusteringMetricConfig metric_config, std::size_t memory_limit, FloatingNumber error_threshold)
{
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

    {
        ScopedTimer timer("incore clustering");
        StaticThreadPool pool;
        for (std::size_t i = 0; i < cells.size(); ++i)
        {
            pool.submit([&datas, &cells, memory_limit = memory_limit / cells.size(), i, metric_config, error_threshold]
                ()
                {
                    datas[i] = incore_cluster(cells[i], metric_config, memory_limit, error_threshold, 0.3);
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
            {
                std::ifstream in{cell, std::ios_base::binary};
                plain_rewrite << in.rdbuf();
            }
        }
        std::filesystem::remove_all(cellsdir);
    }

    ClusteringData total_clustering_data;
    {
        ScopedTimer timer("out of core clustering");
        total_clustering_data  = outofcore_cluster(std::move(datas), metric_config, error_threshold);
    }

    auto clusters_path = workdir / "clusters";
    create_directory(clusters_path);

    {
        std::ifstream plain{plainfile, std::ios_base::binary};

        std::vector<std::ofstream> cluster_files;
        cluster_files.reserve(total_clustering_data.patches.size());
        for (std::size_t i = 0; i < total_clustering_data.patches.size(); ++i)
        {
            cluster_files.emplace_back(clusters_path / std::to_string(i), std::ios_base::binary);
        }

        ThickTriangle triangle;
        std::size_t idx = 0;
        while (plain.read(reinterpret_cast<char*>(&triangle), sizeof(triangle)))
        {
            cluster_files[total_clustering_data.accumulated_mapping[idx]]
                .write(reinterpret_cast<char*>(&triangle), sizeof(triangle));
            ++idx;
        }
    }

    debug_output(clusters_path, workdir / "clustered");

    {
        ScopedTimer timer("quadrangulation");
        StaticThreadPool pool;
        for (const auto& entry : std::filesystem::directory_iterator{clusters_path})
        {
            pool.submit(
                [cell = entry.path(), &total_clustering_data]
                ()
                {
                    try
                    {
                        quadrangulate(cell, std::stoull(cell.filename()), total_clustering_data);
                    }
                    catch (...)
                    {

                    }
                });
        }
    }

    debug_output(clusters_path, workdir / "quadrangulated");
}

void debug_output(const std::filesystem::path& folder, const std::filesystem::path& output)
{
    ScopedTimer timer("debug output");

    std::ofstream clustered{output, std::ios_base::binary};

    std::size_t fp_size = sizeof(FloatingNumber);
    static_assert(sizeof(fp_size) == 8);
    clustered.write(reinterpret_cast<char*>(&fp_size), sizeof(fp_size));


    std::size_t patch_idx = 0;
    std::size_t patch_count =
        std::distance(std::filesystem::directory_iterator{folder}, std::filesystem::directory_iterator{});
    for (const auto& entry : std::filesystem::directory_iterator{folder})
    {
        std::ifstream plain{entry.path(), std::ios_base::binary};

        ThickTriangle tri{};
        while (plain.read(reinterpret_cast<char*>(&tri), sizeof(tri)))
        {
            tri.a.u = tri.b.u = tri.c.u = static_cast<FloatingNumber>(patch_idx);
            tri.a.v = tri.b.v = tri.c.v = static_cast<FloatingNumber>(patch_count);
            tri.a.w = tri.b.w = tri.c.w = 0;
            clustered.write(reinterpret_cast<char*>(&tri), sizeof(tri));
        }

        ++patch_idx;
    }
}
