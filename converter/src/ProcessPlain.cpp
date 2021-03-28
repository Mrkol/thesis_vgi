#include "ProcessPlain.hpp"

#include <fstream>

#include "clustering/Gridify.hpp"
#include "clustering/InCoreClustering.hpp"
#include "clustering/OutOfCoreClustering.hpp"
#include "clustering/Quadrangulation.hpp"
#include "Parametrization.hpp"
#include "ScopedTimer.hpp"
#include "StaticThreadPool.hpp"


void debug_cluster_output(const std::filesystem::path& folder, const std::filesystem::path& output);
void debug_parametrization_output(const std::filesystem::path& quad_folder,
    const std::filesystem::path& quad_info_folder, const std::filesystem::path& output);

void process_plain(const std::filesystem::path& plainfile, const std::filesystem::path& workdir,
    ClusteringMetricConfig metric_config, std::size_t memory_limit, FloatingNumber error_threshold,
    ParametrizationConfig parametrization_config)
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
            pool.submit([&datas, &cells, metric_config, error_threshold, i,
                memory_limit = memory_limit / cells.size()]
                ()
                {
                    if (cells[i].filename() != BORDER_FILENAME)
                    {
                        datas[i] = incore_cluster(cells[i],
                            metric_config, memory_limit, error_threshold / std::sqrt(cells.size()), 0.3);
                    }
                    else
                    {
                        datas[i] = triangle_soup_to_clusters(read_plainfile(cells[i]));
                    }
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

    debug_cluster_output(clusters_path, workdir / "clustered");

    auto quad_info_path = workdir / "cluster_info";

    create_directory(quad_info_path);

    {
        ScopedTimer timer("quadrangulation");
        StaticThreadPool pool;
        for (const auto& entry : std::filesystem::directory_iterator{clusters_path})
        {
            pool.submit(
                [cell = entry.path(), &total_clustering_data, &quad_info_path]
                    ()
                {
                    quadrangulate(cell, quad_info_path, std::stoull(cell.filename()), total_clustering_data);
                });
        }
    }

    debug_cluster_output(clusters_path, workdir / "quadrangulated");

    {
        ScopedTimer timer("parametrization");
        StaticThreadPool pool;
        for (const auto& entry : std::filesystem::directory_iterator{clusters_path})
        {
            pool.submit(
                [cell = entry.path(), &quad_info_path, &parametrization_config]()
                {
                    parametrize(cell, quad_info_path, parametrization_config);
                });
        }
    }

    debug_parametrization_output(clusters_path, quad_info_path, workdir / "parametrized");
}

void debug_cluster_output(const std::filesystem::path& folder, const std::filesystem::path& output)
{
    ScopedTimer timer("debug cluster output");

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

void debug_parametrization_output(const std::filesystem::path& quad_folder,
    const std::filesystem::path& quad_info_folder, const std::filesystem::path& output)
{
    ScopedTimer timer("debug cluster output");

    std::ofstream clustered{output, std::ios_base::binary};

    std::size_t fp_size = sizeof(FloatingNumber);
    static_assert(sizeof(fp_size) == 8);
    clustered.write(reinterpret_cast<char*>(&fp_size), sizeof(fp_size));


    std::size_t patch_idx = 0;
    for (const auto& entry : std::filesystem::directory_iterator{quad_folder})
    {
        std::unordered_map<HashableCoords, std::tuple<FloatingNumber, FloatingNumber>> mapping;
        {
            auto info_path = quad_info_folder / entry.path().filename();

            using MappingElement = std::pair<HashableCoords, std::tuple<FloatingNumber, FloatingNumber>>;
            std::vector<MappingElement> elements;
            elements.resize(file_size(info_path) / sizeof(MappingElement));

            std::ifstream info{info_path};
            info.read(reinterpret_cast<char*>(elements.data()), file_size(info_path));

            for (auto&[key, value] : elements)
            {
                mapping.emplace(key, value);
            }
        }



        std::ifstream plain{entry.path(), std::ios_base::binary};

        ThickTriangle tri{};
        while (plain.read(reinterpret_cast<char*>(&tri), sizeof(tri)))
        {
            auto[a, b, c] = triangle_verts(tri);
            auto m_a = mapping[a];
            auto m_b = mapping[b];
            auto m_c = mapping[c];
            tri.a.u = std::get<0>(m_a);
            tri.a.v = std::get<1>(m_a);

            tri.b.u = std::get<0>(m_b);
            tri.b.v = std::get<1>(m_b);

            tri.c.u = std::get<0>(m_c);
            tri.c.v = std::get<1>(m_c);

            tri.a.w = tri.b.w = tri.c.w = 0;
            clustered.write(reinterpret_cast<char*>(&tri), sizeof(tri));
        }

        ++patch_idx;
    }
}
