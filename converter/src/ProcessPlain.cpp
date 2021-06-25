#include "ProcessPlain.hpp"

#include <fstream>


#include "FileStreamPool.hpp"
#include "clustering/Gridify.hpp"
#include "clustering/InCoreClustering.hpp"
#include "clustering/OutOfCoreClustering.hpp"
#include "clustering/Quadrangulation.hpp"
#include "Parametrization.hpp"
#include "ScopedTimer.hpp"
#include "StaticThreadPool.hpp"
#include "clustering/EdgeStraightening.hpp"


void debug_cluster_output(const std::filesystem::path& folder, const std::filesystem::path& output);
void debug_parametrization_output(const std::filesystem::path& quad_folder,
    const std::filesystem::path& quad_info_folder, const std::filesystem::path& output);
void debug_convert_resampled(const std::filesystem::path& resampled_dir, const std::filesystem::path& output_dir,
    const ResamplerConfig& config);

void process_plain(const std::filesystem::path& plainfile, const std::filesystem::path& workdir,
    const std::filesystem::path& output_dir, bool debug_output,
    ClusteringMetricConfig metric_config, std::size_t memory_limit, FloatingNumber error_threshold,
    ParametrizationConfig parametrization_config, ResamplerConfig resampler_config)
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
                    auto triangles = read_plainfile(cells[i]);
                    if (cells[i].filename() == BORDER_FILENAME)
                    {
                        triangles = fixup_border(std::move(triangles));
                        datas[i] = triangle_soup_to_clusters(triangles);
                        write_plainfile(cells[i], triangles);
                        return;
                    }

                    datas[i] = incore_cluster(triangles,
                        metric_config, memory_limit, error_threshold / std::sqrt(cells.size()), 0.3);
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
        remove_all(cellsdir);
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

        FileStreamPool<std::ofstream> file_pool{500};

        ThickTriangle triangle;
        std::size_t idx = 0;
        while (plain.read(reinterpret_cast<char*>(&triangle), sizeof(triangle)))
        {
            auto i = total_clustering_data.accumulated_mapping[idx];
            file_pool
                .get(clusters_path / std::to_string(i), std::ios_base::binary | std::ios_base::app)
                .write(reinterpret_cast<char*>(&triangle), sizeof(triangle));
            ++idx;
        }
    }

    if (debug_output)
    {
        debug_cluster_output(clusters_path, workdir / "clustered");
    }

    {
        ScopedTimer timer("edge straightening");

        std::vector<bool> used_flags(total_clustering_data.patches.size());

        std::mutex mtx;
        std::condition_variable pairs_available;
        std::unordered_set<SymmetricPair<std::size_t>> adjacent_pairs; // guarded by mtx
        for (std::size_t idx = 0; idx < used_flags.size(); ++idx)
        {
            for (auto& edge : total_clustering_data.patches[idx].boundary)
            {
                if (edge.patch_idx != Patch::NONE)
                {
                    adjacent_pairs.insert({idx, edge.patch_idx});
                }
            }
        }

        StaticThreadPool pool;
        for (std::size_t i = 0; i < adjacent_pairs.size(); ++i)
        {
            pool.submit(
                [&total_clustering_data, &clusters_path,
                    &used_flags, &mtx, &pairs_available, &adjacent_pairs]()
                {
                    std::unique_lock lock{mtx};

                    // TODO: Refactor, this is pretty shite
                    auto it = adjacent_pairs.begin();

                    auto locked = [&used_flags](const auto& pair)
                        { return used_flags[pair.first] || used_flags[pair.second]; };

                    while ((it = std::find_if_not(adjacent_pairs.begin(), adjacent_pairs.end(), locked))
                        == adjacent_pairs.end())
                    {
                        pairs_available.wait(lock);
                    }
                    std::size_t i = it->first;
                    std::size_t j = it->second;
                    adjacent_pairs.erase(it);

                    used_flags[i] = used_flags[j] = true;
                    lock.unlock();

                    auto first_path = clusters_path / std::to_string(i);
                    auto second_path = clusters_path / std::to_string(j);

                    auto first = read_plainfile(first_path);
                    auto second = read_plainfile(second_path);
                    straighten_edges({first, i}, {second, j}, total_clustering_data);
                    write_plainfile(first_path, first);
                    write_plainfile(second_path, second);

                    lock.lock();
                    used_flags[i] = used_flags[j] = false;
                    pairs_available.notify_all();
                });
        }
    }

    if (debug_output)
    {
        debug_cluster_output(clusters_path, workdir / "straightened");
    }

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
                    auto quads = quadrangulate(read_plainfile(cell),
                        std::stoull(cell.filename()), total_clustering_data);
                    remove(cell);
                    for (std::size_t i = 0; i < quads.size(); ++i)
                    {
                        auto& quad = quads[i];
                        std::string name = cell.filename().string() + "," + std::to_string(i);
                        write_plainfile(cell.parent_path() / name, quad.triangles);
                        {
                            std::ofstream info_file{quad_info_path / name, std::ios_base::binary};
                            for (auto& edge : quad.boundary)
                            {
                                write_range(info_file, edge.begin(), edge.end());
                            }
                        }
                    }
                });
        }
    }

    if (debug_output)
    {
        debug_cluster_output(clusters_path, workdir / "quadrangulated");
    }

    auto read_quad = [](const std::filesystem::path& quad, const std::filesystem::path& info)
        {
            QuadPatch result;
            result.triangles = read_plainfile(quad);
            {
                std::ifstream in{info, std::ios_base::binary};
                for (auto& edge : result.boundary)
                {
                    edge = read_vector<HashableCoords>(in);
                }
            }
            return result;
        };

    auto parametrization_dir = workdir / "parametrization";

    create_directory(parametrization_dir);

    {
        ScopedTimer timer("parametrization");
        StaticThreadPool pool;
        for (const auto& entry : std::filesystem::directory_iterator{clusters_path})
        {
            pool.submit(
                [quad_path = entry.path(), &quad_info_path, &parametrization_config, &read_quad, &parametrization_dir]
                ()
                {
                    auto quad = read_quad(quad_path, quad_info_path / quad_path.filename());
                    auto result = parametrize(quad, parametrization_config);

                    std::ofstream out{parametrization_dir / quad_path.filename(), std::ios_base::binary};
                    write_range(out, result.begin(), result.end());
                });
        }
    }

    if (debug_output)
    {
        debug_parametrization_output(clusters_path, parametrization_dir, workdir / "parametrized");
    }

    auto resampled_dir = output_dir / "resampled";
    create_directory(resampled_dir);

    {
        ScopedTimer timer("resampling");

        if (resampler_config.thread_count == 0)
        {
            resampler_config.thread_count = std::thread::hardware_concurrency();
        }

        // TODO: replace with proper downsampling

        std::size_t max_mip = resampler_config.log_resolution;
        for (std::size_t mip_level = 3; mip_level < max_mip; ++mip_level)
        {
            resampler_config.log_resolution = mip_level;

            Resampler resampler{resampler_config};
            StaticThreadPool pool{resampler_config.thread_count};

            for (const auto& entry : std::filesystem::directory_iterator{clusters_path})
            {
                pool.submit(
                    [quad_path = entry.path(), &quad_info_path, &resampled_dir,
                        &resampler, &read_quad, &parametrization_dir, mip_level]
                        ()
                    {
                        auto quad = read_quad(quad_path, quad_info_path / quad_path.filename());
                        std::vector<MappingElement> mapping;
                        {
                            std::ifstream in{parametrization_dir / quad_path.filename(), std::ios_base::binary};
                            mapping = read_vector<MappingElement>(in);
                        }


                        auto result = resampler.resample(quad, mapping, StaticThreadPool::current_thread_index());

                        std::string filename = quad_path.filename().string() + "," + std::to_string(mip_level);

                        std::ofstream out{resampled_dir / filename, std::ios_base::binary};

                        out.write(reinterpret_cast<const char*>(result.data()),
                            static_cast<std::streamsize>(result.size() * sizeof(result[0])));
                    });
            }
        }
    }

    if (debug_output)
    {
        auto debug_images_dir = workdir / "images";
        create_directory(debug_images_dir);
        debug_convert_resampled(resampled_dir, debug_images_dir, resampler_config);
    }
    else
    {
        remove_all(workdir);
    }
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
            clustered.write(reinterpret_cast<char*>(&tri), sizeof(tri));
        }

        ++patch_idx;
    }
}

void debug_parametrization_output(const std::filesystem::path& quad_folder,
    const std::filesystem::path& parametrization_folder, const std::filesystem::path& output)
{
    ScopedTimer timer("debug cluster output");

    std::ofstream clustered{output, std::ios_base::binary};

    std::size_t fp_size = sizeof(FloatingNumber);
    static_assert(sizeof(fp_size) == 8);
    clustered.write(reinterpret_cast<char*>(&fp_size), sizeof(fp_size));


    std::size_t patch_idx = 0;
    for (const auto& entry : std::filesystem::directory_iterator{quad_folder})
    {
        std::unordered_map<HashableCoords, MappingCoords> mapping;
        {
            auto info_path = parametrization_folder / entry.path().filename();

            std::vector<MappingElement> elements;

            {
                std::ifstream info{info_path, std::ios_base::binary};
                elements = read_vector<MappingElement>(info);
            }

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
            auto m_a = mapping.at(a);
            auto m_b = mapping.at(b);
            auto m_c = mapping.at(c);
            tri.a.u = std::get<0>(m_a);
            tri.a.v = std::get<1>(m_a);

            tri.b.u = std::get<0>(m_b);
            tri.b.v = std::get<1>(m_b);

            tri.c.u = std::get<0>(m_c);
            tri.c.v = std::get<1>(m_c);

            clustered.write(reinterpret_cast<char*>(&tri), sizeof(tri));
        }

        ++patch_idx;
    }
}

void debug_convert_resampled(const std::filesystem::path& resampled_dir, const std::filesystem::path& output_dir,
    const ResamplerConfig& config)
{
    ScopedTimer timer("debug resampled output");

    for (const auto& entry : std::filesystem::directory_iterator(resampled_dir))
    {
        std::vector<std::array<float, 8>> data{file_size(entry.path()) / sizeof(data[0])};
        {
            std::ifstream input{entry.path(), std::ios_base::binary};
            input.read(reinterpret_cast<char*>(data.data()),
                static_cast<std::streamsize>(data.size() * sizeof(data[0])));
        }

        std::array<float, 3> min{data.front()[0], data.front()[1], data.front()[2]};
        std::array<float, 3> max = min;
        for (const auto& pixel : data)
        {
            for (std::size_t i = 0; i < 3; ++i)
            {
                min[i] = std::min(min[i], pixel[i]);
                max[i] = std::max(max[i], pixel[i]);
            }
        }

        std::vector<std::array<uint8_t, 3>> converted{data.size()};
        for (std::size_t i = 0; i < data.size(); ++i)
        {
            for (std::size_t j = 0; j < 3; ++j)
            {
                converted[i][j] = uint8_t(255 * (data[i][j] - min[j]) / (max[j] - min[j]));
            }
        }

        auto res = std::size_t(std::sqrt(data.size()));

        std::ofstream output
            {output_dir / (entry.path().filename().string() + ".ppm"), std::ios_base::binary};
        output << "P6\n" << res << "\n" << res << "\n" << 255 << "\n";
        output.write(reinterpret_cast<const char*>(converted.data()),
            static_cast<std::streamsize>(converted.size() * sizeof(converted[0])));
    }
}
