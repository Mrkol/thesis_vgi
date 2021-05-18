#include "ToPlainConverters.hpp"

#include <sstream>
#include <array>
#include <string>

#include "ObjParsingHelpers.hpp"
#include "DataTypes.hpp"


void obj_to_plain(const std::filesystem::path& obj_file, const std::filesystem::path& output_file, const std::filesystem::path& workdir)
{
    if (!exists(obj_file)
        || exists(output_file)
        || !exists(workdir)
        || !is_regular_file(obj_file)
        || !is_directory(workdir))
    {
        throw std::invalid_argument("Specified files and/or directories are not valid.");
    }

    const auto vert_filename = workdir / "vertices";
    const auto norm_filename = workdir / "normals";
    const auto uv_filename = workdir / "uvs";

    Dimensions dims;

    {
        std::ifstream in{obj_file};

        std::ofstream vert_file{vert_filename, std::ios_base::binary};
        std::ofstream norm_file{norm_filename, std::ios_base::binary};
        std::ofstream uv_file{uv_filename, std::ios_base::binary};

        std::string line;
        while (std::getline(in, line))
        {
            auto[token, rest] = next_token(line);

            std::ofstream* target = nullptr;
            if (token == "v"sv)
            {
                target = &vert_file;
            }
            else if (token == "vn"sv)
            {
                target = &norm_file;
            }
            else if (token == "vt"sv)
            {
                target = &uv_file;
            }

            if (target != nullptr)
            {
                auto opt = try_get_triple<FloatingNumber>(rest);
                if (!opt.has_value())
                {
                    throw std::domain_error("Failed to parse v/vn/vt tag!");
                }

                if (target == &vert_file)
                {
                    auto[x, y, z] = opt.value();
                    dims.min_x = std::min(dims.min_x, x);
                    dims.max_x = std::max(dims.max_x, x);
                    dims.min_y = std::min(dims.min_y, y);
                    dims.max_y = std::max(dims.max_y, y);
                    dims.min_z = std::min(dims.min_z, z);
                    dims.max_z = std::max(dims.max_z, z);
                }

                static_assert(sizeof(opt.value()) == sizeof(FloatingNumber)*3);
                target->write(reinterpret_cast<char*>(&opt.value()), sizeof(opt.value()));
            }
        }
    }


    {
        // TODO: Replace with mmap
        std::ifstream vert_file{vert_filename, std::ios_base::binary};
        std::ifstream norm_file{norm_filename, std::ios_base::binary};
        std::ifstream uv_file{uv_filename, std::ios_base::binary};

        auto LookupInFile =
            [](std::ifstream& stream, size_t index)
            {
                stream.seekg(sizeof(HashableCoords) * index);
                HashableCoords result;
                stream.read(reinterpret_cast<char*>(&result), sizeof(HashableCoords));
                return result;
            };

        auto LookupVertData =
            [&LookupInFile, &vert_file, &norm_file, &uv_file](size_t vert_idx, size_t norm_idx, size_t uv_idx)
            {
                auto[x, y, z] = LookupInFile(vert_file, vert_idx - 1);
                auto[nx, ny, nz] = LookupInFile(norm_file, norm_idx - 1);
                auto[u, v, w] = LookupInFile(uv_file, uv_idx - 1);
                return ThickVertex{x, y, z, nx, ny, nz, u, v, w};
            };

        std::ofstream out{output_file, std::ios_base::binary};

        out.write(reinterpret_cast<char*>(&dims), sizeof(dims));

        std::ifstream in{obj_file, std::ios_base::binary};

        std::string line;
        while (std::getline(in, line))
        {
            auto[token, rest] = next_token(line);

            if (token != "f"sv)
            {
                continue;
            }

            auto[first, rest1] = next_token(rest);
            auto[second, rest2] = next_token(rest1);
            auto[third, rest3] = next_token(rest2);

            static constexpr char SLASH[] = " /";
            auto first_opt = try_get_triple<size_t, SLASH>(first);
            auto second_opt = try_get_triple<size_t, SLASH>(second);
            auto third_opt = try_get_triple<size_t, SLASH>(third);

            if (!first_opt.has_value()
                || !second_opt.has_value()
                || !third_opt.has_value())
            {
                throw std::domain_error("Failed to parse f tag!");
            }

            auto v1 = std::apply(LookupVertData, first_opt.value());
            auto v2 = std::apply(LookupVertData, second_opt.value());
            auto v3 = std::apply(LookupVertData, third_opt.value());

            ThickTriangle all_verts{v1, v2, v3};

            out.write(reinterpret_cast<char*>(&all_verts), sizeof(FloatingNumber)*9*3);
        }
    }
}
