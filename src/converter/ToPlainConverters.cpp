#include "ToPlainConverters.hpp"

#include <fstream>
#include <sstream>
#include <string>

#include "ObjParsingHelpers.hpp"


void obj_to_plain(std::filesystem::path obj_file, std::filesystem::path output_file, std::filesystem::path workdir)
{
    
    if (!exists(obj_file)
        || !exists(output_file)
        || !exists(workdir)
        || !is_regular_file(obj_file)
        || !is_regular_file(output_file)
        || !is_directory(workdir))
    {
        throw std::invalid_argument("Specified files and/or directories are not valid.");
    }

    const auto vert_filename = workdir / "vertices";
    const auto norm_filename = workdir / "normals";
    const auto uv_filename = workdir / "uvs";

    {
        std::ifstream in{obj_file};
        
        std::ofstream vert_file{vert_filename};
        std::ofstream norm_file{norm_filename};
        std::ofstream uv_file{uv_filename};


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
                auto opt = try_get_triple(rest);
                if (!opt.has_value())
                {
                    throw std::domain_error("Failed to parse v/vn/vt tag!");
                }

                static_assert(sizeof(opt.value()) == 12);
                target->write(reinterpret_cast<char*>(&opt.value()), 12);
            }
        }
    }

    
    {
        std::ifstream vert_file{vert_filename};
        std::ifstream norm_file{norm_filename};
        std::ifstream uv_file{uv_filename};
        
        std::ifstream in{obj_file};
        
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
            

        }
    }
}
