#include "Gridify.hpp"

#include "ObjParsingHelpers.hpp"

#include <fstream>
#include <string>
#include <algorithm>
#include <iostream>
#include <numeric>


namespace Gridify
{

std::vector<Result> gridify(Arguments args)
{
    if (!exists(args.input_file)
        || !exists(args.output_directory)
        || !is_regular_file(args.input_file)
        || !is_directory(args.output_directory))
    {
        throw std::invalid_argument("Specified files and/or directories are not valid.");
    }
    
    
    float maxX = -std::numeric_limits<float>::max();
    float minX = std::numeric_limits<float>::max();
    float maxY = -std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float maxZ = -std::numeric_limits<float>::max();
    float minZ = std::numeric_limits<float>::max();

    size_t vertice_count = 0;

    {
        std::ifstream in{args.input_file};
        // First out of core iteration. Collect some statistics.
        std::string line;
        while (std::getline(in, line))
        {
            auto maybe_vert = try_get_vertex(line);

            if (!maybe_vert.has_value())
            {
                continue;
            }

            auto[x, y, z] = maybe_vert.value();
            ++vertice_count;
            
            maxX = std::max(maxX, x);
            minX = std::min(minX, x);
            maxY = std::max(maxY, y);
            minY = std::min(minY, y);
            maxZ = std::max(maxZ, z);
            minZ = std::min(minZ, z);
        }
    }

    // TODO: Play around with this
    float approx_wanted_block_count = vertice_count / 1e7f;
    float one_dim_count = std::cbrt(approx_wanted_block_count);
    float block_size = std::min((maxX - minX)/one_dim_count, std::min((maxY - minY)/one_dim_count, (maxZ - minZ)/one_dim_count));

    return {};
}

}

