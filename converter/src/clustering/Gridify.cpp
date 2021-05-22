#include "Gridify.hpp"

#include "../DataTypes.hpp"

#include <fstream>
#include <string>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <unordered_map>


void gridify(const std::filesystem::path& input_file, const std::filesystem::path& output_directory)
{
	if (!exists(input_file)
		|| !is_regular_file(input_file)
		|| !is_directory(output_directory))
	{
        throw std::invalid_argument("Specified files and/or directories are not valid.");
	}

	std::ifstream in{input_file, std::ios_base::binary};


	std::size_t size = file_size(input_file);

	Dimensions dims;
	in.read(reinterpret_cast<char*>(&dims), sizeof(dims));

	// Dumb heuristics. 2MB is the limit for 1 in-core merge for now
	// TODO: improve
	auto part_count = std::ceil(std::cbrt(std::max<std::size_t>(size / (2 * 1024 * 1024), 1)));
	auto grid_size = MeanDimension(dims) / part_count;

	using IntegerNumber = int64_t;
	using BucketKey = std::tuple<IntegerNumber, IntegerNumber, IntegerNumber>;

    std::unordered_map<BucketKey, std::ofstream> streams;
    std::ofstream border_stream{output_directory / BORDER_FILENAME, std::ios_base::binary};


	ThickTriangle current;
	while (in.read(reinterpret_cast<char*>(&current), sizeof(current)))
	{
	    Matrix3 tri_vertices;
	    tri_vertices << current.a.x, current.b.x, current.c.x,
                        current.a.y, current.b.y, current.c.y,
                        current.a.z, current.b.z, current.c.z;

	    tri_vertices /= grid_size;

	    Eigen::Matrix<IntegerNumber, 3, 3> rounded = tri_vertices.cast<IntegerNumber>();

        std::ostream* target;

        if (rounded.col(0) == rounded.col(1) && rounded.col(1) == rounded.col(2))
        {
            IntegerNumber x = rounded(0, 0);
            IntegerNumber y = rounded(1, 0);
            IntegerNumber z = rounded(2, 0);
            if (auto it = streams.find({x, y, z}); it != streams.end())
            {
                target = &(it->second);
            }
            else
            {
                auto path = output_directory / (std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z));
                target = &(streams.emplace(std::tuple{x, y, z},
                    std::ofstream{path, std::ios_base::binary}).first->second);
            }
        }
        else
        {
            target = &border_stream;
        }

		target->write(reinterpret_cast<char*>(&current), sizeof(current));
	}
}

