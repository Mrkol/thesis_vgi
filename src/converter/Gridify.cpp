#include "Gridify.hpp"

#include "DataTypes.hpp"
#include "TupleHash.hpp"

#include <fstream>
#include <string>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <unordered_map>

void gridify(std::filesystem::path input_file, std::filesystem::path output_directory)
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

	// Dumb heuristics. 1MB is the limit for 1 in-core merge for now
	// TODO: improve
	auto part_count = std::ceil(std::cbrt(std::max<std::size_t>(size / (1024 * 1024), 1)));
	auto grid_size = MeanDimension(dims) / part_count;

	using BucketKey = std::tuple<int64_t, int64_t, int64_t>;

    std::unordered_map<BucketKey, std::ofstream> streams;

	auto stream_for_tri =
		[&output_directory, &streams]
		(int64_t x, int64_t y, int64_t z) -> std::ofstream&
		{
			if (auto it = streams.find({x, y, z}); it != streams.end())
			{
				return it->second;
			}

			return streams.emplace(std::tuple{x, y, z},
				std::ofstream{output_directory / (std::to_string(x) + ","
				+ std::to_string(y) + ","
				+ std::to_string(z)), std::ios_base::binary}).first->second;
		};

	ThickTriangle current;
	while (in.read(reinterpret_cast<char*>(&current), sizeof(current)))
	{
		auto& stream = stream_for_tri(
            int64_t(current.a.x / grid_size),
            int64_t(current.a.y / grid_size),
            int64_t(current.a.z / grid_size));

		stream.write(reinterpret_cast<char*>(&current), sizeof(current));
	}
}

