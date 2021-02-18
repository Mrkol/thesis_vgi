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

	std::ifstream in{input_file};


	auto size = std::filesystem::file_size(input_file);

	Dimensions dims;
	in.read(reinterpret_cast<char*>(&dims), sizeof(dims));

	auto part_count = std::max(size / 4096, 1ul);
	auto grid_size = MeanDimension(dims) / part_count;

	using BucketKey = std::tuple<int, int, int>;

	std::unordered_map<BucketKey, std::ofstream> streams;

	auto StreamForTri =
		[&streams, &output_directory]
		(int x, int y, int z) -> std::ofstream&
		{
			if (auto it = streams.find({x, y, z}); it != streams.end())
			{
				return it->second;
			}


			return streams.emplace(std::tuple{x, y, z},
				std::ofstream{output_directory / (std::to_string(x) + ","
				+ std::to_string(y) + ","
				+ std::to_string(z))}).first->second;
		};



	ThickTriangle current;
	while (in.read(reinterpret_cast<char*>(&current), sizeof(current)))
	{
		auto& stream = StreamForTri(current.a.x / grid_size,
			current.a.y / grid_size,
			current.a.z / grid_size);

		stream.write(reinterpret_cast<char*>(&current), sizeof(current));
	}
}

