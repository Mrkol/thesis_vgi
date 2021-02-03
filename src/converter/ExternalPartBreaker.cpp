#include "ExternalPartBreaker.hpp"

#include "ObjParsingHelpers.hpp"

#include <fstream>
#include <string>
#include <cassert>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/SVD>

namespace ExternalPartBreaker
{

std::vector<std::tuple<float, float, float>> load_obj_vertices(std::filesystem::path input_file)
{
    std::ifstream in{input_file};

    std::vector<std::tuple<float, float, float>> vertices;

    std::string line;
    while (std::getline(in, line))
    {
        auto maybe_vert = try_get_vertex(line);

        if (!maybe_vert.has_value())
        {
            continue;
        }

        vertices.push_back(maybe_vert.value());
    }

    return vertices;
}

Eigen::MatrixXf calculate_pca_matrix(std::vector<std::tuple<float, float, float>> vertices)
{
    Eigen::MatrixXf Data{3, vertices.size()};
    for (size_t i = 0; i < vertices.size(); ++i)
    {
        auto[x, y, z] = vertices[i];
        Data.block<3, 1>(0, i) << x, y, z;
    }

    auto SVD = Data.bdcSvd(Eigen::DecompositionOptions::ComputeFullU);
    auto U = SVD.matrixU();
    assert(U.size() == 9);
    U.transposeInPlace();

    return U;
}

std::vector<Result> break_into_parts(Arguments args)
{
    if (!exists(args.input_file)
        || !exists(args.output_directory)
        || !is_regular_file(args.input_file)
        || !is_directory(args.output_directory))
    {
        throw std::invalid_argument("Specified files and/or directories are not valid.");
    }

    auto vertices = load_obj_vertices(args.input_file);

    auto M = calculate_pca_matrix(vertices);


    
    
    return {};
}

}

