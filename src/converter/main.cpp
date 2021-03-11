#include <cxxopts.hpp>
#include <fstream>

#include "common/StaticThreadPool.hpp"
#include "common/ScopedTimer.hpp"

#include "ToPlainConverters.hpp"
#include "clustering/ClusterPlain.hpp"


int main(int argc, char** argv)
{
    cxxopts::Options options("VGI Converter", "Converts .obj files to VGI format");

    options.add_options()
        ("i,input", "Input obj file", cxxopts::value<std::string>())
        ("o,output", "Output directory", cxxopts::value<std::string>())
        ("clustering-error-threshold", "Error threshold for stopping clustering",
            cxxopts::value<FloatingNumber>()->default_value("150"))
        ("clustering-planarity-weight", "Weight for planarity error in the clustering metric",
            cxxopts::value<FloatingNumber>()->default_value("1"))
        ("clustering-orientation-weight", "Weight for orientation error in the clustering metric",
            cxxopts::value<FloatingNumber>()->default_value("0"))
        ("clustering-compactness-weight", "Weight for compactness error in the clustering metric",
            cxxopts::value<FloatingNumber>()->default_value("1"))
        ("clustering-max-memory", "Memory limit for clustering (in bytes)",
            cxxopts::value<std::size_t>()->default_value("104857600"))
        ("h,help", "Print usage")
        ;

    auto parsed = options.parse(argc, argv);


    if (parsed.count("help"))
    {
        std::cout << options.help() << std::endl;
        return 0;
    }

    std::filesystem::path input = parsed["i"].as<std::string>();
    std::filesystem::path output = parsed["o"].as<std::string>();
    FloatingNumber clustering_error_threshold = parsed["clustering-error-threshold"].as<FloatingNumber>();
    ClusteringMetricConfig clustering_metric_config
    {
        parsed["clustering-planarity-weight"].as<FloatingNumber>(),
        parsed["clustering-orientation-weight"].as<FloatingNumber>(),
        parsed["clustering-compactness-weight"].as<FloatingNumber>()
    };
    std::size_t memory_limit = parsed["clustering-max-memory"].as<std::size_t>();


    auto workdir = output / "work";
    create_directory(workdir);

    auto plainfile = workdir / "plain";

    {
        ScopedTimer timer{"plainification"};
        obj_to_plain(input, plainfile, workdir);
    }

    cluster_plain(plainfile, workdir, clustering_metric_config, memory_limit, clustering_error_threshold);

    return 0;
}
