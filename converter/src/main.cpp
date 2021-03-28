#include <cxxopts.hpp>
#include <fstream>

#include "StaticThreadPool.hpp"
#include "ScopedTimer.hpp"

#include "ToPlainConverters.hpp"
#include "ProcessPlain.hpp"


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
        ("clustering-irregularity-weight", "Weight for irregularity error in the clustering metric",
            cxxopts::value<FloatingNumber>()->default_value("0"))
        ("clustering-irregularity-difference-weight", "Weight for irregularity difference in the clustering metric",
            cxxopts::value<FloatingNumber>()->default_value("1.7"))
        ("clustering-max-memory", "Memory limit for clustering (in bytes)",
            cxxopts::value<std::size_t>()->default_value("104857600"))
        ("parametrization-uniform-springs-rate", "Learning rate for gradient descent in uniform springs",
            cxxopts::value<FloatingNumber>()->default_value("1e-2"))
        ("parametrization-uniform-gradient-threshold", "Stopping threshold for uniform springs",
            cxxopts::value<FloatingNumber>()->default_value("1e-4"))
        ("parametrization-l2-stretch-threshold", "Stopping threshold for L2 stretch optimizer",
            cxxopts::value<FloatingNumber>()->default_value("1e-6"))
        ("parametrization-l2-stretch-max-iterations", "Max iterations for the L2 stretch optimizer",
            cxxopts::value<std::size_t>()->default_value("100"))
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
        parsed["clustering-irregularity-difference-weight"].as<FloatingNumber>(),
        parsed["clustering-irregularity-weight"].as<FloatingNumber>(),
    };
    std::size_t memory_limit = parsed["clustering-max-memory"].as<std::size_t>();

    ParametrizationConfig parametrization_config
    {
        {
            parsed["parametrization-uniform-springs-rate"].as<FloatingNumber>(),
            parsed["parametrization-uniform-gradient-threshold"].as<FloatingNumber>()
        },
        {
            parsed["parametrization-l2-stretch-threshold"].as<FloatingNumber>(),
            parsed["parametrization-l2-stretch-max-iterations"].as<std::size_t>()
        }
    };

    auto workdir = output / "work";
    create_directory(workdir);

    auto plainfile = workdir / "plain";

    {
        ScopedTimer timer{"plainification"};
        obj_to_plain(input, plainfile, workdir);
    }

    process_plain(plainfile, workdir, clustering_metric_config, memory_limit, clustering_error_threshold,
        parametrization_config);

    return 0;
}