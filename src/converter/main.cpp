#include <cxxopts.hpp>

#include "Gridify.hpp"


int main(int argc, char** argv)
{
    cxxopts::Options options("VGI Converter", "Converts .obj files to VGI format");

    options.add_options()
        ("i,input", "Input obj file", cxxopts::value<std::string>())
        ("o,output", "Output directory", cxxopts::value<std::string>())
        ;

    auto parsed = options.parse(argc, argv);

    Gridify::gridify({parsed["i"].as<std::string>(), parsed["o"].as<std::string>()});


    return 0;
}
