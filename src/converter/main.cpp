#include <cxxopts.hpp>

#include "ExternalPartBreaker.hpp"


int main(int argc, char** argv)
{
    cxxopts::Options options("VGI Converter", "Converts .obj files to VGI format");

    options.add_options()
        ("i,input", "Input obj file", cxxopts::value<std::string>())
        ("o,output", "Output directory", cxxopts::value<std::string>())
        ;

    auto parsed = options.parse(argc, argv);

    ExternalPartBreaker::break_into_parts({parsed["i"].as<std::string>(), parsed["o"].as<std::string>()});


    return 0;
}
