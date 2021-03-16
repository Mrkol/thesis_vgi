#pragma once

#include <filesystem>


constexpr auto BORDER_FILENAME = "border";

void gridify(const std::filesystem::path& input_file, const std::filesystem::path& output_directory);
