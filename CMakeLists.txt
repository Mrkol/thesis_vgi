cmake_minimum_required(VERSION 3.10)

project(thesis_vgi)

find_package(glfw3 3.3 REQUIRED)
find_package(Vulkan REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(cxxopts 2.2 REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)


file(GLOB_RECURSE src_common ${PROJECT_SOURCE_DIR}/src/common/*.cpp)
file(GLOB_RECURSE src_renderer ${PROJECT_SOURCE_DIR}/src/renderer/*.cpp)
file(GLOB_RECURSE src_converter ${PROJECT_SOURCE_DIR}/src/converter/*.cpp)

add_library(common STATIC ${src_common})


add_executable(renderer ${src_renderer})
target_link_libraries(renderer common)
target_link_libraries(renderer Vulkan::Vulkan)
target_link_libraries(renderer glfw)
target_link_libraries(renderer Eigen3::Eigen)

add_executable(converter ${src_converter})
target_link_libraries(converter Eigen3::Eigen)
target_link_libraries(converter cxxopts::cxxopts)
target_link_libraries(converter common)
 