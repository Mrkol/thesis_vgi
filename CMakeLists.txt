cmake_minimum_required(VERSION 3.10)

project(thesis_vgi)

add_subdirectory(thirdparty)

find_package(glfw3 3.3 REQUIRED)
find_package(Vulkan REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(cxxopts 2.2 REQUIRED)
find_package(Threads REQUIRED)

option(CHECKS "Enables slow consistency checks" OFF)

if (CHECKS)
    add_definitions(-DCLUSTERING_CONSISTENCY_CHECKS)
endif ()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if (UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")
#    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lprofiler -ltcmalloc")
endif()

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif ()

file(GLOB_RECURSE src_common ${PROJECT_SOURCE_DIR}/src/common/*.cpp)
file(GLOB_RECURSE src_renderer ${PROJECT_SOURCE_DIR}/src/renderer/*.cpp)
file(GLOB_RECURSE src_converter ${PROJECT_SOURCE_DIR}/src/converter/*.cpp)

add_library(common STATIC ${src_common})
target_include_directories(common PRIVATE ${PROJECT_SOURCE_DIR}/include/common)
target_include_directories(common PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(common function2)
target_link_libraries(common Threads::Threads)


add_executable(renderer ${src_renderer})
target_link_libraries(renderer common)
target_link_libraries(renderer Vulkan::Vulkan)
target_link_libraries(renderer glfw)
target_link_libraries(renderer Eigen3::Eigen)

add_executable(converter ${src_converter})
target_link_libraries(converter Eigen3::Eigen)
target_link_libraries(converter cxxopts::cxxopts)
target_link_libraries(converter common)
