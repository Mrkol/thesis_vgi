cmake_minimum_required(VERSION 3.10)


find_package(Vulkan REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(cxxopts 2.2 REQUIRED)
find_package(Threads REQUIRED)

option(CHECKS "Enables slow consistency checks" OFF)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")

if (CHECKS)
    add_definitions(-DCLUSTERING_CONSISTENCY_CHECKS)
endif ()

file(GLOB_RECURSE src_converter ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)
file(GLOB_RECURSE shaders_converter ${CMAKE_CURRENT_LIST_DIR}/shaders/*)

add_executable(converter ${src_converter})
target_link_libraries(converter function2::function2)
target_link_libraries(converter Eigen3::Eigen)
target_link_libraries(converter Vulkan::Vulkan)
target_link_libraries(converter cxxopts::cxxopts)
target_link_libraries(converter VulkanMemoryAllocator)
target_link_libraries(converter common)
target_add_shaders(converter ${shaders_converter})
