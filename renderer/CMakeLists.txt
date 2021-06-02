cmake_minimum_required(VERSION 3.10)


find_package(glfw3 3.3 REQUIRED)
find_package(Vulkan REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(cxxopts 2.2 REQUIRED)
find_package(Threads REQUIRED)

option(CHECKS "Enables slow consistency checks" OFF)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")

if (CHECKS)
    add_definitions(-DCLUSTERING_CONSISTENCY_CHECKS)
endif ()


file(GLOB_RECURSE src_renderer ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)
file(GLOB_RECURSE shaders_renderer ${CMAKE_CURRENT_LIST_DIR}/shaders/*)

add_executable(renderer ${src_renderer})
target_link_libraries(renderer common)
target_link_libraries(renderer Vulkan::Vulkan)
target_link_libraries(renderer cxxopts::cxxopts)
target_link_libraries(renderer glfw)
target_link_libraries(renderer Eigen3::Eigen)
target_link_libraries(renderer VulkanMemoryAllocator)
target_link_libraries(renderer DearImGui)
target_link_libraries(renderer Stb)
target_link_libraries(renderer tinyobjloader)
target_add_shaders(renderer ${shaders_renderer})
