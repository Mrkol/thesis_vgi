cmake_minimum_required(VERSION 3.10)


find_package(Threads REQUIRED)
find_package(Vulkan REQUIRED)

file(GLOB_RECURSE src_common ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)

add_library(common STATIC ${src_common})
target_include_directories(common PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(common function2)
target_link_libraries(common Threads::Threads)
target_link_libraries(common Vulkan::Vulkan)
target_link_libraries(common VulkanMemoryAllocator)
