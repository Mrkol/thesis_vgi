cmake_minimum_required(VERSION 3.10)

add_subdirectory(function2)

add_library(VulkanMemoryAllocator INTERFACE)
target_include_directories(VulkanMemoryAllocator INTERFACE VulkanMemoryAllocator/include/)
