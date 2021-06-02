cmake_minimum_required(VERSION 3.10)

find_package(glfw3 3.3 REQUIRED)
find_package(Vulkan REQUIRED)



add_subdirectory(function2)
add_subdirectory(tinyobjloader)

add_library(VulkanMemoryAllocator INTERFACE)
target_include_directories(VulkanMemoryAllocator INTERFACE VulkanMemoryAllocator/include/)

add_library(Stb INTERFACE)
target_include_directories(Stb INTERFACE stb)

add_library(DearImGui imgui/imgui.cpp imgui/imgui_draw.cpp imgui/imgui_tables.cpp imgui/imgui_widgets.cpp
        imgui/imgui_demo.cpp
        imgui/backends/imgui_impl_vulkan.cpp imgui/backends/imgui_impl_glfw.cpp)
target_include_directories(DearImGui PUBLIC imgui)
target_add_shaders(DearImGui ${CMAKE_CURRENT_LIST_DIR}/imgui/backends/vulkan/glsl_shader.frag
        ${CMAKE_CURRENT_LIST_DIR}/imgui/backends/vulkan/glsl_shader.vert)
target_link_libraries(DearImGui Vulkan::Vulkan)
target_link_libraries(DearImGui glfw)
