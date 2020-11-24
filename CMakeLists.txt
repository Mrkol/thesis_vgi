cmake_minimum_required(VERSION 3.10)

project(thesis_vgi)

find_package(glfw3 3.3 REQUIRED)
find_package(GLM REQUIRED)
find_package(Vulkan REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB_RECURSE src_common ${PROJECT_SOURCE_DIR}/src/common/*.cpp)
file(GLOB_RECURSE src_renderer ${PROJECT_SOURCE_DIR}/src/renderer/*.cpp)
file(GLOB_RECURSE src_converter ${PROJECT_SOURCE_DIR}/src/converter/*.cpp)

add_executable(renderer ${src_common} ${src_renderer})
target_link_libraries(renderer Vulkan::Vulkan)
target_link_libraries(renderer glfw)
target_link_libraries(renderer glm::glm)

add_executable(converter ${src_common} ${src_converter})

if(MSVC_IDE)
	# Macro to preserve source files hierarchy in the IDE
	macro(GroupSources curdir)
		file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir} ${PROJECT_SOURCE_DIR}/${curdir}/*)

		foreach(child ${children})
			if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
				GroupSources(${curdir}/${child})
			else()
				string(REPLACE "/" "\\" groupname ${curdir})
				string(REPLACE "src" "Sources" groupname ${groupname})
				source_group(${groupname} FILES ${PROJECT_SOURCE_DIR}/${curdir}/${child})
			endif()
		endforeach()
	endmacro()

	# Run macro
	GroupSources(src)
endif()
