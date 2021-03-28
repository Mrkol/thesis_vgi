cmake_minimum_required(VERSION 3.10)

project(thesis_vgi)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_program(glslang_validator glslangValidator)

function(target_add_shaders TARGET GLSL_SOURCE_FILES)
    set(shader_dir "${CMAKE_CURRENT_BINARY_DIR}/shaders/")

    foreach(GLSL ${GLSL_SOURCE_FILES})
        get_filename_component(FILE_NAME ${GLSL} NAME)
        set(SPIRV "${temp_shader_dir}/${FILE_NAME}.spv")
        add_custom_command(
                OUTPUT ${SPIRV}
                COMMAND ${CMAKE_COMMAND} -E make_directory ${shader_dir}
                COMMAND ${glslang_validator} -V ${GLSL} -o ${SPIRV}
                DEPENDS ${GLSL})
        list(APPEND SPIRV_BINARY_FILES ${SPIRV})
    endforeach(GLSL)

    set(custom_target_name "${TARGET}_shaders")
    add_custom_target(${custom_target_name} DEPENDS ${SPIRV_BINARY_FILES})
    add_dependencies(${TARGET} ${custom_target_name})
endfunction()

add_subdirectory(thirdparty)

add_subdirectory(common)
add_subdirectory(converter)
add_subdirectory(renderer)
