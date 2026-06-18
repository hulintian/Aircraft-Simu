cmake_minimum_required(VERSION 3.16)

project(missile_sim C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wshadow
    -Wmissing-prototypes
)

enable_testing()

add_subdirectory(common)
add_subdirectory(environment_sim)
add_subdirectory(flight_control_sim)
add_subdirectory(tools/instance_manager)
add_subdirectory(tests)
