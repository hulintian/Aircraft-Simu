cmake_minimum_required(VERSION 3.16)

project(missile_sim C)

include(CheckCSourceCompiles)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

option(MISSILE_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(MISSILE_ENABLE_COVERAGE "Enable GCC/Clang coverage instrumentation" OFF)
option(MISSILE_ENABLE_LONG_TESTS "Enable optional long-running campaign pressure tests" OFF)

if(MISSILE_ENABLE_SANITIZERS AND MISSILE_ENABLE_COVERAGE)
    message(FATAL_ERROR "MISSILE_ENABLE_SANITIZERS and MISSILE_ENABLE_COVERAGE are mutually exclusive")
endif()

add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wshadow
    -Wmissing-prototypes
)

if(MISSILE_ENABLE_SANITIZERS)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        set(CMAKE_REQUIRED_FLAGS "-fsanitize=address,undefined -fno-omit-frame-pointer")
        set(CMAKE_REQUIRED_LINK_OPTIONS "-fsanitize=address,undefined")
        check_c_source_compiles("int main(void) { return 0; }" MISSILE_SANITIZER_RUNTIME_AVAILABLE)
        unset(CMAKE_REQUIRED_FLAGS)
        unset(CMAKE_REQUIRED_LINK_OPTIONS)
        if(NOT MISSILE_SANITIZER_RUNTIME_AVAILABLE)
            message(FATAL_ERROR "MISSILE_ENABLE_SANITIZERS requires available ASan/UBSan runtime libraries")
        endif()
        add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
        add_link_options(-fsanitize=address,undefined)
    else()
        message(FATAL_ERROR "MISSILE_ENABLE_SANITIZERS requires GCC or Clang")
    endif()
endif()

if(MISSILE_ENABLE_COVERAGE)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        add_compile_options(-O0 -g --coverage)
        add_link_options(--coverage)
    else()
        message(FATAL_ERROR "MISSILE_ENABLE_COVERAGE requires GCC or Clang")
    endif()
endif()

enable_testing()

add_subdirectory(common)
add_subdirectory(environment_sim)
add_subdirectory(flight_control_sim)
add_subdirectory(tools/instance_manager)
add_subdirectory(tools/log_convert)
add_subdirectory(tools/replay)
add_subdirectory(tools/compare_logs)
add_subdirectory(tools/batch_stats)
add_subdirectory(tools/map_preprocess)
add_subdirectory(tools/batch_runner)
add_subdirectory(tests)
