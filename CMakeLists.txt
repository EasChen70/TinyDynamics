cmake_minimum_required(VERSION 3.16)

project(TinyDynamics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Vec2 tests
add_executable(test_vec2
    tests/test_vec2.cpp
)

target_include_directories(test_vec2
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Particle tests
add_executable(test_particle
    tests/test_particle.cpp
)

target_include_directories(test_particle
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)