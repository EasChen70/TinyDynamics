cmake_minimum_required(VERSION 3.16)

project(TinyDynamics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(tinydynamics INTERFACE)

target_include_directories(tinydynamics
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(test_vec2
    tests/test_vec2.cpp
)
target_link_libraries(test_vec2 PRIVATE tinydynamics)

add_executable(test_particle
    tests/test_particle.cpp
)
target_link_libraries(test_particle PRIVATE tinydynamics)

add_executable(test_rotational_body
    tests/test_rotational_body.cpp
)
target_link_libraries(test_rotational_body PRIVATE tinydynamics)

add_executable(test_damping
    tests/test_damping.cpp
)
target_link_libraries(test_damping PRIVATE tinydynamics)