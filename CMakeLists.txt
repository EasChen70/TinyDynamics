cmake_minimum_required(VERSION 3.16)

project(TinyDynamics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Plain `cmake -S . -B build` builds the C++ tests and nothing Python-related.
# scikit-build-core (pip install) flips both switches via pyproject.toml so a
# wheel build never compiles the test executables and never needs them.
option(TD_BUILD_TESTS  "Build the C++ test executables" ON)
option(TD_BUILD_PYTHON "Build the pybind11 Python module" OFF)

add_library(tinydynamics INTERFACE)

target_include_directories(tinydynamics
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(TD_BUILD_TESTS)
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

    add_executable(test_actuator
        tests/test_actuator.cpp
    )
    target_link_libraries(test_actuator PRIVATE tinydynamics)

    add_executable(test_pan_tilt_sim
        tests/test_pan_tilt_sim.cpp
    )
    target_link_libraries(test_pan_tilt_sim PRIVATE tinydynamics)
endif()

if(TD_BUILD_PYTHON)
    add_subdirectory(python)
endif()
