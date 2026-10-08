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
    enable_testing()

    # Each name builds tests/test_<name>.cpp into test_<name> and registers it
    # with CTest. Adding a test file means adding one name here.
    set(TD_TESTS
        vec2
        vec3
        particle
        rotational_body
        damping
        actuator
        pan_tilt_sim
    )

    foreach(name IN LISTS TD_TESTS)
        add_executable(test_${name} tests/test_${name}.cpp)
        target_link_libraries(test_${name} PRIVATE tinydynamics)
        add_test(NAME ${name} COMMAND test_${name})
    endforeach()
endif()

if(TD_BUILD_PYTHON)
    add_subdirectory(python)
endif()
