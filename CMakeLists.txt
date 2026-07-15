cmake_minimum_required(VERSION 3.16)
project(TinyDynamics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # makes VS Code autocomplete work

# Download Catch2 (the test framework) automatically
include(FetchContent)
FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.5.4
)
FetchContent_MakeAvailable(Catch2)

# One test program for now. Add more .cpp files to this list as you write them.
add_executable(td_tests
    tests/test_vec2.cpp
)
target_include_directories(td_tests PRIVATE src)   # lets you write #include "td/Vec2.h"
target_link_libraries(td_tests PRIVATE Catch2::Catch2WithMain)