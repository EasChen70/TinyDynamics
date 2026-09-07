# Python bindings for TinyDynamics (pybind11).
#
# Only entered when TD_BUILD_PYTHON is ON, which scikit-build-core sets via pyproject.toml.
# A plain `cmake -S . -B build` for the C++ tests never reaches this file.

find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_core
    src/module.cpp
    src/bind_servo_actuator.cpp
    src/bind_pan_tilt_sim.cpp
)

target_link_libraries(_core PRIVATE tinydynamics)

# Lands next to python/tinydynamics/__init__.py inside the wheel.
install(TARGETS _core DESTINATION tinydynamics)
