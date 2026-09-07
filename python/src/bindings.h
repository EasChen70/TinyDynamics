#pragma once

#include <pybind11/pybind11.h>

// One binder per bound C++ type. Adding a new type to the Python module means
// adding one bind_*.cpp file and one call in module.cpp.

void bind_servo_actuator(pybind11::module_& m);
void bind_pan_tilt_sim(pybind11::module_& m);
