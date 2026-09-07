#include <pybind11/pybind11.h>

#include "bindings.h"

namespace py = pybind11;

PYBIND11_MODULE(_core, m) {
    m.doc() =
        "TinyDynamics: simulated pan/tilt servo plant for LookAt.\n"
        "All angles are radians, all times are seconds.";

    bind_servo_actuator(m);
    bind_pan_tilt_sim(m);
}
