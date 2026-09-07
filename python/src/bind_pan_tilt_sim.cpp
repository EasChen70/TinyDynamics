#include <pybind11/pybind11.h>

#include "bindings.h"
#include "td/PanTiltSim.h"

namespace py = pybind11;

void bind_pan_tilt_sim(py::module_& m) {
    py::class_<PanTiltSim>(m, "PanTiltSim",
        "Two ServoActuators composed into one simulated pan/tilt unit.\n"
        "Angles are raw servo coordinates in radians (default 0 to pi per axis),\n"
        "times are seconds.")
        .def(py::init<>())

        // Read-only members: the returned object is a live reference to the child
        // servo, so `sim.pan.max_angular_velocity = 5.0` tunes the sim in place,
        // while `sim.pan = ServoActuator()` is rejected.
        .def_readonly("pan", &PanTiltSim::pan,
            "The pan-axis ServoActuator (live reference; tune its config here).")
        .def_readonly("tilt", &PanTiltSim::tilt,
            "The tilt-axis ServoActuator (live reference; tune its config here).")

        // Commands
        .def("set_pan_target", &PanTiltSim::setPanTarget, py::arg("angle"),
            "Command the pan target [rad]. Clamped to the pan servo's travel.")
        .def("set_tilt_target", &PanTiltSim::setTiltTarget, py::arg("angle"),
            "Command the tilt target [rad]. Clamped to the tilt servo's travel.")

        // Simulation
        .def("update", &PanTiltSim::update, py::arg("dt"),
            "Advance both axes by dt seconds. dt must be > 0.")

        // Observation
        .def("pan_angle", &PanTiltSim::panAngle,
            "Current pan shaft angle [rad].")
        .def("tilt_angle", &PanTiltSim::tiltAngle,
            "Current tilt shaft angle [rad].")
        .def("pan_velocity", &PanTiltSim::panVelocity,
            "Pan shaft velocity from the last update [rad/s].")
        .def("tilt_velocity", &PanTiltSim::tiltVelocity,
            "Tilt shaft velocity from the last update [rad/s].");
}
