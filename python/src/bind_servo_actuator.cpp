#include <pybind11/pybind11.h>

#include "bindings.h"
#include "td/ServoActuator.h"

namespace py = pybind11;

void bind_servo_actuator(py::module_& m) {
    py::class_<ServoActuator>(m, "ServoActuator",
        "Kinematic MG90S servo model (V1): a rate-limited position follower with\n"
        "travel limits and a deadband. No inertia, torque, or overshoot.\n"
        "Angles are radians, times are seconds.")
        .def(py::init<>())

        // Configuration: writable so Python can calibrate against real hardware.
        .def_readwrite("min_angle", &ServoActuator::minAngle,
            "Lower travel limit [rad].")
        .def_readwrite("max_angle", &ServoActuator::maxAngle,
            "Upper travel limit [rad].")
        .def_readwrite("max_angular_velocity", &ServoActuator::maxAngularVelocity,
            "Rated no-load speed [rad/s].")
        .def_readwrite("angular_deadband", &ServoActuator::angularDeadband,
            "Errors smaller than this are ignored [rad]. Angular equivalent of the\n"
            "5 us pulse-width deadband; not the electrical spec itself.")

        // State: read-only. Only set_target_angle() and update() may change these,
        // so Python cannot bypass clamping or the rate limit.
        .def_readonly("target_angle", &ServoActuator::targetAngle,
            "Commanded angle after clamping [rad].")
        .def_readonly("current_angle", &ServoActuator::currentAngle,
            "Actual shaft angle [rad].")
        .def_readonly("angular_velocity", &ServoActuator::angularVelocity,
            "Shaft velocity from the last update [rad/s].")

        .def("set_target_angle", &ServoActuator::setTargetAngle, py::arg("angle"),
            "Command a target angle [rad]. Clamped to [min_angle, max_angle].")
        .def("update", &ServoActuator::update, py::arg("dt"),
            "Advance the servo by dt seconds. dt must be > 0.");
}
