#pragma once

#include "td/ServoActuator.h"

// PanTiltSim V1
// Composes two ServoActuators into one simulated pan/tilt unit.
//
//                 PanTiltSim
//                 |        |
//           pan servo    tilt servo
//                 |        |
//           pan angle    tilt angle
//
// Responsibilities:
// - coordinate two axes
// - expose simple pan/tilt commands
// - expose state
//
// All servo physics (travel limits, speed limit, deadband, shaft update) live
// in ServoActuator. PanTiltSim must not duplicate any of it.
//
// V1 works in raw servo coordinates (default 0 → π per axis). A later version
// may become the place that translates logical camera coordinates
// (0 = centered, ±π/2 travel) to and from raw servo angles.

struct PanTiltSim {
    ServoActuator pan;
    ServoActuator tilt;

    void setPanTarget(float angle) {
        pan.setTargetAngle(angle);
    }

    void setTiltTarget(float angle) {
        tilt.setTargetAngle(angle);
    }

    void update(float dt) {
        pan.update(dt);
        tilt.update(dt);
    }

    float panAngle() const {
        return pan.currentAngle;
    }

    float tiltAngle() const {
        return tilt.currentAngle;
    }

    float panVelocity() const {
        return pan.angularVelocity;
    }

    float tiltVelocity() const {
        return tilt.angularVelocity;
    }
};
