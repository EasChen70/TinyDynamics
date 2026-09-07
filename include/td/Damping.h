#pragma once

#include <cassert>
#include "td/RotationalBody.h"

struct AngularDamping {
    float coefficient = 0.0f; // Damping coefficient

    explicit AngularDamping(float initialCoefficient = 0.0f)
        : coefficient(initialCoefficient) {
        assert(initialCoefficient >= 0.0f);
    }

    void apply(RotationalBody& body) const {
        // Damping torque opposes angular motion:
        // tau_d = -c * omega
        const float dampingTorque =
            -coefficient * body.angularVelocity;

        body.applyTorque(dampingTorque);
    }
};