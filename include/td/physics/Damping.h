#pragma once

#include <cassert>
#include "td/physics/RotationalBody.h"

// Viscous damping torque for a RotationalBody (tau = -c * omega).
//
// Call apply(body) each step before body.integrate(dt); integrate() clears
// accumulated torque, so damping applied after it is lost.
// Coefficient must be >= 0 (negative values would add energy).

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