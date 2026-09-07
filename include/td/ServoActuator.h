#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>

// ServoActuator V1
// Modeled after an MG90S 9g metal-gear micro servo.
//
// Relevant specs:
// - Stall torque: 2.0 kgf·cm @ 4.8 V
// - No-load speed: 0.11 s / 60° @ 4.8 V
// - Operating voltage: 4.8 V
// - Mass: ~9 g
// - Deadband: 5 µs
// - Travel range: 180°
//
// V1 is kinematic: a rate-limited position follower with no inertia, torque,
// acceleration limits, or overshoot. It provides a simple baseline for LookAt
// using the MG90S rated no-load speed. Load-dependent behavior can be added
// later using measured hardware response and a torque-based model that drives
// a RotationalBody. Stall torque is therefore listed above but unused in V1.

struct ServoActuator {
    // Configuration (MG90S @ 4.8 V)
    //
    // Defaults on every member so `ServoActuator s;` is fully defined.
    // Without them, reading currentAngle before assignment is undefined behavior.
    float minAngle           = 0.0f;
    float maxAngle           = 3.14159f;  // 180° travel
    float maxAngularVelocity = 9.5f;      // 0.11 s / 60° → 545°/s

    // Angular equivalent of the 5 µs pulse-width deadband, assuming a 1000 µs
    // pulse span maps to the full 180° travel: 5/1000 × 180° = 0.9° ≈ 0.016 rad.
    // Named in angle units so it isn't mistaken for the electrical 5 µs spec.
    float angularDeadband    = 0.016f;

    // State
    float targetAngle     = 0.0f;
    float currentAngle    = 0.0f;
    float angularVelocity = 0.0f;

    void setTargetAngle(float angle) {
        // Clamp to mechanical travel. std::clamp replaces the if/else ladder.
        targetAngle = std::clamp(angle, minAngle, maxAngle);
    }

    void update(float dt) {
        // A non-positive dt makes the angularVelocity division below invalid.
        assert(dt > 0.0f);

        const float error = targetAngle - currentAngle;

        // Deadband: real servos ignore errors smaller than their pulse resolution.
        // This is what makes the servo sit still at rest instead of twitching.
        if (std::abs(error) < angularDeadband) {
            angularVelocity = 0.0f;
            return;
        }

        // Move toward the target by at most maxAngularVelocity * dt this step.
        // Equivalent to clamping error/dt to ±maxAngularVelocity, but phrased
        // as a distance so it reads as intended and avoids dividing by a tiny dt.
        const float maxStep = maxAngularVelocity * dt;
        const float step = std::clamp(error, -maxStep, maxStep);

        currentAngle += step;
        angularVelocity = step / dt;
    }
};