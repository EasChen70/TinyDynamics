#pragma once
#include <cassert>

struct RotationalBody{
    float angularPosition = 0.0f; //radians
    float angularVelocity = 0.0f; //radians per second

    float inertia = 1.0f;
    float inverseInertia = 1.0f; // reciprocal of inertia

    float torque = 0.0f;

    explicit RotationalBody(
        float initialInertia = 1.0f,
        float initialAngularPosition = 0.0f,
        float initialAngularVelocity = 0.0f
    )
        : angularPosition(initialAngularPosition),
          angularVelocity(initialAngularVelocity) {
        setInertia(initialInertia);
    }

    void setInertia(float newInertia) {
        assert(newInertia > 0.0f);
        inertia = newInertia;
        inverseInertia = 1.0f / newInertia;
    }

    void applyTorque(float appliedTorque) {
        torque += appliedTorque;
    }

    void clearTorque() {
        torque = 0.0f;
    }

    void integrate(float dt) {
        assert(dt > 0.0f);

        const float angularAcceleration = torque * inverseInertia; // derivative of angular velocity

        angularVelocity += angularAcceleration * dt;
        angularPosition += angularVelocity * dt;

        clearTorque(); // Clear torque after integration to prepare for the next time step
    }


};