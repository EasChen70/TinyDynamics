#pragma once

#include <cassert>
#include "td/math/Vec2.h"

// 2D point mass.
//
// Per time step:
//   1. applyForce() any number of times; forces accumulate.
//   2. integrate(dt) advances velocity and position, then clears forces.
//
// Mass must be > 0. inverseMass is cached so integrate() multiplies
// instead of divides; always change mass through setMass() to keep it in sync.

struct Particle {
    float mass;
    float inverseMass; // reciprocal of mass

    Vec2 position;
    Vec2 velocity; // derivative of position 
    Vec2 force; // rate of change of momentum


    explicit Particle(
        float initialMass = 1.0f, 
        const Vec2& initialPosition = Vec2(), 
        const Vec2& initialVelocity = Vec2())
        : mass(1.0f), 
          inverseMass(1.0f), 
          position(initialPosition), 
          velocity(initialVelocity), 
          force() {
        setMass(initialMass);
    }

    void applyForce(const Vec2& appliedForce) {
        force += appliedForce;
    }

    void clearForces() {
        force = Vec2();
    }

    void setMass(float newMass) {
        assert(newMass > 0.0f);
        mass = newMass;
        inverseMass = 1.0f / newMass;
    }

    // Semi-implicit (symplectic) Euler: update velocity first, then move
    // position with the new velocity. More stable than explicit Euler for
    // oscillating systems at the same dt.
    void integrate(float dt) {
        assert(dt > 0.0f);

        const Vec2 acceleration = force * inverseMass; // derivative of velocity

        velocity += acceleration * dt;
        position += velocity * dt;

        clearForces(); // Clear forces after integration to prepare for the next time step
    }
    
};