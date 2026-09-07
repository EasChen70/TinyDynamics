#pragma once

#include <cassert>
#include "td/Vec2.h"

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

    /* 
    integrating a particle refers to using calculus or numerical methods to calculate 
    how a particle's position, velocity, and acceleration change over time
    */
    void integrate(float dt) {
        assert(dt > 0.0f);

        const Vec2 acceleration = force * inverseMass; // derivative of velocity

        velocity += acceleration * dt;
        position += velocity * dt;

        clearForces(); // Clear forces after integration to prepare for the next time step
    }
    
};