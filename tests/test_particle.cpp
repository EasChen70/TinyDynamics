#include <cassert>
#include <cmath>
#include <iostream>

#include "td/Particle.h"

void testDefaultConstructor() {
    // Default mass, position, velocity, and force are initialized correctly
    Particle p;

    assert(std::abs(p.mass - 1.0f) < 1e-6f);
    assert(std::abs(p.inverseMass - 1.0f) < 1e-6f);
    assert(p.position.nearlyEqual(Vec2(0.0f, 0.0f)));
    assert(p.velocity.nearlyEqual(Vec2(0.0f, 0.0f)));
    assert(p.force.nearlyEqual(Vec2(0.0f, 0.0f)));
}

void testCustomConstructor() {
    // Custom mass, position, and velocity are initialized correctly
    Particle p(
        2.0f,
        Vec2(1.0f, 1.0f),
        Vec2(0.5f, 0.5f)
    );

    assert(std::abs(p.mass - 2.0f) < 1e-6f);
    assert(std::abs(p.inverseMass - 0.5f) < 1e-6f);
    assert(p.position.nearlyEqual(Vec2(1.0f, 1.0f)));
    assert(p.velocity.nearlyEqual(Vec2(0.5f, 0.5f)));
    assert(p.force.nearlyEqual(Vec2(0.0f, 0.0f)));
}

void testSetMass() {
    Particle p(1.0f);

    p.setMass(4.0f);

    // Mass and inverse mass should remain consistent
    assert(std::abs(p.mass - 4.0f) < 1e-6f);
    assert(std::abs(p.inverseMass - 0.25f) < 1e-6f);
    assert(std::abs((p.mass * p.inverseMass) - 1.0f) < 1e-6f);
}

void testForceAccumulation() {
    Particle p(1.0f);

    // Multiple forces should add together
    p.applyForce(Vec2(1.0f, 0.0f));
    p.applyForce(Vec2(0.0f, 2.0f));

    assert(p.force.nearlyEqual(Vec2(1.0f, 2.0f)));

    // Negative forces should subtract from the accumulated force
    p.applyForce(Vec2(-0.5f, -1.0f));

    assert(p.force.nearlyEqual(Vec2(0.5f, 1.0f)));
}

void testClearForces() {
    Particle p(
        1.0f,
        Vec2(2.0f, 3.0f),
        Vec2(4.0f, 5.0f)
    );

    p.applyForce(Vec2(1.0f, 2.0f));
    p.clearForces();

    // Clearing forces should only reset the force accumulator
    assert(p.force.nearlyEqual(Vec2(0.0f, 0.0f)));
    assert(p.position.nearlyEqual(Vec2(2.0f, 3.0f)));
    assert(p.velocity.nearlyEqual(Vec2(4.0f, 5.0f)));
}

void testMotionWithoutForce() {
    Particle p(
        1.0f,
        Vec2(0.0f, 0.0f),
        Vec2(1.0f, 1.0f)
    );

    const float dt = 1.0f;
    p.integrate(dt);

    // Without force, velocity should stay constant
    assert(p.velocity.nearlyEqual(Vec2(1.0f, 1.0f)));

    // Position should change according to the existing velocity
    assert(p.position.nearlyEqual(Vec2(1.0f, 1.0f)));
}

void testMotionWithForce() {
    Particle p(
        2.0f,
        Vec2(0.0f, 0.0f),
        Vec2(0.0f, 0.0f)
    );

    // Force of 4 on mass 2 produces acceleration of 2
    p.applyForce(Vec2(4.0f, 0.0f));

    const float dt = 1.0f;
    p.integrate(dt);

    // velocity = initial velocity + acceleration * dt
    assert(p.velocity.nearlyEqual(Vec2(2.0f, 0.0f)));

    // Semi-implicit Euler uses the updated velocity for position
    assert(p.position.nearlyEqual(Vec2(2.0f, 0.0f)));
}

void testForcesClearedAfterIntegration() {
    Particle p(
        1.0f,
        Vec2(0.0f, 0.0f),
        Vec2(0.0f, 0.0f)
    );

    p.applyForce(Vec2(1.0f, 1.0f));

    const float dt = 1.0f;
    p.integrate(dt);

    // After integration, forces should be cleared
    assert(p.force.nearlyEqual(Vec2(0.0f, 0.0f)));
}

void testForceDoesNotPersist() {
    Particle p(
        1.0f,
        Vec2(0.0f, 0.0f),
        Vec2(0.0f, 0.0f)
    );

    // Apply force during the first simulation step
    p.applyForce(Vec2(1.0f, 0.0f));
    p.integrate(1.0f);

    assert(p.velocity.nearlyEqual(Vec2(1.0f, 0.0f)));
    assert(p.position.nearlyEqual(Vec2(1.0f, 0.0f)));

    // No new force is applied during the second step
    p.integrate(1.0f);

    // Velocity should remain constant because the old force was cleared
    assert(p.velocity.nearlyEqual(Vec2(1.0f, 0.0f)));
    assert(p.position.nearlyEqual(Vec2(2.0f, 0.0f)));
}

void testMassAffectsAcceleration() {
    Particle lightParticle(1.0f);
    Particle heavyParticle(2.0f);

    const Vec2 appliedForce(4.0f, 0.0f);

    // Apply the same force to particles with different masses
    lightParticle.applyForce(appliedForce);
    heavyParticle.applyForce(appliedForce);

    lightParticle.integrate(1.0f);
    heavyParticle.integrate(1.0f);

    // The lighter particle should accelerate more
    assert(lightParticle.velocity.nearlyEqual(Vec2(4.0f, 0.0f)));
    assert(heavyParticle.velocity.nearlyEqual(Vec2(2.0f, 0.0f)));
}

int main() {
    std::cout << "Running Particle tests...\n";

    std::cout << "  Default constructor... ";
    testDefaultConstructor();
    std::cout << "PASS\n";

    std::cout << "  Custom constructor... ";
    testCustomConstructor();
    std::cout << "PASS\n";

    std::cout << "  Set mass... ";
    testSetMass();
    std::cout << "PASS\n";

    std::cout << "  Force accumulation... ";
    testForceAccumulation();
    std::cout << "PASS\n";

    std::cout << "  Clear forces... ";
    testClearForces();
    std::cout << "PASS\n";

    std::cout << "  Motion without force... ";
    testMotionWithoutForce();
    std::cout << "PASS\n";

    std::cout << "  Motion with force... ";
    testMotionWithForce();
    std::cout << "PASS\n";

    std::cout << "  Forces cleared after integration... ";
    testForcesClearedAfterIntegration();
    std::cout << "PASS\n";

    std::cout << "  Force lifetime... ";
    testForceDoesNotPersist();
    std::cout << "PASS\n";

    std::cout << "  Mass affects acceleration... ";
    testMassAffectsAcceleration();
    std::cout << "PASS\n";

    std::cout << "\nAll Particle tests passed!\n";

    return 0;
}