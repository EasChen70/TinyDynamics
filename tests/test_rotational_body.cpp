#include <cassert>
#include <cmath>
#include <iostream>

#include "td/RotationalBody.h"

constexpr float EPSILON = 1e-6f;

bool nearlyEqual(float a, float b) {
    return std::abs(a - b) < EPSILON;
}


void testDefaultConstructor() {
    RotationalBody body;

    // A default body starts at rest with unit inertia and no applied torque.
    assert(nearlyEqual(body.inertia, 1.0f));
    assert(nearlyEqual(body.inverseInertia, 1.0f));
    assert(nearlyEqual(body.angularPosition, 0.0f));
    assert(nearlyEqual(body.angularVelocity, 0.0f));
    assert(nearlyEqual(body.torque, 0.0f));
}


void testCustomConstructor() {
    RotationalBody body(
        2.0f,  // inertia
        1.0f,  // angular position
        0.5f   // angular velocity
    );

    // Constructor should preserve the supplied rotational state.
    assert(nearlyEqual(body.inertia, 2.0f));
    assert(nearlyEqual(body.inverseInertia, 0.5f));
    assert(nearlyEqual(body.angularPosition, 1.0f));
    assert(nearlyEqual(body.angularVelocity, 0.5f));
    assert(nearlyEqual(body.torque, 0.0f));
}


void testSetInertia() {
    RotationalBody body;

    body.setInertia(4.0f);

    // Inertia and inverse inertia must remain reciprocal.
    assert(nearlyEqual(body.inertia, 4.0f));
    assert(nearlyEqual(body.inverseInertia, 0.25f));
    assert(nearlyEqual(body.inertia * body.inverseInertia, 1.0f));
}


void testTorqueAccumulation() {
    RotationalBody body;

    // Multiple torques acting during the same timestep should sum.
    body.applyTorque(3.0f);
    body.applyTorque(2.0f);

    assert(nearlyEqual(body.torque, 5.0f));

    // Opposing torque should reduce the accumulated net torque.
    body.applyTorque(-1.5f);

    assert(nearlyEqual(body.torque, 3.5f));
}


void testClearTorque() {
    RotationalBody body;

    body.applyTorque(5.0f);
    body.clearTorque();

    // Clearing torque should not modify the body's rotational state.
    assert(nearlyEqual(body.torque, 0.0f));
    assert(nearlyEqual(body.angularPosition, 0.0f));
    assert(nearlyEqual(body.angularVelocity, 0.0f));
}


void testMotionWithoutTorque() {
    RotationalBody body(
        1.0f,
        0.0f,
        2.0f
    );

    body.integrate(1.0f);

    // With zero net torque, angular velocity remains constant.
    assert(nearlyEqual(body.angularVelocity, 2.0f));

    // Position advances according to the existing angular velocity.
    assert(nearlyEqual(body.angularPosition, 2.0f));
}


void testMotionWithTorque() {
    RotationalBody body(
        2.0f,  // inertia
        0.0f,  // angular position
        0.0f   // angular velocity
    );

    body.applyTorque(4.0f);
    body.integrate(1.0f);

    // alpha = torque / inertia = 4 / 2 = 2 rad/s^2.
    // Therefore omega_new = 0 + 2 * 1 = 2 rad/s.
    assert(nearlyEqual(body.angularVelocity, 2.0f));

    // Semi-implicit Euler uses the updated angular velocity:
    // theta_new = 0 + 2 * 1 = 2 radians.
    assert(nearlyEqual(body.angularPosition, 2.0f));
}


void testNegativeTorque() {
    RotationalBody body;

    body.applyTorque(-2.0f);
    body.integrate(1.0f);

    // Negative torque produces negative angular acceleration and rotation.
    assert(nearlyEqual(body.angularVelocity, -2.0f));
    assert(nearlyEqual(body.angularPosition, -2.0f));
}


void testTorqueClearedAfterIntegration() {
    RotationalBody body;

    body.applyTorque(3.0f);
    body.integrate(1.0f);

    // Applied torque belongs only to the current simulation step.
    assert(nearlyEqual(body.torque, 0.0f));
}


void testTorqueDoesNotPersist() {
    RotationalBody body;

    // Apply torque during only the first simulation step.
    body.applyTorque(1.0f);
    body.integrate(1.0f);

    assert(nearlyEqual(body.angularVelocity, 1.0f));
    assert(nearlyEqual(body.angularPosition, 1.0f));

    // No torque is applied during the second step.
    body.integrate(1.0f);

    // Angular velocity stays constant because the old torque was cleared.
    assert(nearlyEqual(body.angularVelocity, 1.0f));
    assert(nearlyEqual(body.angularPosition, 2.0f));
}


void testInertiaAffectsAngularAcceleration() {
    RotationalBody lowInertia(1.0f);
    RotationalBody highInertia(2.0f);

    const float appliedTorque = 4.0f;

    // Apply the same torque to bodies with different moments of inertia.
    lowInertia.applyTorque(appliedTorque);
    highInertia.applyTorque(appliedTorque);

    lowInertia.integrate(1.0f);
    highInertia.integrate(1.0f);

    // alpha = torque / inertia, so larger inertia means less acceleration.
    assert(nearlyEqual(lowInertia.angularVelocity, 4.0f));
    assert(nearlyEqual(highInertia.angularVelocity, 2.0f));
}


int main() {
    std::cout << "Running RotationalBody tests...\n";

    std::cout << "  Default constructor... ";
    testDefaultConstructor();
    std::cout << "PASS\n";

    std::cout << "  Custom constructor... ";
    testCustomConstructor();
    std::cout << "PASS\n";

    std::cout << "  Set inertia... ";
    testSetInertia();
    std::cout << "PASS\n";

    std::cout << "  Torque accumulation... ";
    testTorqueAccumulation();
    std::cout << "PASS\n";

    std::cout << "  Clear torque... ";
    testClearTorque();
    std::cout << "PASS\n";

    std::cout << "  Motion without torque... ";
    testMotionWithoutTorque();
    std::cout << "PASS\n";

    std::cout << "  Motion with torque... ";
    testMotionWithTorque();
    std::cout << "PASS\n";

    std::cout << "  Negative torque... ";
    testNegativeTorque();
    std::cout << "PASS\n";

    std::cout << "  Torque cleared after integration... ";
    testTorqueClearedAfterIntegration();
    std::cout << "PASS\n";

    std::cout << "  Torque lifetime... ";
    testTorqueDoesNotPersist();
    std::cout << "PASS\n";

    std::cout << "  Inertia affects angular acceleration... ";
    testInertiaAffectsAngularAcceleration();
    std::cout << "PASS\n";

    std::cout << "\nAll RotationalBody tests passed!\n";
}