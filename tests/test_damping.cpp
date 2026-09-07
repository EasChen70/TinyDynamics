#include <cassert>
#include <cmath>
#include <iostream>

#include "td/Damping.h"
#include "td/RotationalBody.h"

constexpr float EPSILON = 1e-6f;

bool nearlyEqual(float a, float b) {
    return std::abs(a - b) < EPSILON;
}


void testZeroCoefficientHasNoEffect() {
    RotationalBody body(1.0f, 0.0f, 3.0f);
    AngularDamping damping(0.0f);

    damping.apply(body);

    assert(nearlyEqual(body.torque, 0.0f));
}


void testBodyAtRestReceivesNoTorque() {
    RotationalBody body;
    AngularDamping damping(2.0f);

    damping.apply(body);

    // Damping is proportional to velocity, so zero velocity means zero torque.
    assert(nearlyEqual(body.torque, 0.0f));
}


void testDampingTorqueOpposesMotion() {
    RotationalBody spinningForward(1.0f, 0.0f, 2.0f);
    RotationalBody spinningBackward(1.0f, 0.0f, -2.0f);
    AngularDamping damping(0.5f);

    damping.apply(spinningForward);
    damping.apply(spinningBackward);

    // torque = -c * omega
    assert(nearlyEqual(spinningForward.torque, -1.0f));
    assert(nearlyEqual(spinningBackward.torque, 1.0f));
}


void testDampingTorqueScalesWithVelocity() {
    RotationalBody slow(1.0f, 0.0f, 1.0f);
    RotationalBody fast(1.0f, 0.0f, 4.0f);
    AngularDamping damping(1.0f);

    damping.apply(slow);
    damping.apply(fast);

    assert(nearlyEqual(fast.torque, 4.0f * slow.torque));
}


void testDampingAccumulatesWithOtherTorques() {
    RotationalBody body(1.0f, 0.0f, 2.0f);
    AngularDamping damping(1.0f);

    body.applyTorque(5.0f);
    damping.apply(body);

    // Damping should add to, not replace, existing torque.
    assert(nearlyEqual(body.torque, 3.0f));
}


void testVelocityDecreasesAfterIntegration() {
    RotationalBody body(1.0f, 0.0f, 4.0f);
    AngularDamping damping(0.5f);

    damping.apply(body);
    body.integrate(0.1f);

    // alpha = -0.5 * 4 / 1 = -2, omega_new = 4 - 2 * 0.1 = 3.8
    assert(nearlyEqual(body.angularVelocity, 3.8f));
    assert(body.angularVelocity > 0.0f);
}


void testRepeatedDampingDecaysTowardRest() {
    RotationalBody body(1.0f, 0.0f, 10.0f);
    AngularDamping damping(1.0f);
    const float dt = 0.01f;

    float previousSpeed = std::abs(body.angularVelocity);

    for (int step = 0; step < 1000; ++step) {
        damping.apply(body);
        body.integrate(dt);

        const float speed = std::abs(body.angularVelocity);

        // Speed must decrease monotonically and never flip sign.
        assert(speed < previousSpeed);
        assert(body.angularVelocity > 0.0f);
        previousSpeed = speed;
    }

    // After 10 seconds with c = 1, omega ~ 10 * e^-10, effectively at rest.
    assert(std::abs(body.angularVelocity) < 1e-3f);
}


void testDampingDoesNotAffectPositionDirectly() {
    RotationalBody body(1.0f, 1.5f, 2.0f);
    AngularDamping damping(3.0f);

    damping.apply(body);

    // apply() only queues torque; position changes only through integrate().
    assert(nearlyEqual(body.angularPosition, 1.5f));
    assert(nearlyEqual(body.angularVelocity, 2.0f));
}


int main() {
    std::cout << "Running AngularDamping tests...\n";

    std::cout << "  Zero coefficient has no effect... ";
    testZeroCoefficientHasNoEffect();
    std::cout << "PASS\n";

    std::cout << "  Body at rest receives no torque... ";
    testBodyAtRestReceivesNoTorque();
    std::cout << "PASS\n";

    std::cout << "  Damping torque opposes motion... ";
    testDampingTorqueOpposesMotion();
    std::cout << "PASS\n";

    std::cout << "  Damping torque scales with velocity... ";
    testDampingTorqueScalesWithVelocity();
    std::cout << "PASS\n";

    std::cout << "  Damping accumulates with other torques... ";
    testDampingAccumulatesWithOtherTorques();
    std::cout << "PASS\n";

    std::cout << "  Velocity decreases after integration... ";
    testVelocityDecreasesAfterIntegration();
    std::cout << "PASS\n";

    std::cout << "  Repeated damping decays toward rest... ";
    testRepeatedDampingDecaysTowardRest();
    std::cout << "PASS\n";

    std::cout << "  Damping does not affect position directly... ";
    testDampingDoesNotAffectPositionDirectly();
    std::cout << "PASS\n";

    std::cout << "\nAll AngularDamping tests passed!\n";
}