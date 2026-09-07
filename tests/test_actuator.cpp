#include <cassert>
#include <cmath>
#include <iostream>

#include "td/ServoActuator.h"

constexpr float EPSILON = 1e-5f;
constexpr float PI      = 3.14159f;
constexpr float DEG     = PI / 180.0f;

bool nearlyEqual(float a, float b, float eps = EPSILON) {
    return std::abs(a - b) < eps;
}


void testDefaultsMatchDatasheet() {
    ServoActuator servo;

    // Defaults encode the MG90S spec at 4.8 V.
    assert(nearlyEqual(servo.minAngle, 0.0f));
    assert(nearlyEqual(servo.maxAngle, PI));
    assert(nearlyEqual(servo.maxAngularVelocity, 9.5f));
    assert(nearlyEqual(servo.angularDeadband, 0.016f));
    assert(nearlyEqual(servo.currentAngle, 0.0f));
    assert(nearlyEqual(servo.angularVelocity, 0.0f));
}


void testTargetClampsToTravelRange() {
    ServoActuator servo;

    servo.setTargetAngle(-1.0f);
    assert(nearlyEqual(servo.targetAngle, servo.minAngle));

    servo.setTargetAngle(10.0f);
    assert(nearlyEqual(servo.targetAngle, servo.maxAngle));

    servo.setTargetAngle(1.0f);
    assert(nearlyEqual(servo.targetAngle, 1.0f));
}


void testNoMotionInsideDeadband() {
    ServoActuator servo;
    servo.currentAngle = 1.0f;
    servo.setTargetAngle(1.0f + servo.angularDeadband * 0.5f);

    servo.update(0.01f);

    // Error below deadband: servo holds position and reports zero velocity.
    assert(nearlyEqual(servo.currentAngle, 1.0f));
    assert(nearlyEqual(servo.angularVelocity, 0.0f));
}


void testMotionJustOutsideDeadband() {
    ServoActuator servo;
    servo.currentAngle = 1.0f;
    servo.setTargetAngle(1.0f + servo.angularDeadband * 2.0f);

    servo.update(0.01f);

    // Error above deadband: servo moves.
    assert(servo.currentAngle > 1.0f);
}


void testVelocityNeverExceedsLimit() {
    ServoActuator servo;
    servo.setTargetAngle(PI);  // full 180° step

    for (int i = 0; i < 200; ++i) {
        servo.update(0.001f);
        assert(std::abs(servo.angularVelocity) <= servo.maxAngularVelocity + EPSILON);
    }
}


void testSmallStepCompletesInOneUpdate() {
    ServoActuator servo;
    const float dt = 0.01f;
    const float smallStep = servo.maxAngularVelocity * dt * 0.5f;  // half of what's reachable
    servo.setTargetAngle(smallStep);

    servo.update(dt);

    // Reachable within one step: servo lands exactly on target.
    assert(nearlyEqual(servo.currentAngle, smallStep));
}


void testLargeStepIsRateLimited() {
    ServoActuator servo;
    const float dt = 0.01f;
    servo.setTargetAngle(PI);

    servo.update(dt);

    // Not reachable in one step: moved exactly maxAngularVelocity * dt.
    assert(nearlyEqual(servo.currentAngle, servo.maxAngularVelocity * dt));
    assert(nearlyEqual(servo.angularVelocity, servo.maxAngularVelocity));
}


void testSixtyDegreeStepTakesSpecTime() {
    ServoActuator servo;
    const float dt = 0.001f;
    servo.setTargetAngle(60.0f * DEG);

    float elapsed = 0.0f;
    while (std::abs(servo.targetAngle - servo.currentAngle) >= servo.angularDeadband) {
        servo.update(dt);
        elapsed += dt;
        assert(elapsed < 1.0f);  // guard against a runaway loop
    }

    // Datasheet: 0.11 s per 60°. Allow ±10% for deadband and step quantization.
    assert(elapsed > 0.099f && elapsed < 0.121f);
}


void testSettlesOnTargetAndStops() {
    ServoActuator servo;
    servo.setTargetAngle(2.0f);

    for (int i = 0; i < 1000; ++i) {
        servo.update(0.001f);
    }

    // Within deadband of target and holding still.
    assert(nearlyEqual(servo.currentAngle, 2.0f, servo.angularDeadband));
    assert(nearlyEqual(servo.angularVelocity, 0.0f));
}


void testNegativeDirection() {
    ServoActuator servo;
    servo.currentAngle = 2.0f;
    servo.setTargetAngle(1.0f);

    servo.update(0.01f);

    // Moving toward a lower target produces negative velocity and decreasing angle.
    assert(servo.currentAngle < 2.0f);
    assert(servo.angularVelocity < 0.0f);
}


void testRetargetMidMotion() {
    ServoActuator servo;
    servo.setTargetAngle(PI);

    for (int i = 0; i < 10; ++i) {
        servo.update(0.01f);
    }
    const float midAngle = servo.currentAngle;

    // Reverse the target while still moving.
    servo.setTargetAngle(0.0f);
    servo.update(0.01f);

    // Servo immediately reverses; no momentum in a kinematic model.
    assert(servo.currentAngle < midAngle);
    assert(servo.angularVelocity < 0.0f);
}


int main() {
    std::cout << "Running ServoActuator tests...\n";

    std::cout << "  Defaults match datasheet... ";
    testDefaultsMatchDatasheet();
    std::cout << "PASS\n";

    std::cout << "  Target clamps to travel range... ";
    testTargetClampsToTravelRange();
    std::cout << "PASS\n";

    std::cout << "  No motion inside deadband... ";
    testNoMotionInsideDeadband();
    std::cout << "PASS\n";

    std::cout << "  Motion just outside deadband... ";
    testMotionJustOutsideDeadband();
    std::cout << "PASS\n";

    std::cout << "  Velocity never exceeds limit... ";
    testVelocityNeverExceedsLimit();
    std::cout << "PASS\n";

    std::cout << "  Small step completes in one update... ";
    testSmallStepCompletesInOneUpdate();
    std::cout << "PASS\n";

    std::cout << "  Large step is rate limited... ";
    testLargeStepIsRateLimited();
    std::cout << "PASS\n";

    std::cout << "  60 degree step takes spec time... ";
    testSixtyDegreeStepTakesSpecTime();
    std::cout << "PASS\n";

    std::cout << "  Settles on target and stops... ";
    testSettlesOnTargetAndStops();
    std::cout << "PASS\n";

    std::cout << "  Negative direction... ";
    testNegativeDirection();
    std::cout << "PASS\n";

    std::cout << "  Retarget mid-motion... ";
    testRetargetMidMotion();
    std::cout << "PASS\n";

    std::cout << "\nAll ServoActuator tests passed!\n";
}