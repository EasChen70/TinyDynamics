#include <cassert>
#include <cmath>
#include <iostream>

#include "td/PanTiltSim.h"

constexpr float EPSILON = 1e-5f;
constexpr float PI      = 3.14159f;

bool nearlyEqual(float a, float b, float eps = EPSILON) {
    return std::abs(a - b) < eps;
}


void testDefaultsStartAtZero() {
    PanTiltSim sim;

    assert(nearlyEqual(sim.panAngle(), 0.0f));
    assert(nearlyEqual(sim.tiltAngle(), 0.0f));
    assert(nearlyEqual(sim.panVelocity(), 0.0f));
    assert(nearlyEqual(sim.tiltVelocity(), 0.0f));
}


void testPanMovesIndependently() {
    PanTiltSim sim;

    sim.setPanTarget(1.0f);
    sim.update(0.01f);

    // Only the pan axis was commanded: tilt must stay put.
    assert(sim.panAngle() > 0.0f);
    assert(nearlyEqual(sim.tiltAngle(), 0.0f));
    assert(nearlyEqual(sim.tiltVelocity(), 0.0f));
}


void testTiltMovesIndependently() {
    PanTiltSim sim;

    sim.setTiltTarget(1.0f);
    sim.update(0.01f);

    // Only the tilt axis was commanded: pan must stay put.
    assert(sim.tiltAngle() > 0.0f);
    assert(nearlyEqual(sim.panAngle(), 0.0f));
    assert(nearlyEqual(sim.panVelocity(), 0.0f));
}


void testBothAxesMove() {
    PanTiltSim sim;

    sim.setPanTarget(1.0f);
    sim.setTiltTarget(0.5f);

    sim.update(0.01f);

    assert(sim.panAngle() > 0.0f);
    assert(sim.tiltAngle() > 0.0f);
}


void testAxesSettleIndependently() {
    PanTiltSim sim;

    // Tilt has a short trip and should arrive well before pan does.
    sim.setPanTarget(PI);
    sim.setTiltTarget(0.1f);

    for (int i = 0; i < 20; ++i) {
        sim.update(0.001f);
    }

    // Tilt (0.1 rad at 9.5 rad/s ≈ 0.0105 s) has settled and stopped.
    assert(nearlyEqual(sim.tiltAngle(), 0.1f, sim.tilt.angularDeadband));
    assert(nearlyEqual(sim.tiltVelocity(), 0.0f));

    // Pan (π rad ≈ 0.33 s) is still mid-travel at full speed.
    assert(sim.panAngle() < PI * 0.5f);
    assert(nearlyEqual(sim.panVelocity(), sim.pan.maxAngularVelocity));
}


void testPanTargetClamps() {
    PanTiltSim sim;

    sim.setPanTarget(-1.0f);
    assert(nearlyEqual(sim.pan.targetAngle, sim.pan.minAngle));

    sim.setPanTarget(10.0f);
    assert(nearlyEqual(sim.pan.targetAngle, sim.pan.maxAngle));

    sim.setPanTarget(1.0f);
    assert(nearlyEqual(sim.pan.targetAngle, 1.0f));
}


void testTiltTargetClamps() {
    PanTiltSim sim;

    sim.setTiltTarget(-1.0f);
    assert(nearlyEqual(sim.tilt.targetAngle, sim.tilt.minAngle));

    sim.setTiltTarget(10.0f);
    assert(nearlyEqual(sim.tilt.targetAngle, sim.tilt.maxAngle));

    sim.setTiltTarget(1.0f);
    assert(nearlyEqual(sim.tilt.targetAngle, 1.0f));
}


void testUpdateAdvancesBothAxes() {
    PanTiltSim sim;
    const float dt = 0.01f;

    sim.setPanTarget(PI);
    sim.setTiltTarget(PI);

    sim.update(dt);

    // Each axis advanced by exactly one rate-limited step, matching a lone servo.
    const float expectedStep = sim.pan.maxAngularVelocity * dt;
    assert(nearlyEqual(sim.panAngle(), expectedStep));
    assert(nearlyEqual(sim.tiltAngle(), expectedStep));

    sim.update(dt);

    assert(nearlyEqual(sim.panAngle(), 2.0f * expectedStep));
    assert(nearlyEqual(sim.tiltAngle(), 2.0f * expectedStep));
}


void testMatchesStandaloneServos() {
    // PanTiltSim must not add or alter any physics: driving it must produce
    // exactly the same trajectory as driving two ServoActuators by hand.
    PanTiltSim sim;
    ServoActuator refPan;
    ServoActuator refTilt;

    sim.setPanTarget(2.0f);
    sim.setTiltTarget(0.7f);
    refPan.setTargetAngle(2.0f);
    refTilt.setTargetAngle(0.7f);

    for (int i = 0; i < 300; ++i) {
        sim.update(0.001f);
        refPan.update(0.001f);
        refTilt.update(0.001f);

        assert(nearlyEqual(sim.panAngle(), refPan.currentAngle));
        assert(nearlyEqual(sim.tiltAngle(), refTilt.currentAngle));
        assert(nearlyEqual(sim.panVelocity(), refPan.angularVelocity));
        assert(nearlyEqual(sim.tiltVelocity(), refTilt.angularVelocity));
    }
}


void testGettersMatchUnderlyingServos() {
    PanTiltSim sim;

    sim.setPanTarget(PI);
    sim.setTiltTarget(0.3f);
    sim.update(0.01f);

    // Getters are pure views onto the servo state.
    assert(nearlyEqual(sim.panAngle(), sim.pan.currentAngle));
    assert(nearlyEqual(sim.tiltAngle(), sim.tilt.currentAngle));
    assert(nearlyEqual(sim.panVelocity(), sim.pan.angularVelocity));
    assert(nearlyEqual(sim.tiltVelocity(), sim.tilt.angularVelocity));

    // And they reflect real motion, not just zeros.
    assert(sim.panVelocity() > 0.0f);
    assert(sim.tiltVelocity() > 0.0f);
}


int main() {
    std::cout << "Running PanTiltSim tests...\n";

    std::cout << "  Defaults start at zero... ";
    testDefaultsStartAtZero();
    std::cout << "PASS\n";

    std::cout << "  Pan moves independently... ";
    testPanMovesIndependently();
    std::cout << "PASS\n";

    std::cout << "  Tilt moves independently... ";
    testTiltMovesIndependently();
    std::cout << "PASS\n";

    std::cout << "  Both axes move... ";
    testBothAxesMove();
    std::cout << "PASS\n";

    std::cout << "  Axes settle independently... ";
    testAxesSettleIndependently();
    std::cout << "PASS\n";

    std::cout << "  Pan target clamps... ";
    testPanTargetClamps();
    std::cout << "PASS\n";

    std::cout << "  Tilt target clamps... ";
    testTiltTargetClamps();
    std::cout << "PASS\n";

    std::cout << "  update(dt) advances both axes... ";
    testUpdateAdvancesBothAxes();
    std::cout << "PASS\n";

    std::cout << "  Matches standalone servos... ";
    testMatchesStandaloneServos();
    std::cout << "PASS\n";

    std::cout << "  Getters match underlying servos... ";
    testGettersMatchUnderlyingServos();
    std::cout << "PASS\n";

    std::cout << "\nAll PanTiltSim tests passed!\n";
}
