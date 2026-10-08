#include "TestCheck.h"
#include <iostream>

#include "td/sensors/Ray.h"
#include "td/sensors/Segment.h"

void testRayNormalizesDirection(){
    Ray ray{ Vec2(0.0f, 0.0f), Vec2(2.0f, 0.0f) };
    CHECK(ray.direction().nearlyEqual(Vec2(1.0f, 0.0f)));

    Ray diagonal{ Vec2(1.0f, 1.0f), Vec2(3.0f, 4.0f) };
    CHECK(diagonal.direction().nearlyEqual(Vec2(0.6f, 0.8f)));
    CHECK(diagonal.origin().nearlyEqual(Vec2(1.0f, 1.0f)));
}

void testRayPointAtIsDistance(){
    // A wall 3 m away is reached at t = 3, whatever the input direction length.
    Ray ray{ Vec2(0.0f, 0.0f), Vec2(2.0f, 0.0f) };
    CHECK(ray.pointAt(0.0f).nearlyEqual(Vec2(0.0f, 0.0f)));
    CHECK(ray.pointAt(3.0f).nearlyEqual(Vec2(3.0f, 0.0f)));

    Ray diagonal{ Vec2(1.0f, 1.0f), Vec2(3.0f, 4.0f) };
    CHECK(diagonal.pointAt(5.0f).nearlyEqual(Vec2(4.0f, 5.0f), 1e-5f));
}

void testSegmentPointAt(){
    Segment wall{ Vec2(3.0f, -1.0f), Vec2(3.0f, 1.0f) };
    CHECK(wall.pointAt(0.0f).nearlyEqual(Vec2(3.0f, -1.0f)));
    CHECK(wall.pointAt(0.5f).nearlyEqual(Vec2(3.0f, 0.0f)));
    CHECK(wall.pointAt(1.0f).nearlyEqual(Vec2(3.0f, 1.0f)));
}

int main(){
    std::cout << "Running Raycast tests...\n";

    std::cout << "  Ray normalizes direction... ";
    testRayNormalizesDirection();
    std::cout << "PASS\n";

    std::cout << "  Ray pointAt is distance... ";
    testRayPointAtIsDistance();
    std::cout << "PASS\n";

    std::cout << "  Segment pointAt... ";
    testSegmentPointAt();
    std::cout << "PASS\n";

    std::cout << "All Raycast tests passed.\n";

    return 0;
}
