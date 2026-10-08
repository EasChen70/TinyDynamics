#include <cassert>
#include <iostream>
#include <cmath>

#include "td/Vec3.h"

void testConstructors(){
    Vec3 a;
    assert(a.nearlyEqual(Vec3(0.0f, 0.0f, 0.0f)));

    Vec3 b(1.0f, 2.0f, 3.0f);
    assert(b.nearlyEqual(Vec3(1.0f, 2.0f, 3.0f)));
}

void testArithmetic(){
    Vec3 a(1.0f, 3.0f, 5.0f);
    Vec3 b(4.0f, 4.0f, 4.0f);

    Vec3 c = a + b;
    assert(c.nearlyEqual(Vec3(5.0f, 7.0f, 9.0f)));

    c = b - a;
    assert(c.nearlyEqual(Vec3(3.0f, 1.0f, -1.0f)));

    c = a * 2.0f;
    assert(c.nearlyEqual(Vec3(2.0f, 6.0f, 10.0f)));

    c = 2.0f * a;
    assert(c.nearlyEqual(Vec3(2.0f, 6.0f, 10.0f)));

    c = b / 2.0f;
    assert(c.nearlyEqual(Vec3(2.0f, 2.0f, 2.0f)));
}

void testCompoundArithmetic(){
    Vec3 value(5.0f, 10.0f, 15.0f);

    value += Vec3(1.0f, 2.0f, 3.0f);
    assert(value.nearlyEqual(Vec3(6.0f, 12.0f, 18.0f)));

    value -= Vec3(2.0f, 3.0f, 4.0f);
    assert(value.nearlyEqual(Vec3(4.0f, 9.0f, 14.0f)));

    value *= 2.0f;
    assert(value.nearlyEqual(Vec3(8.0f, 18.0f, 28.0f)));

    value /= 2.0f;
    assert(value.nearlyEqual(Vec3(4.0f, 9.0f, 14.0f)));
}

void testMagnitude(){
    Vec3 a(2.0f, 3.0f, 6.0f);
    assert(std::abs(a.length() - 7.0f) < 1e-6f);
    assert(std::abs(a.lengthSquared() - 49.0f) < 1e-6f);
}

void testNormalization(){
    Vec3 a(2.0f, 3.0f, 6.0f);
    Vec3 normalizedA = a.normalized();
    assert(std::abs(normalizedA.length() - 1.0f) < 1e-6f);
    assert(normalizedA.nearlyEqual(Vec3(2.0f / 7.0f, 3.0f / 7.0f, 6.0f / 7.0f)));

    Vec3 zeroVec(0.0f, 0.0f, 0.0f);
    Vec3 normalizedZero = zeroVec.normalized();
    assert(normalizedZero.nearlyEqual(Vec3(0.0f, 0.0f, 0.0f)));
}

void testDot(){
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);

    float dotProduct = a.dot(b);
    assert(std::abs(dotProduct - 32.0f) < 1e-6f);
}

void testCross(){
    const Vec3 xAxis(1.0f, 0.0f, 0.0f);
    const Vec3 yAxis(0.0f, 1.0f, 0.0f);
    const Vec3 zAxis(0.0f, 0.0f, 1.0f);

    // Right-handed basis: x × y = z, y × z = x, z × x = y
    assert(xAxis.cross(yAxis).nearlyEqual(zAxis));
    assert(yAxis.cross(zAxis).nearlyEqual(xAxis));
    assert(zAxis.cross(xAxis).nearlyEqual(yAxis));

    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);
    Vec3 aCrossB = a.cross(b);
    assert(aCrossB.nearlyEqual(Vec3(-3.0f, 6.0f, -3.0f)));

    // Anticommutative: a × b = -(b × a)
    assert(aCrossB.nearlyEqual(b.cross(a) * -1.0f));

    // Result is perpendicular to both inputs
    assert(std::abs(aCrossB.dot(a)) < 1e-5f);
    assert(std::abs(aCrossB.dot(b)) < 1e-5f);
}

int main() {
    std::cout << "Running Vec3 tests...\n";

    std::cout << "  Constructors... ";
    testConstructors();
    std::cout << "PASS\n";

    std::cout << "  Arithmetic... ";
    testArithmetic();
    std::cout << "PASS\n";

    std::cout << "  Compound arithmetic... ";
    testCompoundArithmetic();
    std::cout << "PASS\n";

    std::cout << "  Magnitude... ";
    testMagnitude();
    std::cout << "PASS\n";

    std::cout << "  Normalization... ";
    testNormalization();
    std::cout << "PASS\n";

    std::cout << "  Dot... ";
    testDot();
    std::cout << "PASS\n";

    std::cout << "  Cross... ";
    testCross();
    std::cout << "PASS\n";

    std::cout << "All Vec3 tests passed.\n";

    return 0;
}
