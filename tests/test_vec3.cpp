#include "TestCheck.h"
#include <iostream>
#include <cmath>

#include "td/Vec3.h"

void testConstructors(){
    Vec3 a;
    CHECK(a.nearlyEqual(Vec3(0.0f, 0.0f, 0.0f)));

    Vec3 b(1.0f, 2.0f, 3.0f);
    CHECK(b.nearlyEqual(Vec3(1.0f, 2.0f, 3.0f)));
}

void testArithmetic(){
    Vec3 a(1.0f, 3.0f, 5.0f);
    Vec3 b(4.0f, 4.0f, 4.0f);

    Vec3 c = a + b;
    CHECK(c.nearlyEqual(Vec3(5.0f, 7.0f, 9.0f)));

    c = b - a;
    CHECK(c.nearlyEqual(Vec3(3.0f, 1.0f, -1.0f)));

    c = a * 2.0f;
    CHECK(c.nearlyEqual(Vec3(2.0f, 6.0f, 10.0f)));

    c = 2.0f * a;
    CHECK(c.nearlyEqual(Vec3(2.0f, 6.0f, 10.0f)));

    c = b / 2.0f;
    CHECK(c.nearlyEqual(Vec3(2.0f, 2.0f, 2.0f)));
}

void testCompoundArithmetic(){
    Vec3 value(5.0f, 10.0f, 15.0f);

    value += Vec3(1.0f, 2.0f, 3.0f);
    CHECK(value.nearlyEqual(Vec3(6.0f, 12.0f, 18.0f)));

    value -= Vec3(2.0f, 3.0f, 4.0f);
    CHECK(value.nearlyEqual(Vec3(4.0f, 9.0f, 14.0f)));

    value *= 2.0f;
    CHECK(value.nearlyEqual(Vec3(8.0f, 18.0f, 28.0f)));

    value /= 2.0f;
    CHECK(value.nearlyEqual(Vec3(4.0f, 9.0f, 14.0f)));
}

void testMagnitude(){
    Vec3 a(2.0f, 3.0f, 6.0f);
    CHECK(std::abs(a.length() - 7.0f) < 1e-6f);
    CHECK(std::abs(a.lengthSquared() - 49.0f) < 1e-6f);
}

void testNormalization(){
    Vec3 a(2.0f, 3.0f, 6.0f);
    Vec3 normalizedA = a.normalized();
    CHECK(std::abs(normalizedA.length() - 1.0f) < 1e-6f);
    CHECK(normalizedA.nearlyEqual(Vec3(2.0f / 7.0f, 3.0f / 7.0f, 6.0f / 7.0f)));

    Vec3 zeroVec(0.0f, 0.0f, 0.0f);
    Vec3 normalizedZero = zeroVec.normalized();
    CHECK(normalizedZero.nearlyEqual(Vec3(0.0f, 0.0f, 0.0f)));
}

void testDot(){
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);

    float dotProduct = a.dot(b);
    CHECK(std::abs(dotProduct - 32.0f) < 1e-6f);
}

void testCross(){
    const Vec3 xAxis(1.0f, 0.0f, 0.0f);
    const Vec3 yAxis(0.0f, 1.0f, 0.0f);
    const Vec3 zAxis(0.0f, 0.0f, 1.0f);

    // Right-handed basis: x × y = z, y × z = x, z × x = y
    CHECK(xAxis.cross(yAxis).nearlyEqual(zAxis));
    CHECK(yAxis.cross(zAxis).nearlyEqual(xAxis));
    CHECK(zAxis.cross(xAxis).nearlyEqual(yAxis));

    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);
    Vec3 aCrossB = a.cross(b);
    CHECK(aCrossB.nearlyEqual(Vec3(-3.0f, 6.0f, -3.0f)));

    // Anticommutative: a × b = -(b × a)
    CHECK(aCrossB.nearlyEqual(b.cross(a) * -1.0f));

    // Result is perpendicular to both inputs
    CHECK(std::abs(aCrossB.dot(a)) < 1e-5f);
    CHECK(std::abs(aCrossB.dot(b)) < 1e-5f);
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
