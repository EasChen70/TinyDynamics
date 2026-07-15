#include <cassert>
#include <iostream>

#include "td/Vec2.h"

void testConstructors(){
    Vec2 a;
    assert(a.nearlyEqual(Vec2(0.0f, 0.0f)));

    Vec2 b(1.0f, 2.0f);
    assert(b.nearlyEqual(Vec2(1.0f, 2.0f)));
}

void testArithmetic(){
    Vec2 a(1.0f, 3.0f);
    Vec2 b(4.0f, 4.0f);

    Vec2 c = a + b;
    assert(c.nearlyEqual(Vec2(5.0f, 7.0f)));

    c = b - a;
    assert(c.nearlyEqual(Vec2(3.0f, 1.0f)));

    c = a * 2.0f;
    assert(c.nearlyEqual(Vec2(2.0f, 6.0f)));

    c = b / 2.0f;
    assert(c.nearlyEqual(Vec2(2.0, 2.0f)));
}

void testCompoundArithmetic(){
    Vec2 value(5.0f, 10.0f);

    value += Vec2(1.0f, 2.0f);
    assert(value.nearlyEqual(Vec2(6.0f, 12.0f)));

    value -= Vec2(2.0f, 3.0f);
    assert(value.nearlyEqual(Vec2(4.0f, 9.0f)));

    value *= 2.0f;
    assert(value.nearlyEqual(Vec2(8.0f, 18.0f)));

    value /= 2.0f;
    assert(value.nearlyEqual(Vec2(4.0f, 9.0f)));
}

void testMagnitude(){
    Vec2 a(3.0f, 4.0f);
    assert(std::abs(a.length() - 5.0f) < 1e-6f);
    assert(std::abs(a.lengthSquared() - 25.0f) < 1e-6f);
}

void testNormalization(){
    Vec2 a(3.0f, 4.0f);
    Vec2 normalizedA = a.normalized();
    assert(std::abs(normalizedA.length() - 1.0f) < 1e-6f);

    Vec2 zeroVec(0.0f, 0.0f);
    Vec2 normalizedZero = zeroVec.normalized();
    assert(normalizedZero.nearlyEqual(Vec2(0.0f, 0.0f)));
}

void testDotAndCross(){
    Vec2 a(1.0f, 2.0f);
    Vec2 b(3.0f, 4.0f);

    float dotProduct = a.dot(b);
    assert(std::abs(dotProduct - 11.0f) < 1e-6f);

    float crossProduct = a.cross(b);
    assert(std::abs(crossProduct - (-2.0f)) < 1e-6f);
}

int main() {
    std::cout << "Running Vec2 tests...\n";

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

    std::cout << "  Dot/Cross... ";
    testDotAndCross();
    std::cout << "PASS\n";

    std::cout << "All Vec2 tests passed.\n";

    return 0;
}
