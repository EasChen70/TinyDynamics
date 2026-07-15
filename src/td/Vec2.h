#pragma once

#include <cassert>
#include <cmath>

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    Vec2() = default;
    Vec2(float x, float y) : x(x), y(y) {}

    Vec2 operator+(const Vec2& other) const {
        return {x + other.x, y + other.y};
    }

    Vec2 operator-(const Vec2& other) const {
        return {x - other.x, y - other.y};
    }

    Vec2 operator*(float scalar) const {
        return {x * scalar, y * scalar};
    }

    Vec2 operator/(float scalar) const {
        assert(std::abs(scalar) > 1e-6f);
        return {x / scalar, y / scalar};
    }

    Vec2& operator+=(const Vec2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vec2& operator-=(const Vec2& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vec2& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    Vec2& operator/=(float scalar) {
        assert(std::abs(scalar) > 1e-6f);
        x /= scalar;
        y /= scalar;
        return *this;
    }

    float lengthSquared() const {
        return x * x + y * y;
    }

    float length() const {
        return std::sqrt(lengthSquared());
    }

    Vec2 normalized() const {
        const float len = length();

        if (len <= 1e-6f) {
            return {0.0f, 0.0f};
        }

        return *this / len;
    }

    float dot(const Vec2& other) const {
        return x * other.x + y * other.y;
    }

    float cross(const Vec2& other) const {
        return x * other.y - y * other.x;
    }

    bool nearlyEqual(const Vec2& other, float epsilon = 1e-6f) const {
        return std::abs(x - other.x) <= epsilon &&
               std::abs(y - other.y) <= epsilon;
    }
};

inline Vec2 operator*(float scalar, const Vec2& vector) {
    return vector * scalar;
}