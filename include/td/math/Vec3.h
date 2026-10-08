#pragma once

#include <cassert>
#include <cmath>

// 3D float vector. Same API and conventions as Vec2; see Vec2.h.
// Not yet used by any body type; intended for future 3D primitives.

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vec3() = default;
    Vec3(float x, float y, float z ) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    Vec3 operator-(const Vec3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    Vec3 operator*(float scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    Vec3 operator/(float scalar) const {
        assert(std::abs(scalar) > 1e-6f);
        return {x / scalar, y / scalar, z / scalar};
    }

    Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vec3& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Vec3& operator/=(float scalar) {
        assert(std::abs(scalar) > 1e-6f);
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }

    float lengthSquared() const {
        return x * x + y * y + z * z;
    }

    float length() const {
        return std::sqrt(lengthSquared());
    }

    // A (near-)zero vector returns zero, matching Vec2.
    Vec3 normalized() const {
        const float len = length();

        if (len <= 1e-6f) {
            return {0.0f, 0.0f, 0.0f};
        }

        return *this / len;
    }

    float dot(const Vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    // Right-handed: x.cross(y) == z.
    Vec3 cross(const Vec3& other) const {
        return {y * other.z - z * other.y,
                z * other.x - x * other.z,
                x * other.y - y * other.x};
    }

    bool nearlyEqual(const Vec3& other, float epsilon = 1e-6f) const {
        return std::abs(x - other.x) <= epsilon &&
               std::abs(y - other.y) <= epsilon &&
               std::abs(z - other.z) <= epsilon;
    }

};

inline Vec3 operator*(float scalar, const Vec3& vector) {
    return vector * scalar;
}