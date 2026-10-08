#pragma once
#include <cassert>
#include "td/math/Vec2.h"

// A half-line starting at `origin` and going along `direction`.
//
// Invariant: `direction` is always unit length (normalized in the constructor),
// so the `t` in pointAt(t) is the distance in meters from `origin`. Raycast and
// Lidar can report `t` directly as a range reading with no conversion.
//
// Why not allow any length and convert later (distance = t * |direction|)?
// It's easy to forget the conversion, and then every reading is quietly scaled.
// A zero direction would also slip through and only fail deep in the
// intersection math. Normalizing here makes a zero direction an assert at the
// point where the bad ray is built.
//
// The fields are private with read-only getters, so nothing can overwrite
// `direction` with a non-unit vector after construction. There is no default
// constructor, because a ray with no direction has no meaning. See DECISIONS.md.
class Ray {
public:
    Ray(Vec2 origin, Vec2 direction) : origin_(origin), direction_(direction.normalized()) {
        assert(direction.lengthSquared() > 1e-12f && "Ray direction must be non-zero");
    }

    Vec2 origin() const { return origin_; }
    Vec2 direction() const { return direction_; }

    Vec2 pointAt(float t) const {
        assert(t >= 0.0f);
        return origin_ + direction_ * t;
    }

private:
    Vec2 origin_;
    Vec2 direction_;
};
