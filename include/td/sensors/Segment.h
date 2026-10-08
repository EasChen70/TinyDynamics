#pragma once
#include "td/math/Vec2.h"
#include <cassert>

struct Segment {
    Vec2 start;
    Vec2 end;

    Vec2 pointAt(float u) const {
        assert(u >= 0.0f && u <= 1.0f);
        return start + (end - start) * u;
    }
};
