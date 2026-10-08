# Decisions

## Ray direction is normalized at construction

**Context.** `Ray::pointAt(t)` returns `origin + direction * t`. `t` only equals
distance when `direction` has length 1. For example, with `direction = (2, 0)`,
a wall 3 m away is hit at `t = 1.5`. If Lidar reported that `t`, every reading
would be quietly off by the direction's length.

**Decision.** `Ray`'s constructor normalizes `direction`, and asserts if it is
zero. So `t` always means meters from `origin`.

**Alternative rejected.** Allow any length and have raycast convert with
`distance = t * direction.length()`. Every caller would have to remember the
conversion. A zero direction would also get through construction and only show
up as a missed hit or a 0/0 inside the intersection math.

**Consequences.**
- Raycast and Lidar can use `t` directly as range.
- A zero direction fails at the line that builds the bad ray.
- `Ray` is no longer an aggregate. `Ray{origin, dir}` still works, but it now
  calls the constructor. `Segment` stays a plain aggregate.
- `origin` and `direction` are private, with read-only getters. A public field
  would let `ray.direction = Vec2(2, 0)` break the invariant after
  construction. This is stricter than `Particle`, which only documents "use
  setMass()". Here the type itself enforces the rule.
- There is no default constructor, so `std::vector<Ray> rays(360)` won't
  compile. Build ray lists with `reserve` + `push_back`/`emplace_back`.
- The zero-direction assert is compiled out in Release (and the Python module
  is always Release). There, a zero direction quietly becomes `(0, 0)` and the
  ray hits nothing. Revisit this when `Lidar` gets Python bindings.
