# TinyDynamics

**TinyDynamics** is a header-only C++17 library of lightweight physics primitives, plus a simulated **MG90S pan/tilt servo unit** used as the plant for [LookAt](../LookAt). The servo simulation is also available from Python through `pybind11`.

> **Units:** angles are in **radians** and time is in **seconds** throughout.

---

## What's Inside

Headers live in `include/td/`, grouped by layer: `math/`, `physics/`, `actuators/`.

| Header | Type | Purpose |
|---|---|---|
| `math/Vec2.h` / `math/Vec3.h` | `Vec2`, `Vec3` | Float vector math: arithmetic, length, normalize, dot, cross |
| `physics/Particle.h` | `Particle` | 2D point mass with force accumulation |
| `physics/RotationalBody.h` | `RotationalBody` | Single-axis rigid body with torque accumulation |
| `physics/Damping.h` | `AngularDamping` | Viscous damping torque (τ = −c·ω) for a `RotationalBody` |
| `actuators/ServoActuator.h` | `ServoActuator` | Kinematic MG90S model: rate-limited, clamped, with deadband |
| `actuators/PanTiltSim.h` | `PanTiltSim` | Two `ServoActuator`s combined into a pan/tilt unit |

**Simulation loop:** bodies collect forces or torques, then `integrate(dt)` advances state with semi-implicit Euler and clears them. Apply every force, damping included, *before* calling `integrate`.

---

## C++ Usage

Link the `tinydynamics` CMake target, or add `include/` to your include path:

```cmake
add_subdirectory(TinyDynamics)
target_link_libraries(my_app PRIVATE tinydynamics)
```

```cpp
#include "td/physics/Damping.h"
#include "td/actuators/PanTiltSim.h"
#include "td/physics/Particle.h"
#include "td/physics/RotationalBody.h"

// Point mass under gravity
Particle ball(2.0f);                       // 2 kg at the origin
for (int i = 0; i < 100; ++i) {
    ball.applyForce(Vec2(0.0f, -9.81f) * ball.mass);
    ball.integrate(0.01f);                 // integrate() clears forces
}

// Spinning body slowed by damping
RotationalBody wheel(0.5f, 0.0f, 10.0f);   // inertia, angle, angular velocity
AngularDamping drag(0.2f);
for (int i = 0; i < 100; ++i) {
    drag.apply(wheel);                     // apply torques before integrate()
    wheel.integrate(0.01f);
}

// Pan/tilt servo unit
PanTiltSim sim;
sim.setPanTarget(1.0f);                    // clamped to [0, π]
for (int i = 0; i < 100; ++i) sim.update(0.01f);
float pan = sim.panAngle();
```

Invalid inputs fail with `assert` in debug builds: non-positive mass, inertia or `dt`, a negative damping coefficient, or division by a near-zero scalar.

---

## C++ Tests

```bash
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

* Tests use a `CHECK` macro (`tests/TestCheck.h`) instead of `assert`, so they also run in Release builds (`--config Release` / `-C Release`).
* To run one test directly: `build/Debug/test_pan_tilt_sim` (on Windows, `build\Debug\test_pan_tilt_sim.exe`).
* **To add a test:** create `tests/test_<name>.cpp`, then add `<name>` to `TD_TESTS` in `CMakeLists.txt`.

---

## Python Bindings

`PanTiltSim` and `ServoActuator` are exposed as the `tinydynamics` package. You need **Python 3.9+** and a C++ compiler.

```bash
pip install .            # library only
pip install ".[test]"    # plus pytest
```

```python
import tinydynamics as td

sim = td.PanTiltSim()

sim.set_pan_target(1.0)       # clamped to each servo's travel limits
sim.set_tilt_target(0.5)
sim.update(0.01)              # advance 10 ms

sim.pan_angle(), sim.tilt_angle()
sim.pan_velocity(), sim.tilt_velocity()

# Configuration is writable; simulation state is read-only.
sim.pan.max_angular_velocity = 5.0
sim.pan.angular_deadband = 0.02
```

Run the Python tests:

```bash
python -m pytest
```

**To bind a new type:** add `python/src/bind_<type>.cpp`, declare its function in `python/src/bindings.h`, call it from `module.cpp`, add the file to `python/CMakeLists.txt`, and export the name from `python/tinydynamics/__init__.py`.
