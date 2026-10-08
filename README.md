# TinyDynamics

**TinyDynamics** is a header-only C++17 library providing lightweight physics primitives, along with a simulated **MG90S pan/tilt servo unit** used as the plant for [LookAt](../LookAt).

## Features

* Header-only C++17 physics primitives
* Simulated MG90S pan/tilt servo dynamics
* Python bindings via `pybind11`
* Configurable servo velocity, deadband, and travel limits
* C++ and Python test support

> **Units:** Angles are expressed in **radians** and time in **seconds** throughout the library.

---

## C++ Tests

Configure and build the project:

```bash
cmake -S . -B build
cmake --build build --config Debug
```

Run all C++ tests:

```bash
ctest --test-dir build -C Debug --output-on-failure
```

Tests use a `CHECK` macro (`tests/TestCheck.h`) instead of `assert`, so they also verify behavior in Release builds (`-C Release`).

Run a single test executable directly:

```bash
build\Debug\test_pan_tilt_sim.exe
```

---

## Python Bindings

`PanTiltSim` and `ServoActuator` are exposed to Python through `pybind11`.

### Installation

Install into any **Python 3.9+** environment. A C++ compiler is required.

```bash
pip install .
```

To install the test dependencies as well:

```bash
pip install ".[test]"
```

### Example

```python
import tinydynamics as td

sim = td.PanTiltSim()

# Set target angles.
# Commands are clamped to each servo's travel limits.
sim.set_pan_target(1.0)
sim.set_tilt_target(0.5)

# Advance the simulation by 10 ms.
sim.update(0.01)

# Observe state.
sim.pan_angle()
sim.tilt_angle()

sim.pan_velocity()
sim.tilt_velocity()

# Tune servo parameters.
# Configuration is writable; simulation state is read-only.
sim.pan.max_angular_velocity = 5.0
sim.pan.angular_deadband = 0.02
```

## Python Tests

Run the test suite with:

```bash
python -m pytest
```
