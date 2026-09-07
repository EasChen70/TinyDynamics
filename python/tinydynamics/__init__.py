"""TinyDynamics: simulated pan/tilt servo plant for LookAt.

All angles are radians, all times are seconds.

    import tinydynamics as td

    sim = td.PanTiltSim()
    sim.set_pan_target(1.0)
    sim.update(0.01)
    sim.pan_angle()
"""

from ._core import PanTiltSim, ServoActuator

__all__ = ["PanTiltSim", "ServoActuator"]
