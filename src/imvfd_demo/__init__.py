"""Induction-motor / scalar V-Hz VFD demo package backing this profile README.

See :mod:`imvfd_demo.motor_model` for the physics and
:mod:`imvfd_demo.simulate` for the report/figure generation used to produce
the numbers quoted in the top-level README.
"""

from imvfd_demo.motor_model import EXAMPLE_MOTOR, MotorParameters

__all__ = ["EXAMPLE_MOTOR", "MotorParameters"]

__version__ = "0.1.0"
