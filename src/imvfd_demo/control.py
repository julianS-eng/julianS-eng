"""Open-loop scalar V/Hz (V/f) control law used by a basic induction-motor VFD.

Design decision: this repository demonstrates *scalar* V/f control rather
than field-oriented (vector) control. Scalar control needs no shaft encoder
and is the control strategy implemented on the LabVolt/FESTO variable
frequency drive used for the author's thesis bench work; vector control was
considered and set aside for this demo because it requires rotor flux
estimation or a speed sensor and closed-loop current control that are out
of scope for a steady-state, per-phase equivalent-circuit study. See
``docs/LEARNING.md`` for the full comparison.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class VfControlLaw:
    """Linear volts-per-hertz control law with an optional low-speed voltage boost.

    Below ``rated_frequency_hz`` the output voltage follows a straight line
    from ``boost_voltage_v`` at 0 Hz up to ``rated_voltage_v`` at
    ``rated_frequency_hz``, keeping the air-gap flux approximately constant.
    ``boost_voltage_v`` compensates for the stator IR drop, which would
    otherwise starve the machine of torque at low speed where the nominal
    V/Hz ratio yields very little voltage. Above rated frequency the drive
    cannot exceed the supply/rated voltage, so voltage is held constant
    (field-weakening region) and flux -- and therefore available torque --
    fall off.
    """

    rated_voltage_v: float
    rated_frequency_hz: float
    boost_voltage_v: float = 0.0

    def __post_init__(self) -> None:
        """Validate that the control law parameters describe a sane drive."""
        if self.rated_voltage_v <= 0 or self.rated_frequency_hz <= 0:
            raise ValueError("rated_voltage_v and rated_frequency_hz must be positive")
        if not 0.0 <= self.boost_voltage_v < self.rated_voltage_v:
            raise ValueError("boost_voltage_v must be in [0, rated_voltage_v)")

    def output_voltage(self, frequency_hz: float) -> float:
        """Return the commanded line-to-line RMS voltage (V) for a given frequency."""
        if frequency_hz <= 0.0:
            return 0.0
        if frequency_hz >= self.rated_frequency_hz:
            return self.rated_voltage_v
        slope = (self.rated_voltage_v - self.boost_voltage_v) / self.rated_frequency_hz
        return self.boost_voltage_v + slope * frequency_hz
