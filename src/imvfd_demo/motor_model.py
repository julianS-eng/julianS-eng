"""Steady-state per-phase equivalent-circuit model of a 3-phase induction motor.

The model implements the standard IEEE / Thevenin-equivalent steady-state
torque equation for a squirrel-cage induction machine (see e.g. Chapman,
*Electric Machinery Fundamentals*, ch. 6-7). It is a textbook analytical
model, not a fit to a specific physical machine: :data:`EXAMPLE_MOTOR` holds
illustrative nameplate-style parameters representative of a small
industrial NEMA-Design-B motor (the same class of machine used on a
LabVolt/FESTO electrical-machines training bench), used here to produce
reproducible, checkable numbers for the README and tests -- not the
proprietary experimental data from the author's thesis bench.

Simplifications made explicit for the reader:

* Rotor resistance ``r2_ohm`` is treated as constant with slip (skin effect
  in the rotor bars, which raises effective R2 at high slip / low speed,
  is ignored).
* Core losses, friction and windage are not modelled; the magnetizing
  branch is purely reactive.
* Stator and rotor leakage reactances are assumed to scale linearly with
  electrical frequency (``X = X_rated * f / f_rated``), as is standard
  practice for open-loop scalar V/Hz drives.
"""

from __future__ import annotations

import cmath
import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class MotorParameters:
    """Per-phase equivalent-circuit parameters of a 3-phase squirrel-cage motor.

    All resistances/reactances are per-phase values referred to the stator,
    in ohms, specified at ``rated_frequency_hz``.
    """

    rated_voltage_v: float
    """Rated line-to-line RMS voltage (V)."""

    rated_frequency_hz: float
    """Rated electrical supply frequency (Hz)."""

    poles: int
    """Number of magnetic poles (even integer, e.g. 4)."""

    r1_ohm: float
    """Stator winding resistance per phase (Ohm)."""

    x1_ohm: float
    """Stator leakage reactance per phase at rated frequency (Ohm)."""

    r2_ohm: float
    """Rotor resistance per phase referred to the stator (Ohm)."""

    x2_ohm: float
    """Rotor leakage reactance per phase referred to the stator at rated frequency (Ohm)."""

    xm_ohm: float
    """Magnetizing reactance per phase at rated frequency (Ohm)."""

    def __post_init__(self) -> None:
        """Validate that all parameters describe a physically sane machine."""
        if self.poles < 2 or self.poles % 2 != 0:
            raise ValueError(f"poles must be a positive even integer, got {self.poles}")
        for name in (
            "rated_voltage_v",
            "rated_frequency_hz",
            "r1_ohm",
            "x1_ohm",
            "r2_ohm",
            "x2_ohm",
            "xm_ohm",
        ):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be strictly positive")

    def phase_voltage(self, line_voltage_v: float) -> float:
        """Convert a line-to-line RMS voltage to a line-to-neutral (phase) RMS voltage."""
        return line_voltage_v / math.sqrt(3.0)


EXAMPLE_MOTOR = MotorParameters(
    rated_voltage_v=208.0,
    rated_frequency_hz=60.0,
    poles=4,
    r1_ohm=0.641,
    x1_ohm=1.106,
    r2_ohm=0.332,
    x2_ohm=0.464,
    xm_ohm=26.3,
)
"""Illustrative 208 V, 60 Hz, 4-pole induction-motor equivalent-circuit parameters.

Values are a widely used textbook equivalent-circuit example (order of
magnitude of a small few-kW 3-phase induction motor, the same class of
machine as a LabVolt/FESTO electrical-machines training bench) and are
used throughout this repository's demo/report code to produce reproducible
figures. They are *not* a claim about, or a substitute for, the specific
machine or the experimental data used in the author's thesis.
"""


def synchronous_speed_rpm(frequency_hz: float, poles: int) -> float:
    """Return the synchronous mechanical speed in rpm for a given supply frequency."""
    return 120.0 * frequency_hz / poles


def slip_from_speed(frequency_hz: float, poles: int, mechanical_speed_rpm: float) -> float:
    """Return the slip for a given rotor mechanical speed at the given supply frequency."""
    n_sync = synchronous_speed_rpm(frequency_hz, poles)
    return (n_sync - mechanical_speed_rpm) / n_sync


def speed_from_slip(frequency_hz: float, poles: int, slip: float) -> float:
    """Return the rotor mechanical speed in rpm for a given slip and supply frequency."""
    n_sync = synchronous_speed_rpm(frequency_hz, poles)
    return n_sync * (1.0 - slip)


def _scaled_reactance(x_rated_ohm: float, frequency_hz: float, rated_frequency_hz: float) -> float:
    """Scale a reactance linearly with electrical frequency, as X = 2*pi*f*L."""
    return x_rated_ohm * frequency_hz / rated_frequency_hz


def thevenin_equivalent(
    motor: MotorParameters, phase_voltage_v: float, frequency_hz: float
) -> tuple[complex, complex]:
    """Compute the Thevenin equivalent (voltage, impedance) seen by the rotor branch.

    Returns ``(v_th, z_th)`` where ``v_th`` is a phasor (V) and ``z_th`` an
    impedance (Ohm), both referred to the stator at the given electrical
    frequency.
    """
    x1 = _scaled_reactance(motor.x1_ohm, frequency_hz, motor.rated_frequency_hz)
    xm = _scaled_reactance(motor.xm_ohm, frequency_hz, motor.rated_frequency_hz)
    z1 = complex(motor.r1_ohm, x1)
    zm = complex(0.0, xm)
    z_th = (z1 * zm) / (z1 + zm)
    v_th = phase_voltage_v * zm / (z1 + zm)
    return v_th, z_th


def torque_nm(
    motor: MotorParameters,
    phase_voltage_v: float,
    frequency_hz: float,
    slip: float,
) -> float:
    """Return the steady-state electromagnetic torque (N*m) at the given slip.

    Uses the Thevenin-equivalent torque equation. ``slip == 0`` returns 0
    N*m (no rotor current at synchronous speed); very small negative
    denominators (slip -> 0 from the braking region) are handled naturally
    since the equation is continuous there.
    """
    if slip == 0.0:
        return 0.0
    v_th, z_th = thevenin_equivalent(motor, phase_voltage_v, frequency_hz)
    r_th, x_th = z_th.real, z_th.imag
    x2 = _scaled_reactance(motor.x2_ohm, frequency_hz, motor.rated_frequency_hz)
    r2_over_s = motor.r2_ohm / slip
    denom = (r_th + r2_over_s) ** 2 + (x_th + x2) ** 2
    n_sync_rpm = synchronous_speed_rpm(frequency_hz, motor.poles)
    omega_sync = n_sync_rpm * 2.0 * math.pi / 60.0
    v_th_mag = abs(v_th)
    return (3.0 * v_th_mag**2 * r2_over_s) / (omega_sync * denom)


def breakdown_torque_analytic(
    motor: MotorParameters, phase_voltage_v: float, frequency_hz: float
) -> tuple[float, float]:
    """Return the analytic (breakdown slip, breakdown torque) at the given frequency.

    Closed-form maximum of :func:`torque_nm` with respect to slip, used both
    to report the pull-out torque of the machine and as a numerical check
    against the general torque equation in the test suite.
    """
    v_th, z_th = thevenin_equivalent(motor, phase_voltage_v, frequency_hz)
    r_th, x_th = z_th.real, z_th.imag
    x2 = _scaled_reactance(motor.x2_ohm, frequency_hz, motor.rated_frequency_hz)
    z_mag = math.sqrt(r_th**2 + (x_th + x2) ** 2)
    s_max = motor.r2_ohm / z_mag
    n_sync_rpm = synchronous_speed_rpm(frequency_hz, motor.poles)
    omega_sync = n_sync_rpm * 2.0 * math.pi / 60.0
    v_th_mag = abs(v_th)
    t_max = (3.0 * v_th_mag**2) / (2.0 * omega_sync * (r_th + z_mag))
    return s_max, t_max


def stator_current_a(
    motor: MotorParameters,
    phase_voltage_v: float,
    frequency_hz: float,
    slip: float,
) -> complex:
    """Return the per-phase stator current phasor (A) at the given slip."""
    x1 = _scaled_reactance(motor.x1_ohm, frequency_hz, motor.rated_frequency_hz)
    x2 = _scaled_reactance(motor.x2_ohm, frequency_hz, motor.rated_frequency_hz)
    xm = _scaled_reactance(motor.xm_ohm, frequency_hz, motor.rated_frequency_hz)
    z1 = complex(motor.r1_ohm, x1)
    zm = complex(0.0, xm)
    slip_safe = slip if slip != 0.0 else 1e-9
    z2 = complex(motor.r2_ohm / slip_safe, x2)
    v_phase = complex(phase_voltage_v, 0.0)
    z_rotor_parallel = (zm * z2) / (zm + z2)
    z_total = z1 + z_rotor_parallel
    return v_phase / z_total


def phasor_magnitude(value: complex) -> float:
    """Return the magnitude of a complex phasor; a thin, typed wrapper around abs()."""
    return cmath.polar(value)[0]
