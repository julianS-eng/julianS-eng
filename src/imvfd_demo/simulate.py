"""Simulation helpers built on top of :mod:`imvfd_demo.motor_model`.

This module turns the per-slip equations in ``motor_model`` into the two
studies used for the README: a torque-speed curve family swept across a
scalar V/Hz frequency ramp, and a Monte Carlo study of how breakdown torque
is affected by rotor-parameter manufacturing tolerance. All randomness goes
through :func:`numpy.random.default_rng` seeded explicitly by the caller, so
results are bit-for-bit reproducible.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from imvfd_demo.control import VfControlLaw
from imvfd_demo.motor_model import (
    MotorParameters,
    breakdown_torque_analytic,
    speed_from_slip,
    torque_nm,
)


@dataclass(frozen=True, slots=True)
class TorqueSpeedCurve:
    """A single torque-speed curve at a fixed drive frequency."""

    frequency_hz: float
    speed_rpm: NDArray[np.float64]
    torque_nm: NDArray[np.float64]


def torque_speed_curve(
    motor: MotorParameters,
    control: VfControlLaw,
    frequency_hz: float,
    n_points: int = 400,
    slip_max: float = 1.0,
    slip_min: float = 1e-4,
) -> TorqueSpeedCurve:
    """Sweep slip from near-zero to ``slip_max`` and return the resulting torque curve."""
    slips = np.linspace(slip_min, slip_max, n_points)
    voltage_line = control.output_voltage(frequency_hz)
    voltage_phase = motor.phase_voltage(voltage_line)
    torques = np.array([torque_nm(motor, voltage_phase, frequency_hz, float(s)) for s in slips])
    speeds = np.array([speed_from_slip(frequency_hz, motor.poles, float(s)) for s in slips])
    return TorqueSpeedCurve(frequency_hz=frequency_hz, speed_rpm=speeds, torque_nm=torques)


@dataclass(frozen=True, slots=True)
class OperatingPointMetrics:
    """Key scalar metrics describing a motor driven at a single frequency."""

    frequency_hz: float
    line_voltage_v: float
    synchronous_speed_rpm: float
    starting_torque_nm: float
    breakdown_slip: float
    breakdown_torque_nm: float
    breakdown_speed_rpm: float


def operating_point_metrics(
    motor: MotorParameters, control: VfControlLaw, frequency_hz: float
) -> OperatingPointMetrics:
    """Compute starting torque and breakdown (pull-out) torque/speed at a frequency."""
    voltage_line = control.output_voltage(frequency_hz)
    voltage_phase = motor.phase_voltage(voltage_line)
    starting_torque = torque_nm(motor, voltage_phase, frequency_hz, slip=1.0)
    s_max, t_max = breakdown_torque_analytic(motor, voltage_phase, frequency_hz)
    n_sync = 120.0 * frequency_hz / motor.poles
    breakdown_speed = speed_from_slip(frequency_hz, motor.poles, s_max)
    return OperatingPointMetrics(
        frequency_hz=frequency_hz,
        line_voltage_v=voltage_line,
        synchronous_speed_rpm=n_sync,
        starting_torque_nm=starting_torque,
        breakdown_slip=s_max,
        breakdown_torque_nm=t_max,
        breakdown_speed_rpm=breakdown_speed,
    )


@dataclass(frozen=True, slots=True)
class SensitivityResult:
    """Result of a Monte Carlo breakdown-torque sensitivity study."""

    frequency_hz: float
    n_samples: int
    seed: int
    breakdown_torques_nm: NDArray[np.float64]

    @property
    def mean_nm(self) -> float:
        """Sample mean of breakdown torque across the Monte Carlo draws."""
        return float(np.mean(self.breakdown_torques_nm))

    @property
    def std_nm(self) -> float:
        """Sample standard deviation of breakdown torque across the Monte Carlo draws."""
        return float(np.std(self.breakdown_torques_nm, ddof=1))

    @property
    def coefficient_of_variation_pct(self) -> float:
        """Relative spread of breakdown torque, as 100 * std / mean."""
        return 100.0 * self.std_nm / self.mean_nm


def monte_carlo_breakdown_torque(
    motor: MotorParameters,
    control: VfControlLaw,
    frequency_hz: float,
    n_samples: int = 5000,
    rotor_resistance_tolerance: float = 0.10,
    rotor_reactance_tolerance: float = 0.10,
    seed: int = 42,
) -> SensitivityResult:
    """Propagate rotor-parameter manufacturing tolerance into breakdown torque.

    Rotor resistance and leakage reactance are the equivalent-circuit
    parameters most affected by manufacturing tolerance and rotor
    temperature; ``rotor_resistance_tolerance`` / ``rotor_reactance_tolerance``
    are treated as the half-width of a uniform distribution around the
    nameplate value (e.g. 0.10 means R2 is drawn uniformly from
    [0.9, 1.1] * nameplate R2). The draw uses a seeded NumPy generator so
    the study is exactly reproducible across runs and machines.
    """
    rng = np.random.default_rng(seed)
    r2_samples = motor.r2_ohm * rng.uniform(
        1.0 - rotor_resistance_tolerance, 1.0 + rotor_resistance_tolerance, size=n_samples
    )
    x2_samples = motor.x2_ohm * rng.uniform(
        1.0 - rotor_reactance_tolerance, 1.0 + rotor_reactance_tolerance, size=n_samples
    )
    voltage_line = control.output_voltage(frequency_hz)
    voltage_phase = motor.phase_voltage(voltage_line)

    torques = np.empty(n_samples)
    for i in range(n_samples):
        sample_motor = MotorParameters(
            rated_voltage_v=motor.rated_voltage_v,
            rated_frequency_hz=motor.rated_frequency_hz,
            poles=motor.poles,
            r1_ohm=motor.r1_ohm,
            x1_ohm=motor.x1_ohm,
            r2_ohm=float(r2_samples[i]),
            x2_ohm=float(x2_samples[i]),
            xm_ohm=motor.xm_ohm,
        )
        _, t_max = breakdown_torque_analytic(sample_motor, voltage_phase, frequency_hz)
        torques[i] = t_max

    return SensitivityResult(
        frequency_hz=frequency_hz,
        n_samples=n_samples,
        seed=seed,
        breakdown_torques_nm=torques,
    )
