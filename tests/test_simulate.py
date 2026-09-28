"""Tests for the simulation/report layer (torque-speed sweeps and Monte Carlo study)."""

from __future__ import annotations

import numpy as np
import pytest

from imvfd_demo.control import VfControlLaw
from imvfd_demo.motor_model import EXAMPLE_MOTOR
from imvfd_demo.simulate import (
    monte_carlo_breakdown_torque,
    operating_point_metrics,
    torque_speed_curve,
)

CONTROL = VfControlLaw(
    rated_voltage_v=EXAMPLE_MOTOR.rated_voltage_v,
    rated_frequency_hz=EXAMPLE_MOTOR.rated_frequency_hz,
    boost_voltage_v=8.0,
)


def test_torque_speed_curve_shape_and_monotonic_speed() -> None:
    """The curve has one point per slip sample and monotonically decreasing speed."""
    curve = torque_speed_curve(EXAMPLE_MOTOR, CONTROL, frequency_hz=60.0, n_points=50)
    assert curve.speed_rpm.shape == (50,)
    assert curve.torque_nm.shape == (50,)
    assert np.all(np.diff(curve.speed_rpm) < 0)


def test_operating_point_metrics_breakdown_speed_below_synchronous() -> None:
    """Breakdown (pull-out) speed must be strictly below synchronous speed."""
    metrics = operating_point_metrics(EXAMPLE_MOTOR, CONTROL, frequency_hz=60.0)
    assert metrics.breakdown_speed_rpm < metrics.synchronous_speed_rpm
    assert metrics.breakdown_torque_nm > metrics.starting_torque_nm


def test_monte_carlo_is_reproducible_with_same_seed() -> None:
    """Identical seeds must reproduce bit-identical Monte Carlo samples."""
    result_a = monte_carlo_breakdown_torque(
        EXAMPLE_MOTOR, CONTROL, frequency_hz=60.0, n_samples=200, seed=7
    )
    result_b = monte_carlo_breakdown_torque(
        EXAMPLE_MOTOR, CONTROL, frequency_hz=60.0, n_samples=200, seed=7
    )
    np.testing.assert_array_equal(result_a.breakdown_torques_nm, result_b.breakdown_torques_nm)


def test_monte_carlo_different_seeds_diverge() -> None:
    """Different seeds must not (in general) produce identical samples."""
    result_a = monte_carlo_breakdown_torque(
        EXAMPLE_MOTOR, CONTROL, frequency_hz=60.0, n_samples=200, seed=1
    )
    result_b = monte_carlo_breakdown_torque(
        EXAMPLE_MOTOR, CONTROL, frequency_hz=60.0, n_samples=200, seed=2
    )
    assert not np.array_equal(result_a.breakdown_torques_nm, result_b.breakdown_torques_nm)


def test_monte_carlo_tighter_tolerance_gives_smaller_spread() -> None:
    """Halving the rotor-parameter tolerance must reduce breakdown-torque spread."""
    wide = monte_carlo_breakdown_torque(
        EXAMPLE_MOTOR,
        CONTROL,
        frequency_hz=60.0,
        n_samples=2000,
        rotor_resistance_tolerance=0.20,
        rotor_reactance_tolerance=0.20,
        seed=42,
    )
    narrow = monte_carlo_breakdown_torque(
        EXAMPLE_MOTOR,
        CONTROL,
        frequency_hz=60.0,
        n_samples=2000,
        rotor_resistance_tolerance=0.05,
        rotor_reactance_tolerance=0.05,
        seed=42,
    )
    assert narrow.std_nm < wide.std_nm


def test_monte_carlo_mean_close_to_nominal_breakdown_torque() -> None:
    """The Monte Carlo mean should be close to the nominal (untoleranced) breakdown torque."""
    nominal = operating_point_metrics(EXAMPLE_MOTOR, CONTROL, frequency_hz=60.0)
    result = monte_carlo_breakdown_torque(
        EXAMPLE_MOTOR, CONTROL, frequency_hz=60.0, n_samples=5000, seed=42
    )
    assert result.mean_nm == pytest.approx(nominal.breakdown_torque_nm, rel=0.05)
