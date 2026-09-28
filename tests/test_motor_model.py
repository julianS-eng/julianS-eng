"""Tests for the steady-state induction-motor equivalent-circuit model."""

from __future__ import annotations

import math

import pytest

from imvfd_demo.motor_model import (
    EXAMPLE_MOTOR,
    MotorParameters,
    breakdown_torque_analytic,
    slip_from_speed,
    speed_from_slip,
    stator_current_a,
    synchronous_speed_rpm,
    torque_nm,
)


def test_synchronous_speed_matches_textbook_value() -> None:
    """A 4-pole, 60 Hz supply gives the well-known 1800 rpm synchronous speed."""
    assert synchronous_speed_rpm(frequency_hz=60.0, poles=4) == pytest.approx(1800.0)


@pytest.mark.parametrize("poles", [2, 4, 6, 8])
def test_speed_and_slip_are_inverse_functions(poles: int) -> None:
    """speed_from_slip and slip_from_speed round-trip for any valid slip."""
    frequency_hz = 60.0
    for slip in (0.0, 0.05, 0.5, 1.0):
        speed = speed_from_slip(frequency_hz, poles, slip)
        recovered_slip = slip_from_speed(frequency_hz, poles, speed)
        assert recovered_slip == pytest.approx(slip, abs=1e-9)


def test_torque_is_zero_at_synchronous_speed() -> None:
    """At slip = 0 (synchronous speed) no rotor current flows, so torque is 0."""
    torque = torque_nm(EXAMPLE_MOTOR, phase_voltage_v=120.0, frequency_hz=60.0, slip=0.0)
    assert torque == 0.0


def test_torque_is_positive_for_motoring_slips() -> None:
    """For 0 < s <= 1 (motoring region) torque must be positive."""
    for slip in (0.01, 0.1, 0.3, 0.6, 1.0):
        torque = torque_nm(EXAMPLE_MOTOR, phase_voltage_v=120.0, frequency_hz=60.0, slip=slip)
        assert torque > 0.0


def test_breakdown_torque_is_the_numeric_maximum_of_the_torque_curve() -> None:
    """The analytic breakdown torque must match a fine-grained numeric search."""
    phase_voltage = EXAMPLE_MOTOR.phase_voltage(EXAMPLE_MOTOR.rated_voltage_v)
    frequency_hz = EXAMPLE_MOTOR.rated_frequency_hz

    s_max_analytic, t_max_analytic = breakdown_torque_analytic(
        EXAMPLE_MOTOR, phase_voltage, frequency_hz
    )

    n_points = 200_000
    slips = [1e-4 + i * (1.0 - 1e-4) / (n_points - 1) for i in range(n_points)]
    torques = [torque_nm(EXAMPLE_MOTOR, phase_voltage, frequency_hz, s) for s in slips]
    numeric_max_idx = max(range(len(torques)), key=torques.__getitem__)

    assert slips[numeric_max_idx] == pytest.approx(s_max_analytic, abs=5e-4)
    assert torques[numeric_max_idx] == pytest.approx(t_max_analytic, rel=1e-3)


def test_breakdown_torque_reference_values() -> None:
    """Regression-pin the breakdown slip/torque of EXAMPLE_MOTOR at rated frequency.

    Cross-checked by hand against the standard Thevenin-equivalent equations
    (Rth ~= 0.591 Ohm, Xth ~= 1.075 Ohm, Vth ~= 115.2 V at 208 V / 60 Hz).
    """
    phase_voltage = EXAMPLE_MOTOR.phase_voltage(EXAMPLE_MOTOR.rated_voltage_v)
    s_max, t_max = breakdown_torque_analytic(EXAMPLE_MOTOR, phase_voltage, 60.0)
    assert s_max == pytest.approx(0.2014, abs=2e-3)
    assert t_max == pytest.approx(47.2, abs=0.5)


def test_torque_scales_with_voltage_squared() -> None:
    """Torque is proportional to V^2 in the equivalent-circuit model."""
    frequency_hz = 60.0
    slip = 0.03
    t_full = torque_nm(EXAMPLE_MOTOR, 120.0, frequency_hz, slip)
    t_half_voltage = torque_nm(EXAMPLE_MOTOR, 60.0, frequency_hz, slip)
    assert t_half_voltage == pytest.approx(t_full / 4.0, rel=1e-9)


def test_stator_current_magnitude_is_higher_at_locked_rotor_than_near_sync() -> None:
    """Locked-rotor (starting) current must exceed the near-synchronous running current."""
    frequency_hz = 60.0
    phase_voltage = EXAMPLE_MOTOR.phase_voltage(EXAMPLE_MOTOR.rated_voltage_v)
    i_start = abs(stator_current_a(EXAMPLE_MOTOR, phase_voltage, frequency_hz, slip=1.0))
    i_running = abs(stator_current_a(EXAMPLE_MOTOR, phase_voltage, frequency_hz, slip=0.03))
    assert i_start > i_running


def test_motor_parameters_rejects_odd_pole_count() -> None:
    """Pole count must be a positive even integer."""
    with pytest.raises(ValueError, match="poles"):
        MotorParameters(
            rated_voltage_v=208.0,
            rated_frequency_hz=60.0,
            poles=3,
            r1_ohm=0.6,
            x1_ohm=1.1,
            r2_ohm=0.3,
            x2_ohm=0.4,
            xm_ohm=26.0,
        )


def test_motor_parameters_rejects_non_positive_electrical_values() -> None:
    """Every electrical parameter must be strictly positive."""
    with pytest.raises(ValueError):
        MotorParameters(
            rated_voltage_v=208.0,
            rated_frequency_hz=60.0,
            poles=4,
            r1_ohm=0.0,
            x1_ohm=1.1,
            r2_ohm=0.3,
            x2_ohm=0.4,
            xm_ohm=26.0,
        )


def test_phase_voltage_conversion() -> None:
    """Line-to-neutral conversion follows the standard sqrt(3) factor."""
    assert EXAMPLE_MOTOR.phase_voltage(208.0) == pytest.approx(208.0 / math.sqrt(3.0))
