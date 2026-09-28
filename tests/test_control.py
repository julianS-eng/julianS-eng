"""Tests for the scalar V/Hz control law."""

from __future__ import annotations

import pytest

from imvfd_demo.control import VfControlLaw


def test_voltage_is_zero_below_zero_frequency() -> None:
    """The drive commands no voltage at zero (or negative) frequency."""
    law = VfControlLaw(rated_voltage_v=208.0, rated_frequency_hz=60.0)
    assert law.output_voltage(0.0) == 0.0
    assert law.output_voltage(-5.0) == 0.0


def test_voltage_reaches_exactly_rated_at_rated_frequency() -> None:
    """At the rated frequency, the drive commands exactly the rated voltage."""
    law = VfControlLaw(rated_voltage_v=208.0, rated_frequency_hz=60.0, boost_voltage_v=8.0)
    assert law.output_voltage(60.0) == pytest.approx(208.0)


def test_voltage_is_clamped_above_rated_frequency() -> None:
    """Above rated frequency (field weakening) voltage saturates at rated voltage."""
    law = VfControlLaw(rated_voltage_v=208.0, rated_frequency_hz=60.0)
    assert law.output_voltage(90.0) == pytest.approx(208.0)
    assert law.output_voltage(120.0) == pytest.approx(208.0)


def test_voltage_is_linear_between_boost_and_rated() -> None:
    """Below rated frequency, voltage follows a straight line from the boost floor."""
    law = VfControlLaw(rated_voltage_v=200.0, rated_frequency_hz=50.0, boost_voltage_v=10.0)
    assert law.output_voltage(0.0) == pytest.approx(0.0)
    assert law.output_voltage(25.0) == pytest.approx(10.0 + (190.0 / 50.0) * 25.0)
    assert law.output_voltage(50.0) == pytest.approx(200.0)


def test_zero_boost_reduces_to_constant_volts_per_hertz() -> None:
    """With no boost, V/f is exactly proportional to frequency below rated."""
    law = VfControlLaw(rated_voltage_v=230.0, rated_frequency_hz=50.0, boost_voltage_v=0.0)
    for f in (10.0, 20.0, 40.0):
        ratio = law.output_voltage(f) / f
        assert ratio == pytest.approx(230.0 / 50.0)


def test_rejects_boost_voltage_at_or_above_rated_voltage() -> None:
    """A boost voltage that reaches or exceeds rated voltage is not a valid V/Hz ramp."""
    with pytest.raises(ValueError, match="boost_voltage_v"):
        VfControlLaw(rated_voltage_v=100.0, rated_frequency_hz=60.0, boost_voltage_v=100.0)


def test_rejects_non_positive_ratings() -> None:
    """rated_voltage_v and rated_frequency_hz must both be positive."""
    with pytest.raises(ValueError):
        VfControlLaw(rated_voltage_v=0.0, rated_frequency_hz=60.0)
    with pytest.raises(ValueError):
        VfControlLaw(rated_voltage_v=208.0, rated_frequency_hz=0.0)
