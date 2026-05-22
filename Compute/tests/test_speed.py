"""
Tests for speed.Speed.

The class stores speed in m/s and converts on the way in/out. Each unit
gets a round-trip test (set then get returns the original), plus
cross-unit checks against the gearbox/wheel/pole-pair relationships
defined in constants.py.
"""

import math

import pytest
from constants import GEAR_RATIO, MOTOR_POLE_PAIRS, WHEEL_RADIUS_M
from speed import Speed

MPH_PER_M_S = 2.23694


# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
def test_default_speed_is_zero():
    s = Speed()
    assert s.get_speed_m_s() == 0.0
    assert s.get_mph() == 0.0
    assert s.get_wheel_rpm() == 0.0
    assert s.get_motor_rpm() == 0.0
    assert s.get_motor_erpm() == 0.0


# ---------------------------------------------------------------------------
# Round-trips: set in unit X, read back in unit X
# ---------------------------------------------------------------------------
class TestRoundTrips:
    @pytest.mark.parametrize("v", [0.0, 1.0, 27.7, 100.0, -5.0])
    def test_m_s(self, v):
        s = Speed()
        s.set_speed_m_s(v)
        assert s.get_speed_m_s() == pytest.approx(v)

    @pytest.mark.parametrize("mph", [0.0, 1.0, 60.0, 150.0, -20.0])
    def test_mph(self, mph):
        s = Speed()
        s.set_mph(mph)
        assert s.get_mph() == pytest.approx(mph)

    @pytest.mark.parametrize("wheel_rpm", [0.0, 50.0, 500.0, 1500.0])
    def test_wheel_rpm(self, wheel_rpm):
        s = Speed()
        s.set_wheel_rpm(wheel_rpm)
        assert s.get_wheel_rpm() == pytest.approx(wheel_rpm)

    @pytest.mark.parametrize("motor_rpm", [0.0, 1000.0, 10_000.0, 20_000.0])
    def test_motor_rpm(self, motor_rpm):
        s = Speed()
        s.set_motor_rpm(motor_rpm)
        assert s.get_motor_rpm() == pytest.approx(motor_rpm)

    @pytest.mark.parametrize("erpm", [0.0, 4000.0, 40_000.0, 80_000.0])
    def test_motor_erpm(self, erpm):
        s = Speed()
        s.set_motor_erpm(erpm)
        assert s.get_motor_erpm() == pytest.approx(erpm)


# ---------------------------------------------------------------------------
# Known-value spot checks: catches a flipped constant or wrong formula
# that round-trips would miss.
# ---------------------------------------------------------------------------
class TestKnownValues:
    def test_one_m_s_to_mph(self):
        s = Speed()
        s.set_speed_m_s(1.0)
        assert s.get_mph() == pytest.approx(MPH_PER_M_S, rel=1e-6)

    def test_one_m_s_to_wheel_rpm(self):
        # wheel_rad_s = v / r ; wheel_rpm = rad_s * 60 / (2π)
        expected = (1.0 / WHEEL_RADIUS_M) * 60 / (2 * math.pi)
        s = Speed()
        s.set_speed_m_s(1.0)
        assert s.get_wheel_rpm() == pytest.approx(expected)

    def test_motor_rpm_is_wheel_rpm_times_gear_ratio(self):
        s = Speed()
        s.set_wheel_rpm(100.0)
        assert s.get_motor_rpm() == pytest.approx(100.0 * GEAR_RATIO)

    def test_motor_erpm_is_motor_rpm_times_pole_pairs(self):
        s = Speed()
        s.set_motor_rpm(1000.0)
        assert s.get_motor_erpm() == pytest.approx(1000.0 * MOTOR_POLE_PAIRS)

    def test_set_motor_erpm_divides_by_pole_pairs(self):
        s = Speed()
        s.set_motor_erpm(4000.0)
        assert s.get_motor_rpm() == pytest.approx(4000.0 / MOTOR_POLE_PAIRS)


# ---------------------------------------------------------------------------
# Cross-unit consistency: setting in one unit and reading in another must
# go through the same internal m/s.
# ---------------------------------------------------------------------------
class TestCrossUnit:
    def test_set_mph_then_read_m_s(self):
        s = Speed()
        s.set_mph(60.0)
        assert s.get_speed_m_s() == pytest.approx(60.0 / MPH_PER_M_S)

    def test_set_wheel_rpm_then_read_motor_erpm(self):
        s = Speed()
        s.set_wheel_rpm(100.0)
        # 100 wheel-rpm → 100*GEAR_RATIO motor-rpm → *POLE_PAIRS erpm
        assert s.get_motor_erpm() == pytest.approx(
            100.0 * GEAR_RATIO * MOTOR_POLE_PAIRS
        )

    def test_set_motor_erpm_then_read_m_s(self):
        s = Speed()
        s.set_motor_erpm(40_000.0)
        # erpm → rpm → wheel_rpm → rad/s → m/s
        motor_rpm = 40_000.0 / MOTOR_POLE_PAIRS
        wheel_rpm = motor_rpm / GEAR_RATIO
        wheel_rad_s = wheel_rpm * 2 * math.pi / 60
        assert s.get_speed_m_s() == pytest.approx(wheel_rad_s * WHEEL_RADIUS_M)

    def test_overwrite_unit_replaces_state(self):
        """Setting via a second unit must replace, not accumulate."""
        s = Speed()
        s.set_mph(60.0)
        s.set_speed_m_s(0.0)
        assert s.get_speed_m_s() == 0.0
        assert s.get_mph() == 0.0


# ---------------------------------------------------------------------------
# Sign handling — the inverter can spin in reverse, so negative speeds
# must round-trip without being clipped or absolute-valued.
# ---------------------------------------------------------------------------
class TestNegative:
    def test_negative_m_s_preserved(self):
        s = Speed()
        s.set_speed_m_s(-12.5)
        assert s.get_speed_m_s() == pytest.approx(-12.5)

    def test_negative_motor_erpm_preserved(self):
        s = Speed()
        s.set_motor_erpm(-8000.0)
        assert s.get_motor_erpm() == pytest.approx(-8000.0)
