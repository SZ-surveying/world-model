from __future__ import annotations

import math

import pytest

from navlab.sim.gazebo_sensor.rangefinder import mavlink_constant, meters_to_centimeters, select_down_range_m


def test_down_rangefinder_selects_nearest_valid_range() -> None:
    distance = select_down_range_m([math.inf, 4.0, 0.01, 1.2, 7.0], min_m=0.05, max_m=6.0)

    assert distance == 1.2


def test_down_rangefinder_ignores_invalid_ranges() -> None:
    distance = select_down_range_m([math.inf, math.nan, 0.01, 7.0], min_m=0.05, max_m=6.0)

    assert distance is None


def test_distance_sensor_uses_centimeter_units() -> None:
    assert meters_to_centimeters(1.234, min_m=0.05, max_m=6.0) == 123
    assert meters_to_centimeters(0.01, min_m=0.05, max_m=6.0) == 5
    assert meters_to_centimeters(7.0, min_m=0.05, max_m=6.0) == 600


def test_mavlink_orientation_must_be_known_constant() -> None:
    class Constants:
        MAV_SENSOR_ROTATION_PITCH_270 = 25

    assert mavlink_constant(Constants, "MAV_SENSOR_ROTATION_PITCH_270") == 25
    with pytest.raises(ValueError, match="Unknown MAVLink constant"):
        mavlink_constant(Constants, "NOT_A_REAL_ORIENTATION")
