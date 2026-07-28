import math

import pytest

from car_control_tools.goal_utils import parse_goal_text


def test_xy_input_uses_zero_yaw():
    x, y, yaw = parse_goal_text('1.2 -0.5')
    assert x == 1.2
    assert y == -0.5
    assert yaw == 0.0


def test_optional_yaw_is_converted_from_degrees():
    x, y, yaw = parse_goal_text('1.0 2.0 90')
    assert x == 1.0
    assert y == 2.0
    assert math.isclose(yaw, math.pi / 2.0)


def test_invalid_field_count_is_rejected():
    with pytest.raises(ValueError):
        parse_goal_text('1.0')