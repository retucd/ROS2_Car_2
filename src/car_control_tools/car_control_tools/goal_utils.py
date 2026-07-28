import math
from typing import Tuple


def parse_goal_text(text: str) -> Tuple[float, float, float]:
    values = text.split()
    if len(values) not in (2, 3):
        raise ValueError('������ x y �� x y yaw�Ƕ�')

    try:
        x = float(values[0])
        y = float(values[1])
        yaw_degrees = float(values[2]) if len(values) == 3 else 0.0
    except ValueError as error:
        raise ValueError('����ͽǶȱ���������') from error

    return x, y, math.radians(yaw_degrees)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)