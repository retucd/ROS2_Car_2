from dataclasses import dataclass
import math
from typing import Tuple


@dataclass(frozen=True)
class Telemetry:
    accel: Tuple[float, float, float]
    gyro: Tuple[float, float, float]
    angle: Tuple[float, float, float]
    left_speed: float
    right_speed: float
    left_distance: float
    right_distance: float


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def twist_to_wheel_speeds(
    linear: float,
    angular: float,
    wheel_separation: float,
    max_wheel_speed: float,
) -> Tuple[float, float]:
    if wheel_separation <= 0.0:
        raise ValueError('wheel_separation must be positive')
    if max_wheel_speed <= 0.0:
        raise ValueError('max_wheel_speed must be positive')

    half_track = wheel_separation / 2.0
    left = linear - angular * half_track
    right = linear + angular * half_track

    left = clamp(left, -max_wheel_speed, max_wheel_speed)
    right = clamp(right, -max_wheel_speed, max_wheel_speed)
    return left, right


def encode_wheel_command(left_speed: float, right_speed: float) -> str:
    left_direction = 1 if left_speed >= 0.0 else 0
    right_direction = 1 if right_speed >= 0.0 else 0
    return (
        f'{abs(left_speed):.3f},'
        f'{abs(right_speed):.3f},'
        f'{left_direction},{right_direction},'
    )


def stop_command() -> str:
    return '0.000,0.000,0,0,'


def parse_telemetry(line: str) -> Telemetry:
    text = line.strip()
    if not text.startswith('IMU:'):
        raise ValueError('telemetry must start with IMU:')
    if ';Car_data:' not in text:
        raise ValueError('telemetry is missing Car_data')

    imu_part, car_part = text.split(';Car_data:', 1)
    imu_values = [
        float(value)
        for value in imu_part.removeprefix('IMU:').split(',')
    ]
    car_values = [float(value) for value in car_part.split(',')]

    if len(imu_values) != 9:
        raise ValueError('IMU must contain 9 values')
    if len(car_values) != 4:
        raise ValueError('Car_data must contain 4 values')

    return Telemetry(
        accel=tuple(imu_values[0:3]),
        gyro=tuple(imu_values[3:6]),
        angle=tuple(imu_values[6:9]),
        left_speed=car_values[0],
        right_speed=car_values[1],
        left_distance=car_values[2],
        right_distance=car_values[3],
    )


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def integrate_differential_drive(
    x: float,
    y: float,
    yaw: float,
    left_delta: float,
    right_delta: float,
    wheel_separation: float,
) -> Tuple[float, float, float]:
    if wheel_separation <= 0.0:
        raise ValueError('wheel_separation must be positive')

    center_delta = (right_delta + left_delta) / 2.0
    yaw_delta = (right_delta - left_delta) / wheel_separation
    middle_yaw = yaw + yaw_delta / 2.0

    x += center_delta * math.cos(middle_yaw)
    y += center_delta * math.sin(middle_yaw)
    yaw = normalize_angle(yaw + yaw_delta)
    return x, y, yaw


def quaternion_from_rpy(
    roll: float,
    pitch: float,
    yaw: float,
) -> Tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw