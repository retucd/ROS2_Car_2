#!/usr/bin/env python3
import math
import time
from typing import Optional

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
import serial
from tf2_ros import TransformBroadcaster

from car_base_driver.kinematics import (
    encode_wheel_command,
    integrate_differential_drive,
    parse_telemetry,
    quaternion_from_rpy,
    stop_command,
    twist_to_wheel_speeds,
)


class SerialDriver(Node):
    def __init__(self) -> None:
        super().__init__('car_base_driver')

        self.declare_parameter('port', '/dev/ttyS3')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('send_rate', 20.0)
        self.declare_parameter('reconnect_interval', 2.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('wheel_separation', 0.1497)
        self.declare_parameter('wheel_radius', 0.024)
        self.declare_parameter('max_wheel_speed', 0.8)
        self.declare_parameter('distance_jump_threshold', 0.20)
        self.declare_parameter('left_command_sign', 1.0)
        self.declare_parameter('right_command_sign', 1.0)
        self.declare_parameter('left_feedback_sign', 1.0)
        self.declare_parameter('right_feedback_sign', 1.0)
        self.declare_parameter('acceleration_scale', 9.80665)
        self.declare_parameter('gyro_scale', math.pi / 180.0)
        self.declare_parameter('angle_scale', math.pi / 180.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('imu_frame', 'base_link')
        self.declare_parameter('odom_topic', '/wheel/odom')
        self.declare_parameter('publish_odom_tf', False)

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.send_rate = float(self.get_parameter('send_rate').value)
        self.reconnect_interval = float(
            self.get_parameter('reconnect_interval').value
        )
        self.cmd_vel_timeout = float(
            self.get_parameter('cmd_vel_timeout').value
        )
        self.wheel_separation = float(
            self.get_parameter('wheel_separation').value
        )
        self.wheel_radius = float(
            self.get_parameter('wheel_radius').value
        )
        self.max_wheel_speed = float(
            self.get_parameter('max_wheel_speed').value
        )
        self.distance_jump_threshold = float(
            self.get_parameter('distance_jump_threshold').value
        )
        self.left_command_sign = float(
            self.get_parameter('left_command_sign').value
        )
        self.right_command_sign = float(
            self.get_parameter('right_command_sign').value
        )
        self.left_feedback_sign = float(
            self.get_parameter('left_feedback_sign').value
        )
        self.right_feedback_sign = float(
            self.get_parameter('right_feedback_sign').value
        )
        self.acceleration_scale = float(
            self.get_parameter('acceleration_scale').value
        )
        self.gyro_scale = float(
            self.get_parameter('gyro_scale').value
        )
        self.angle_scale = float(
            self.get_parameter('angle_scale').value
        )
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.imu_frame = str(self.get_parameter('imu_frame').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.publish_odom_tf = bool(
            self.get_parameter('publish_odom_tf').value
        )

        if self.send_rate <= 0.0:
            raise ValueError('send_rate must be positive')
        if self.wheel_separation <= 0.0:
            raise ValueError('wheel_separation must be positive')
        if self.wheel_radius <= 0.0:
            raise ValueError('wheel_radius must be positive')

        self.serial_port: Optional[serial.Serial] = None
        self.rx_buffer = bytearray()
        self.next_reconnect_time = 0.0

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_cmd_time = (
            time.monotonic() - self.cmd_vel_timeout - 1.0
        )

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.previous_left_distance: Optional[float] = None
        self.previous_right_distance: Optional[float] = None

        self.cmd_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
        )
        self.odom_publisher = self.create_publisher(
            Odometry,
            self.odom_topic,
            20,
        )
        self.imu_publisher = self.create_publisher(
            Imu,
            '/imu/data',
            qos_profile_sensor_data,
        )
        self.joint_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            20,
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.control_timer = self.create_timer(
            1.0 / self.send_rate,
            self.control_cycle,
        )
        self.get_logger().info(
            f'底盘驱动已打开：{self.port} @ {self.baudrate}, '
        )

    def cmd_vel_callback(self, message: Twist) -> None:
        self.target_linear = float(message.linear.x)
        self.target_angular = float(message.angular.z)
        self.last_cmd_time = time.monotonic()

    def ensure_serial_connected(self) -> None:
        if self.serial_port is not None:
            return

        now = time.monotonic()
        if now < self.next_reconnect_time:
            return

        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.0,
                write_timeout=0.02,
            )
            self.rx_buffer.clear()
            self.serial_port.write(stop_command().encode('ascii'))
            self.serial_port.flush()
            self.get_logger().info(f'串口已连接：{self.port}')
        except (serial.SerialException, OSError) as error:
            self.serial_port = None
            self.next_reconnect_time = now + self.reconnect_interval
            self.get_logger().error(f'打开串口失败：{error}')

    def disconnect_serial(self, reason: Exception) -> None:
        self.get_logger().error(f'串口通信失败：{reason}')
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except (serial.SerialException, OSError):
                pass
        self.serial_port = None
        self.rx_buffer.clear()
        self.next_reconnect_time = (
            time.monotonic() + self.reconnect_interval
        )

    def current_command(self) -> str:
        if (
            time.monotonic() - self.last_cmd_time
            > self.cmd_vel_timeout
        ):
            return stop_command()

        left, right = twist_to_wheel_speeds(
            linear=self.target_linear,
            angular=self.target_angular,
            wheel_separation=self.wheel_separation,
            max_wheel_speed=self.max_wheel_speed,
        )
        left *= self.left_command_sign
        right *= self.right_command_sign
        return encode_wheel_command(left, right)

    def control_cycle(self) -> None:
        self.ensure_serial_connected()
        if self.serial_port is None:
            return

        try:
            command = self.current_command()
            self.serial_port.write(command.encode('ascii'))
            self.read_available_serial_data()
        except (serial.SerialException, OSError) as error:
            self.disconnect_serial(error)

    def read_available_serial_data(self) -> None:
        if self.serial_port is None:
            return

        waiting = self.serial_port.in_waiting
        if waiting > 0:
            self.rx_buffer.extend(self.serial_port.read(waiting))

        if len(self.rx_buffer) > 8192:
            self.get_logger().warning('串口接收缓存过长，已清空')
            self.rx_buffer.clear()
            return

        while b'\n' in self.rx_buffer:
            raw_line, _, remainder = self.rx_buffer.partition(b'\n')
            self.rx_buffer = bytearray(remainder)
            line = self.decode_serial_line(raw_line.rstrip(b'\r'))
            if line:
                self.handle_telemetry_line(line)

    @staticmethod
    def decode_serial_line(data: bytes) -> str:
        try:
            return data.decode('utf-8').strip()
        except UnicodeDecodeError:
            return data.decode('gbk', errors='replace').strip()

    def handle_telemetry_line(self, line: str) -> None:
        try:
            data = parse_telemetry(line)
        except ValueError as error:
            self.get_logger().warning(
                f'丢掉无效遥测帧：{error}，原文：{line}'
            )
            return

        stamp = self.get_clock().now().to_msg()
        self.publish_imu(data, stamp)
        self.publish_odometry_and_joints(data, stamp)

    def publish_imu(self, data, stamp) -> None:
        roll = data.angle[0] * self.angle_scale
        pitch = data.angle[1] * self.angle_scale
        yaw = data.angle[2] * self.angle_scale
        qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)

        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = self.imu_frame
        message.orientation.x = qx
        message.orientation.y = qy
        message.orientation.z = qz
        message.orientation.w = qw
        message.angular_velocity.x = data.gyro[0] * self.gyro_scale
        message.angular_velocity.y = data.gyro[1] * self.gyro_scale
        message.angular_velocity.z = data.gyro[2] * self.gyro_scale
        message.linear_acceleration.x = (
            data.accel[0] * self.acceleration_scale
        )
        message.linear_acceleration.y = (
            data.accel[1] * self.acceleration_scale
        )
        message.linear_acceleration.z = (
            data.accel[2] * self.acceleration_scale
        )
        message.orientation_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.05,
        ]
        message.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02,
        ]
        message.linear_acceleration_covariance = [
            0.10, 0.0, 0.0,
            0.0, 0.10, 0.0,
            0.0, 0.0, 0.10,
        ]
        self.imu_publisher.publish(message)

    def publish_odometry_and_joints(self, data, stamp) -> None:
        left_speed = data.left_speed * self.left_feedback_sign
        right_speed = data.right_speed * self.right_feedback_sign
        left_distance = (
            data.left_distance * self.left_feedback_sign
        )
        right_distance = (
            data.right_distance * self.right_feedback_sign
        )

        if self.previous_left_distance is None:
            left_delta = 0.0
            right_delta = 0.0
        else:
            left_delta = (
                left_distance - self.previous_left_distance
            )
            right_delta = (
                right_distance - self.previous_right_distance
            )

            if (
                abs(left_delta) > self.distance_jump_threshold
                or abs(right_delta) > self.distance_jump_threshold
            ):
                self.get_logger().warning(
                    '检测到里程计跳变，本次不进行位姿积分'
                )
                left_delta = 0.0
                right_delta = 0.0

        self.previous_left_distance = left_distance
        self.previous_right_distance = right_distance

        self.x, self.y, self.yaw = integrate_differential_drive(
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            left_delta=left_delta,
            right_delta=right_delta,
            wheel_separation=self.wheel_separation,
        )

        linear_speed = (right_speed + left_speed) / 2.0
        angular_speed = (
            right_speed - left_speed
        ) / self.wheel_separation
        qx, qy, qz, qw = quaternion_from_rpy(
            0.0,
            0.0,
            self.yaw,
        )

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = linear_speed
        odom.twist.twist.angular.z = angular_speed
        odom.pose.covariance = [
            0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05,
        ]
        odom.twist.covariance = odom.pose.covariance
        self.odom_publisher.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        if self.publish_odom_tf:
            self.tf_broadcaster.sendTransform(transform)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ['left_wheel_joint', 'right_wheel_joint']
        # URDF ????? +Y?????????????????????
        joints.position = [
            -left_distance / self.wheel_radius,
            -right_distance / self.wheel_radius,
        ]
        joints.velocity = [
            -left_speed / self.wheel_radius,
            -right_speed / self.wheel_radius,
        ]
        self.joint_publisher.publish(joints)

    def destroy_node(self) -> bool:
        if self.serial_port is not None:
            try:
                self.serial_port.write(stop_command().encode('ascii'))
                self.serial_port.flush()
                self.serial_port.close()
            except (serial.SerialException, OSError):
                pass
            self.serial_port = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SerialDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
