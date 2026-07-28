#!/usr/bin/env python3
import time
from typing import Optional

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node

from car_control_tools.goal_utils import (
    parse_goal_text,
    yaw_to_quaternion,
)


class TerminalGoalClient(Node):
    def __init__(self) -> None:
        super().__init__('terminal_goal_client')
        self.action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
        )
        self.active_goal: Optional[ClientGoalHandle] = None
        self.last_feedback_time = 0.0

    def feedback_callback(self, feedback_message) -> None:
        now = time.monotonic()
        if now - self.last_feedback_time < 1.0:
            return
        self.last_feedback_time = now
        feedback = feedback_message.feedback
        self.get_logger().info(
            f'ʣ����룺{feedback.distance_remaining:.2f} m'
        )

    def send_goal(self, x: float, y: float, yaw: float) -> None:
        self.get_logger().info('�ȴ� Nav2 NavigateToPose ���񡭡�')
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'δ�ҵ� navigate_to_pose����ȷ�� Nav2 ������������ active'
            )
            return

        qz, qw = yaw_to_quaternion(yaw)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f'����Ŀ�꣺x={x:.3f}, y={y:.3f}, '
            f'yaw={yaw:.3f} rad'
        )
        send_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Ŀ�걻 Nav2 �ܾ�')
            return

        self.active_goal = goal_handle
        self.get_logger().info('Ŀ���ѽ��ܣ�С����ʼ����')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        self.active_goal = None

        if wrapped_result is None:
            self.get_logger().error('δ�յ��������')
            return

        status = wrapped_result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('�����ɹ�')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning('������ȡ��')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('����ʧ�ܻ���ֹ')
        else:
            self.get_logger().warning(f'����������״̬�룺{status}')

    def cancel_active_goal(self) -> None:
        if self.active_goal is None:
            return
        cancel_future = self.active_goal.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)
        self.get_logger().warning('������ȡ����ǰĿ��')
        self.active_goal = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerminalGoalClient()

    print('�����ʽ��x y �� x y yaw�Ƕ�')
    print('ʾ����1.5 -0.3')
    print('ʾ����1.5 -0.3 90')
    print('���� q �� Enter �˳�')

    try:
        while rclpy.ok():
            text = input('\nĿ������> ').strip()
            if text.lower() in ('q', 'quit', 'exit'):
                break
            if not text:
                continue

            try:
                x, y, yaw = parse_goal_text(text)
            except ValueError as error:
                print(f'�������{error}')
                continue

            node.send_goal(x, y, yaw)
    except (KeyboardInterrupt, EOFError):
        print('\n�����˳�����')
        node.cancel_active_goal()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()