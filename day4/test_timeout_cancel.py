
import argparse

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.utilities import remove_ros_args
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose


class CancelTest(Node):
    def __init__(self):
        super().__init__("navigation_cancel_test")
        self.client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )

    def run(self, x, y, timeout):
        if not self.client.wait_for_server(timeout_sec=10):
            print("失败：Nav2服务不可用")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        print(f"发送导航目标 ({x}, {y})")

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self, send_future, timeout_sec=10
        )

        if not send_future.done():
            print("发送响应超时，目标状态未知")
            return

        handle = send_future.result()

        if handle is None or not handle.accepted:
            print("目标被拒绝")
            return

        print(f"目标已接受，最多等待 {timeout} 秒")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=timeout
        )

        if result_future.done():
            status = result_future.result().status
            print(f"目标已结束，状态码：{status}")
            return

        print("超过时间限制，发送取消请求...")

        cancel_future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(
            self, cancel_future, timeout_sec=5
        )

        if not cancel_future.done():
            print("警告：取消请求未确认")
            return

        response = cancel_future.result()

        if response is None or not response.goals_canceling:
            print("警告：取消请求未被接受")
            return

        print("取消请求已接受，等待最终结果...")

        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=10
        )

        if not result_future.done():
            print("警告：尚未确认机器人已取消任务")
            return

        status = result_future.result().status

        if status == GoalStatus.STATUS_CANCELED:
            print("测试成功：目标已取消")
        elif status == GoalStatus.STATUS_SUCCEEDED:
            print("目标在取消过程中成功完成")
        else:
            print(f"目标结束，状态码：{status}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("--timeout", type=float, default=2.0)

    args = parser.parse_args(remove_ros_args()[1:])

    if args.timeout <= 0:
        parser.error("timeout必须大于0")

    rclpy.init()
    node = CancelTest()

    try:
        node.run(args.x, args.y, args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
