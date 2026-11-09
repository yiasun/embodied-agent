
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.utilities import remove_ros_args
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose


class RobotNavigator(Node):
    def __init__(self):
        super().__init__("embodied_navigation_client")

        self.client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose"
        )
        self.last_feedback = 0.0

    def feedback_callback(self, message):
        now = time.monotonic()

        if now - self.last_feedback >= 2.0:
            feedback = message.feedback
            self.get_logger().info(
                f"剩余距离: {feedback.distance_remaining:.2f} 米"
            )
            self.last_feedback = now

    def navigate(self, x, y):
        self.get_logger().info("正在连接Nav2导航服务...")

        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("导航服务不可用")
            return False

        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        # 目标朝向设为0度
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f"发送目标位置: x={x:.2f}, y={y:.2f}"
        )

        future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("导航目标被拒绝")
            return False

        self.get_logger().info("目标已接受，机器人开始导航")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        response = result_future.result()

        if response.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("导航成功！")
            return True

        self.get_logger().error(
            f"导航失败，状态码: {response.status}"
        )
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    args = parser.parse_args(remove_ros_args()[1:])

    rclpy.init()
    node = RobotNavigator()

    try:
        node.navigate(args.x, args.y)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
