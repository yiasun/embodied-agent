"""ROS Node adapter; legacy CLI delegates to the safe V4 entry."""

def main():
    if __package__:
        from .task_executor_v4 import main as run
    else:
        from task_executor_v4 import main as run
    return run()


# Preview/help must work without importing ROS, just like the V4 entry.
if __name__ == "__main__":
    raise SystemExit(main())

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

if __package__:
    from .navigation_session import NavigationSession
else:
    from navigation_session import NavigationSession


class RobotNavigator(Node):
    def __init__(self):
        super().__init__("embodied_navigation_client")

        self.client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose"
        )
        self.last_feedback = 0.0
        self.last_status, self.last_reason = "PENDING", "not_started"
        self.session = NavigationSession(
            self.client,
            lambda future, seconds: rclpy.spin_until_future_complete(
                self, future, timeout_sec=seconds),
            self._make_goal, self.feedback_callback,
        )

    def feedback_callback(self, message):
        now = time.monotonic()

        if now - self.last_feedback >= 2.0:
            feedback = message.feedback
            self.get_logger().info(
                f"剩余距离: {feedback.distance_remaining:.2f} 米"
            )
            self.last_feedback = now

    def _make_goal(self, x, y):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        return goal

    def navigate(self, x, y, timeout_sec=120.0):
        try:
            return self.session.navigate(x, y, timeout_sec)
        finally:
            self.last_status = self.session.last_status
            self.last_reason = self.session.last_reason
            self.get_logger().info(f"导航结果: {self.last_status} | {self.last_reason}")

    def abort_active(self):
        confirmed = self.session.abort_active()
        if not confirmed:
            self.last_status = "UNKNOWN"
            self.last_reason = self.session.last_reason
        return confirmed


