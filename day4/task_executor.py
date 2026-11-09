
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



import json
from datetime import datetime, timezone
from pathlib import Path


def now_utc():
    return datetime.now(timezone.utc).isoformat()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "coordinates",
        nargs="+",
        type=float,
        help="输入 x1 y1 x2 y2 ..."
    )

    args = parser.parse_args(remove_ros_args()[1:])
    coords = args.coordinates

    if len(coords) < 4 or len(coords) % 2 != 0:
        parser.error("至少需要两个目标，且坐标必须成对")

    goals = list(zip(coords[::2], coords[1::2]))

    log_dir = Path.home() / "embodied-agent" / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    run_id = datetime.now(timezone.utc).strftime(
        "%Y%m%d_%H%M%S_%f"
    )
    log_file = log_dir / f"task_{run_id}.json"

    report = {
        "run_id": run_id,
        "status": "PENDING",
        "created_at": now_utc(),
        "tasks": [
            {
                "id": i,
                "x": x,
                "y": y,
                "status": "PENDING"
            }
            for i, (x, y) in enumerate(goals, 1)
        ]
    }

    def save_report():
        log_file.write_text(
            json.dumps(report, ensure_ascii=False, indent=2),
            encoding="utf-8"
        )

    save_report()

    rclpy.init()
    node = RobotNavigator()

    try:
        report["status"] = "RUNNING"
        save_report()

        for task in report["tasks"]:
            task["status"] = "RUNNING"
            task["started_at"] = now_utc()
            save_report()

            started = time.monotonic()

            node.get_logger().info(
                f"任务{task['id']}开始 | "
                f"目标=({task['x']}, {task['y']})"
            )

            try:
                success = node.navigate(task["x"], task["y"])
            except Exception as exc:
                success = False
                task["error"] = str(exc)

            task["duration_seconds"] = round(
                time.monotonic() - started, 2
            )
            task["ended_at"] = now_utc()
            task["status"] = (
                "SUCCEEDED" if success else "FAILED"
            )

            save_report()

            if not success:
                report["status"] = "FAILED"
                node.get_logger().error(
                    f"任务{task['id']}失败，停止执行"
                )
                break
        else:
            report["status"] = "SUCCEEDED"
            node.get_logger().info("全部任务执行成功！")

    finally:
        if report["status"] == "RUNNING":
            report["status"] = "INTERRUPTED"

        for task in report["tasks"]:
            if task["status"] == "RUNNING":
                task["status"] = "INTERRUPTED"

        report["ended_at"] = now_utc()
        save_report()

        node.get_logger().info(
            f"任务日志已保存：{log_file}"
        )

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
