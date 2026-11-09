
import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.utilities import remove_ros_args

from task_executor_v2 import RobotNavigator


def now_utc():
    return datetime.now(timezone.utc).isoformat()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("coordinates", nargs="+", type=float)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--retries", type=int, default=1)

    args = parser.parse_args(remove_ros_args()[1:])

    if len(args.coordinates) < 4 or len(args.coordinates) % 2:
        parser.error("至少提供两个目标，坐标必须成对")
    if args.timeout <= 0 or args.retries < 0:
        parser.error("timeout必须大于0，retries不能为负数")

    goals = list(zip(
        args.coordinates[::2],
        args.coordinates[1::2]
    ))

    run_id = datetime.now(timezone.utc).strftime(
        "%Y%m%d_%H%M%S_%f"
    )
    log_dir = Path.home() / "embodied-agent" / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / f"task_v3_{run_id}.json"

    report = {
        "run_id": run_id,
        "status": "PENDING",
        "created_at": now_utc(),
        "max_retries": args.retries,
        "tasks": [
            {
                "id": i,
                "x": x,
                "y": y,
                "status": "PENDING",
                "attempts": []
            }
            for i, (x, y) in enumerate(goals, 1)
        ]
    }

    def save():
        log_file.write_text(
            json.dumps(report, ensure_ascii=False, indent=2),
            encoding="utf-8"
        )

    save()
    rclpy.init()
    node = RobotNavigator()

    try:
        report["status"] = "RUNNING"
        save()

        for task in report["tasks"]:
            task["status"] = "RUNNING"
            task["started_at"] = now_utc()
            task_start = time.monotonic()
            save()

            for attempt_num in range(1, args.retries + 2):
                node.get_logger().info(
                    f"任务{task['id']}，第{attempt_num}次尝试"
                )

                started = time.monotonic()
                attempt = {
                    "number": attempt_num,
                    "started_at": now_utc(),
                    "status": "RUNNING"
                }
                task["attempts"].append(attempt)
                save()

                try:
                    success = node.navigate(
                        task["x"],
                        task["y"],
                        timeout_sec=args.timeout
                    )
                    status = node.last_status
                    reason = node.last_reason
                except Exception as exc:
                    success = False
                    status = "FAILED"
                    reason = f"exception_{type(exc).__name__}"

                attempt["status"] = status
                attempt["reason"] = reason
                attempt["duration_seconds"] = round(
                    time.monotonic() - started, 2
                )
                attempt["ended_at"] = now_utc()
                save()

                if success:
                    break

                can_retry = (
                    status == "TIMED_OUT"
                    and reason == "timeout_cancel_confirmed"
                    and attempt_num <= args.retries
                )

                if not can_retry:
                    break

                node.get_logger().warning(
                    "已确认取消导航，1秒后重新尝试"
                )
                time.sleep(1)

            task["status"] = status
            task["reason"] = reason
            task["duration_seconds"] = round(
                time.monotonic() - task_start, 2
            )
            task["ended_at"] = now_utc()
            save()

            if not success:
                report["status"] = status
                node.get_logger().error(
                    f"任务{task['id']}未完成，停止后续任务"
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
        save()

        node.get_logger().info(f"日志保存：{log_file}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
