
from dataclasses import dataclass


@dataclass(frozen=True)
class Decision:
    action: str
    reason: str


def decide(status, reason, attempt, max_retries):
    # 任务成功，执行下一个目标
    if status == "SUCCEEDED":
        return Decision("CONTINUE", "goal_reached")

    # 仅对已确认取消的超时任务允许重试
    if (
        status == "TIMED_OUT"
        and reason == "timeout_cancel_confirmed"
    ):
        if attempt <= max_retries:
            return Decision("RETRY", "retry_allowed")

        return Decision("HALT", "retries_exhausted")

    # 不知道导航是否仍在运行时，禁止重试
    if status == "UNKNOWN":
        return Decision("HALT", "goal_state_unknown")

    # 其他失败暂时不自动重试
    return Decision("HALT", "non_retryable_failure")
