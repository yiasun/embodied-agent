"""Read only the selected provider's environment configuration."""
from .base import PlanningError


def request_timeout(env):
    try:
        return float(env.get('LLM_TIMEOUT_SECONDS', '30'))
    except (TypeError, ValueError):
        raise PlanningError('INVALID_TIMEOUT') from None
