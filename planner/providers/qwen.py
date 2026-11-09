import os

from .base import PlanningError
from .compatible import CompatibleProvider
from .config import request_timeout


class QwenProvider(CompatibleProvider):
    def __init__(self, *, env=None, transport=None):
        env = os.environ if env is None else env
        key = env.get('QWEN_API_KEY')
        if not isinstance(key, str) or not key.strip():
            raise PlanningError('MISSING_API_KEY')
        if key != key.strip() or any(ord(c) < 33 or ord(c) > 126 for c in key):
            raise PlanningError('INVALID_API_KEY_FORMAT')
        super().__init__('qwen', env.get('QWEN_MODEL', 'qwen-plus'),
            env.get('QWEN_BASE_URL', 'https://dashscope.aliyuncs.com/compatible-mode/v1'),
            api_key=key, transport=transport, timeout=request_timeout(env))
