import os

from .base import PlanningError
from .compatible import CompatibleProvider
from .config import request_timeout


class OllamaProvider(CompatibleProvider):
    def __init__(self, *, env=None, transport=None):
        env = os.environ if env is None else env
        model = env.get('OLLAMA_MODEL')
        if not isinstance(model, str) or not model.strip():
            raise PlanningError('MISSING_OLLAMA_MODEL')
        # Local inference only. Do not silently route known cloud model tags.
        if model.lower().endswith((':cloud', '-cloud')):
            raise PlanningError('CLOUD_MODEL_NOT_ALLOWED')
        super().__init__('ollama', model,
            env.get('OLLAMA_BASE_URL', 'http://localhost:11434/v1'),
            transport=transport, timeout=request_timeout(env))
