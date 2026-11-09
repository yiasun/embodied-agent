import os

from .base import PlanningError
from .ollama import OllamaProvider
from .qwen import QwenProvider
from .rule import RuleProvider


def create_provider(*, env=None, transport=None):
    env = os.environ if env is None else env
    selected = env.get('LLM_PROVIDER', 'rule')
    if selected == 'rule':
        return RuleProvider()
    if selected == 'qwen':
        return QwenProvider(env=env, transport=transport)
    if selected == 'ollama':
        return OllamaProvider(env=env, transport=transport)
    raise PlanningError('UNKNOWN_PROVIDER')
