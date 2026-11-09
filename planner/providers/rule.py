import json
import time

from .base import ProviderResult


class RuleProvider:
    def generate_intent(self, instruction, scene):
        from ..intent_validator import validate_input
        validate_input(instruction, scene)
        started = time.monotonic()
        from ..instruction_contract import resolve_navigation_instruction
        contract = resolve_navigation_instruction(instruction, scene)
        intent = {'steps': contract.steps()}
        return ProviderResult(json.dumps(intent, ensure_ascii=False), intent, 'rule',
            'rule-v1', 'rule-v1', None, None, None, None, round(time.monotonic() - started, 6))
