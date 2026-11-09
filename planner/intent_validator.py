"""Strict intent gate: no coordinates, tools, parameters, or extra properties."""
import json
from pathlib import Path

from jsonschema import Draft202012Validator

from .providers.base import PlanningError
from .validation import MAX_TASKS, MIN_TASKS, strict_json_loads, validate_scene

SCHEMA = json.loads((Path(__file__).parent / 'schemas/task_intent.schema.json').read_text())
assert SCHEMA['properties']['steps']['minItems'] == MIN_TASKS
assert SCHEMA['properties']['steps']['maxItems'] == MAX_TASKS
VALIDATOR = Draft202012Validator(SCHEMA)
MAX_INSTRUCTION_CHARS = 8000
MAX_CONTENT_CHARS = 65536


def validate_input(instruction, scene):
    if not isinstance(instruction, str) or not instruction.strip() or len(instruction) > MAX_INSTRUCTION_CHARS:
        raise PlanningError('INVALID_INSTRUCTION')
    try:
        validate_scene(scene)
        from .planner_v1 import ROOT
        from scenes.assets import validate_scene_assets
        validate_scene_assets(scene, ROOT)
    except (ValueError, TypeError):
        raise PlanningError('INVALID_SCENE') from None


def validate_intent(content, scene, *, expected_locations):
    if not isinstance(content, str) or len(content) > MAX_CONTENT_CHARS:
        raise PlanningError('INVALID_JSON')
    try:
        intent = strict_json_loads(content)
    except (ValueError, RecursionError):
        raise PlanningError('INVALID_JSON') from None
    # Check business error codes only after structural types are safe. The full
    # schema still decides acceptance (including additionalProperties=false).
    if isinstance(intent, dict) and isinstance(intent.get('steps'), list):
        for step in intent['steps']:
            if isinstance(step, dict):
                if isinstance(step.get('action'), str) and step['action'] != 'NAVIGATE':
                    raise PlanningError('ILLEGAL_ACTION')
                if isinstance(step.get('location'), str) and step['location'] not in scene['waypoints']:
                    raise PlanningError('UNKNOWN_LOCATION')
    if next(VALIDATOR.iter_errors(intent), None) is not None:
        raise PlanningError('INVALID_INTENT_SCHEMA')
    # Exact list equality preserves cardinality, order, and repeated visits.
    # Set equality or subset checks would silently lose task requirements.
    if [step['location'] for step in intent['steps']] != list(expected_locations):
        raise PlanningError('TASK_FIDELITY_MISMATCH')
    return intent
