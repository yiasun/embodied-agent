"""Strict local validation shared by planning and execution. No ROS imports."""
import json
import math
import re
from pathlib import Path
from datetime import datetime

from jsonschema import Draft202012Validator, FormatChecker

SCENE_ID_PATTERN = r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$"
SCHEMA = json.loads((Path(__file__).parent / 'schemas' / 'navigation_plan.schema.json').read_text())
MIN_TASKS = SCHEMA['properties']['steps']['minItems']
MAX_TASKS = SCHEMA['properties']['steps']['maxItems']
FORMATS = FormatChecker()


@FORMATS.checks('date-time')
def valid_datetime(value):
    if not isinstance(value, str):
        return True  # The schema handles type errors.
    if not re.fullmatch(r'\d{4}-\d{2}-\d{2}[Tt]\d{2}:\d{2}:\d{2}(?:\.\d+)?(?:[Zz]|[+-]\d{2}:\d{2})', value):
        return False
    try:
        parsed = datetime.fromisoformat(value.upper().replace('Z', '+00:00'))
        return parsed.tzinfo is not None
    except ValueError:
        return False


VALIDATOR = Draft202012Validator(SCHEMA, format_checker=FORMATS)
SCHEMA_V2 = json.loads((Path(__file__).parent / 'schemas' / 'navigation_plan_v2.schema.json').read_text())
VALIDATOR_V2 = Draft202012Validator(SCHEMA_V2, format_checker=FORMATS)


def finite_number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def validate_scene_id(value):
    if not isinstance(value, str) or not re.fullmatch(SCENE_ID_PATTERN, value):
        raise ValueError('场景ID格式不合法')


def strict_json_loads(text):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('JSON包含重复字段')
            result[key] = value
        return result

    def constant(_):
        raise ValueError('JSON包含非有限数值')

    return json.loads(text, object_pairs_hook=pairs, parse_constant=constant)


def validate_scene(scene):
    if not isinstance(scene, dict):
        raise ValueError('场景必须为对象')
    validate_scene_id(scene.get('scene_id'))
    if scene.get('frame_id') != 'map':
        raise ValueError('V1仅支持map坐标系')
    waypoints = scene.get('waypoints')
    if not isinstance(waypoints, dict) or not waypoints:
        raise ValueError('地点注册表格式错误')
    for name, waypoint in waypoints.items():
        if not isinstance(name, str) or not name or not isinstance(waypoint, dict):
            raise ValueError('地点注册表格式错误')
        if not all(finite_number(waypoint.get(k)) for k in ('x', 'y')):
            raise ValueError('地点坐标必须为有限数值')


def validate_plan_structure(plan):
    validator = VALIDATOR_V2 if isinstance(plan, dict) and plan.get('schema_version') == '2.0' else VALIDATOR
    error = next(validator.iter_errors(plan), None)
    if error is not None:
        # Do not echo untrusted plan contents into logs/errors.
        raise ValueError('计划不符合navigation_plan JSON Schema')
    for index, step in enumerate(plan['steps'], 1):
        if type(step['step_id']) is not int or step['step_id'] != index:
            raise ValueError('步骤编号必须为连续整数')
        if not all(finite_number(step[k]) for k in ('x', 'y')):
            raise ValueError('坐标必须为有限数值')
