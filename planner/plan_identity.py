"""Versioned semantic identity. A digest is an identity, never validation proof."""
import copy
import hashlib
import json
import re
from decimal import Decimal

from .planner_v1 import load_scene
from .validation import validate_plan_structure

PLAN_ID_SCHEME = 'trusted-plan-sha256-v1'
ASSET_PLAN_ID_SCHEME = 'trusted-plan-sha256-v2'
PLAN_ID_SCHEMES = (PLAN_ID_SCHEME, ASSET_PLAN_ID_SCHEME)
LEGACY_AGENT_SCHEME = 'legacy-agent-pretty-json-sha256-v0'
LEGACY_FILE_SCHEME = 'legacy-file-text-sha256-v0'


def require_plan_id(value):
    if not isinstance(value, str) or not re.fullmatch(r'[0-9a-f]{64}', value):
        raise ValueError('非法plan_id')
    return value


def canonical_json(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False)


def number_text(value):
    # Shortest round-trip decimal representation of the actual Python/ROS number.
    # No rounding to a spatial tolerance. 1==1.0, -0==0, exponent spelling ignored.
    number = Decimal(int(value)) if type(value) is float and value.is_integer() else Decimal(str(value))
    if number == 0:
        return '0'
    text = format(number, 'f')
    return text.rstrip('0').rstrip('.') if '.' in text else text


def plan_scheme(plan):
    validate_plan_structure(plan)
    return ASSET_PLAN_ID_SCHEME if plan['schema_version'] == '2.0' else PLAN_ID_SCHEME


def canonical_plan(plan):
    validate_plan_structure(plan)
    payload = {'plan_id_scheme': plan_scheme(plan), 'schema_version': plan['schema_version'],
            'scene_id': plan['scene_id'], 'frame_id': plan['frame_id'],
            'steps': [{'step_id': step['step_id'], 'action': step['action'],
                       'location': step['location'], 'x': number_text(step['x']),
                       'y': number_text(step['y'])} for step in plan['steps']]}
    if plan['schema_version'] == '2.0':
        payload['scene_assets'] = copy.deepcopy(plan['scene_assets'])
    return payload


def compute_plan_id(plan):
    return hashlib.sha256(canonical_json(canonical_plan(plan)).encode('utf-8')).hexdigest()


def validate_trusted_plan(plan):
    """Recheck schema, registry, ordering and exact effective coordinates."""
    validate_plan_structure(plan)
    scene = load_scene(plan['scene_id'])
    assets = scene.get('assets')
    if assets is not None:
        expected = {'revision': assets['revision'], 'manifest_sha256': assets['manifest_sha256']}
        if plan['schema_version'] != '2.0' or plan.get('scene_assets') != expected:
            raise ValueError('计划场景资产版本不匹配或缺失')
    elif plan['schema_version'] != '1.0':
        raise ValueError('旧场景不接受无对应资产的2.0计划')
    coordinates = []
    for step in plan['steps']:
        waypoint = scene['waypoints'].get(step['location'])
        if waypoint is None:
            raise ValueError('未知地点')
        for axis in ('x', 'y'):
            if step[axis] != waypoint[axis]:
                raise ValueError('计划坐标与注册表不一致')
            coordinates.append(str(waypoint[axis]))
    return copy.deepcopy(plan), coordinates


def verify_plan_id(plan, claimed):
    require_plan_id(claimed)
    if compute_plan_id(plan) != claimed:
        raise ValueError('plan_id与计划内容不匹配')


def legacy_agent_id(plan):
    return hashlib.sha256((json.dumps(plan, ensure_ascii=False, indent=2) + '\n').encode('utf-8')).hexdigest()


def legacy_file_id(text):
    return hashlib.sha256(text.encode('utf-8')).hexdigest()
