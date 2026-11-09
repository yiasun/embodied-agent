"""Typed source metadata, separately bound to the semantic plan identity.

These are local integrity links, not signatures against an adversary with write
access. Raw model JSON is never merged into reserved identity/source fields.
"""
import copy
import hashlib
import math

from .plan_identity import (PLAN_ID_SCHEME, LEGACY_AGENT_SCHEME, canonical_json,
                            compute_plan_id, legacy_agent_id, require_plan_id, verify_plan_id, plan_scheme, PLAN_ID_SCHEMES)
from .validation import finite_number, strict_json_loads

SOURCE_FIELDS = ('provider', 'model', 'requested_model', 'request_id', 'input_tokens',
                 'output_tokens', 'total_tokens', 'duration_seconds')
SOURCE_KEYS = set(SOURCE_FIELDS) | {'source_version', 'plan_id', 'plan_id_scheme', 'provenance_id', 'legacy_reference'}


def metadata_fields(value):
    result = {key: value.get(key) for key in SOURCE_FIELDS}
    if result['provider'] not in ('rule', 'qwen', 'ollama'):
        raise ValueError('非法provider元数据')
    for key in ('model', 'requested_model', 'request_id'):
        item = result[key]
        if item is None and key != 'model':
            continue
        if not isinstance(item, str) or not item or len(item) > 512 or any(ord(c) < 32 for c in item):
            raise ValueError('非法来源文本元数据')
    for key in ('input_tokens', 'output_tokens', 'total_tokens'):
        item = result[key]
        if item is not None and (type(item) is not int or item < 0):
            raise ValueError('非法Token元数据')
    duration = result['duration_seconds']
    if not finite_number(duration) or duration < 0:
        raise ValueError('非法请求耗时元数据')
    return result


def source_digest(source):
    payload = {key: value for key, value in source.items() if key != 'provenance_id'}
    return hashlib.sha256(canonical_json(payload).encode('utf-8')).hexdigest()


def make_source_record(metadata, plan_id, legacy_reference=None, *, plan_id_scheme=PLAN_ID_SCHEME):
    require_plan_id(plan_id)
    if plan_id_scheme not in PLAN_ID_SCHEMES:
        raise ValueError('未知plan_id版本')
    source = {'source_version': '1.0', 'plan_id': plan_id, 'plan_id_scheme': plan_id_scheme,
              **metadata_fields(metadata)}
    if legacy_reference is not None:
        source['legacy_reference'] = copy.deepcopy(legacy_reference)
    source['provenance_id'] = source_digest(source)
    return source


def validate_source_record(source, plan_id, *, plan_id_scheme=None):
    if not isinstance(source, dict) or set(source) - SOURCE_KEYS:
        raise ValueError('非法来源记录结构')
    require_plan_id(plan_id)
    if source.get('source_version') != '1.0' or source.get('plan_id_scheme') not in PLAN_ID_SCHEMES or source.get('plan_id') != plan_id:
        raise ValueError('来源记录与计划不匹配')
    if plan_id_scheme is not None and source['plan_id_scheme'] != plan_id_scheme:
        raise ValueError('来源plan_id算法版本不匹配')
    metadata_fields(source)
    if 'legacy_reference' in source:
        ref = source['legacy_reference']
        if not isinstance(ref, dict) or set(ref) != {'scheme', 'plan_id'} or ref['scheme'] != LEGACY_AGENT_SCHEME:
            raise ValueError('非法旧版本来源映射')
        require_plan_id(ref['plan_id'])
    require_plan_id(source.get('provenance_id'))
    if source_digest(source) != source['provenance_id']:
        raise ValueError('来源摘要不匹配')
    return copy.deepcopy(source)


def load_source(path, plan, *, allow_legacy=False):
    data = strict_json_loads(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError('非法来源文件')
    embedded = data.get('plan') if 'provenance' in data else None
    provenance = data.get('provenance', data)
    if not isinstance(provenance, dict):
        raise ValueError('非法provenance')
    plan_id = compute_plan_id(plan)
    if embedded is not None and compute_plan_id(embedded) != plan_id:
        raise ValueError('来源文件内嵌计划不匹配')
    if provenance.get('provenance_version') == '2.0':
        if provenance.get('plan_id_scheme') != plan_scheme(plan):
            raise ValueError('未知plan_id版本')
        verify_plan_id(plan, provenance.get('plan_id'))
        source = make_source_record(provenance, plan_id, plan_id_scheme=plan_scheme(plan))
        if provenance.get('provenance_id') != source['provenance_id']:
            raise ValueError('来源摘要不匹配')
        return source
    if plan['schema_version'] != '1.0':
        raise ValueError('新资产计划不接受旧哈希来源映射')
    if not allow_legacy or provenance.get('provenance_version') not in (None, '1.0'):
        raise ValueError('旧来源须显式允许或版本不受支持')
    old_id = provenance.get('plan_id')
    require_plan_id(old_id)
    if old_id != legacy_agent_id(embedded if embedded is not None else plan):
        raise ValueError('旧来源哈希不匹配')
    return make_source_record(provenance, plan_id,
                              {'scheme': LEGACY_AGENT_SCHEME, 'plan_id': old_id})
