"""Plan only: symbolic intent -> local validation -> trusted registry coordinates.

There is deliberately no ROS import, subprocess launch, or --execute option.
"""
import argparse
import copy
import hashlib
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from planner.intent_validator import validate_input, validate_intent
from planner.instruction_contract import resolve_navigation_instruction
from planner.planner_v1 import build_plan, load_scene
from planner.providers import PlanningError, ProviderResult, create_provider
from planner.validation import validate_plan_structure
from planner.plan_identity import PLAN_ID_SCHEME, compute_plan_id, validate_trusted_plan, verify_plan_id, plan_scheme
from planner.provenance import SOURCE_FIELDS, make_source_record
from day4.reporting import atomic_write_json


@dataclass(frozen=True)
class PlanningResult:
    plan: dict
    provenance: dict = field(repr=False)


def plan_instruction(instruction, scene, *, provider=None, env=None, transport=None):
    validate_input(instruction, scene)
    registry = copy.deepcopy(scene)
    contract = resolve_navigation_instruction(instruction, registry)
    provider = provider if provider is not None else create_provider(env=env, transport=transport)
    response = provider.generate_intent(instruction, copy.deepcopy(registry))
    if not isinstance(response, ProviderResult):
        raise PlanningError('INVALID_RESPONSE')
    intent = validate_intent(response.content, registry, expected_locations=contract.locations)
    try:
        plan = build_plan(registry, [s['location'] for s in intent['steps']])
        validate_plan_structure(plan)
    except ValueError:
        raise PlanningError('INVALID_PLAN') from None
    plan_id = compute_plan_id(plan)
    metadata = {key: getattr(response, key) for key in SOURCE_FIELDS}
    source = make_source_record(metadata, plan_id, plan_id_scheme=plan_scheme(plan))
    provenance = {
        'provenance_version': '2.0',
        'plan_id_scheme': plan_scheme(plan),
        'provenance_id': source['provenance_id'],
        'faithfulness': {'grammar': contract.grammar,
                         'requested_locations': list(contract.locations),
                         'verified': True},
        'plan_id': plan_id,
        'scene_sha256': hashlib.sha256(json.dumps(registry, ensure_ascii=False, sort_keys=True,
                                                 allow_nan=False).encode('utf-8')).hexdigest(),
        'instruction_sha256': hashlib.sha256(instruction.encode('utf-8')).hexdigest(),
        **metadata,
        'content': response.content,
        'raw_response': response.raw_response,
    }
    return PlanningResult(plan, provenance)


def save_result(result, directory):
    """New bundle only. Publish plan last; never overwrite historical artifacts."""
    # Revalidate again in case a caller modified the returned mutable dict.
    validate_trusted_plan(result.plan)
    verify_plan_id(result.plan, result.provenance.get('plan_id'))
    source = make_source_record(result.provenance, compute_plan_id(result.plan), plan_id_scheme=plan_scheme(result.plan))
    if (result.provenance.get('provenance_version') != '2.0'
            or result.provenance.get('plan_id_scheme') != plan_scheme(result.plan)
            or result.provenance.get('provenance_id') != source['provenance_id']):
        raise PlanningError('INVALID_PROVENANCE')
    directory = Path(directory)
    created = False
    try:
        directory.mkdir(parents=True, exist_ok=False)
        created = True
        atomic_write_json(directory / 'provenance.json', result.provenance)
        atomic_write_json(directory / 'plan.json', result.plan)
    except (OSError, ValueError):
        if created:
            for name in ('plan.json', 'provenance.json'):
                try:
                    (directory / name).unlink(missing_ok=True)
                except OSError:
                    pass
            try:
                directory.rmdir()
            except OSError:
                pass
        raise PlanningError('ARTIFACT_WRITE_ERROR') from None


def main(argv=None, *, env=None, transport=None):
    parser = argparse.ArgumentParser(description='仅规划；默认rule；不发送导航目标')
    parser.add_argument('instruction')
    parser.add_argument('--scene', default='turtlebot3_demo')
    parser.add_argument('--output-dir', type=Path, help='显式保存新计划与来源记录；拒绝覆盖已有目录')
    args = parser.parse_args(argv)
    try:
        try:
            scene = load_scene(args.scene)
        except (ValueError, OSError):
            raise PlanningError('INVALID_SCENE') from None
        result = plan_instruction(args.instruction, scene, env=env, transport=transport)
        if args.output_dir is not None:
            save_result(result, args.output_dir)
    except PlanningError as exc:
        print(json.dumps(exc.as_dict(), ensure_ascii=False), file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('{"error":"INTERRUPTED"}', file=sys.stderr)
        return 130
    except Exception:
        # An unexpected internal failure must not print exception text, request
        # headers, or an untrusted response; it also must not publish a plan.
        print('{"error":"PLANNING_ERROR"}', file=sys.stderr)
        return 1
    # Raw model output stays out of console logs. Persisted provenance is redacted.
    summary = {k: result.provenance[k] for k in ('provider', 'model', 'requested_model',
               'request_id', 'input_tokens', 'output_tokens', 'total_tokens', 'duration_seconds', 'plan_id',
               'provenance_version', 'plan_id_scheme', 'provenance_id')}
    print(json.dumps({'plan': result.plan, 'provenance': summary}, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
