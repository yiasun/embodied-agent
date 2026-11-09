"""Synthetic binding fixtures only; never presented as real experiment evidence."""
import copy
import json
import shutil

from planner.plan_identity import compute_plan_id, plan_scheme
from planner.planner_v1 import build_plan, load_scene
from planner.provenance import make_source_record
from replay import build as b


def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False), encoding='utf-8')


def synthetic_bundle(root, monkeypatch):
    specs = {name: (name + '_SYNTHETIC', 'synthetic-' + name.lower()) for name in ['D1', 'D2', 'D3']}
    monkeypatch.setattr(b, 'SPECS', specs)
    shutil.copytree(b.ROOT / 'assets/scenes', root / 'assets/scenes')
    shutil.copytree(b.ROOT / 'configs', root / 'configs')
    plan = build_plan(load_scene('inspection_world'), ['配电间', '仓储间', '机房', '巡检站'])
    identity = compute_plan_id(plan)
    source = make_source_record({'provider': 'rule', 'model': 'synthetic-fixture', 'requested_model': None,
        'request_id': None, 'input_tokens': None, 'output_tokens': None, 'total_tokens': None,
        'duration_seconds': 0}, identity, plan_id_scheme=plan_scheme(plan))
    tasks = [{'id': s['step_id'], 'x': s['x'], 'y': s['y'], 'status': 'SUCCEEDED', 'reason': 'goal_reached',
        'duration_seconds': 1, 'attempts': [{'status': 'SUCCEEDED', 'recovery_action': 'CONTINUE'}]} for s in plan['steps']]
    for name, (stem, run) in specs.items():
        report = {'report_version': '2.0', 'run_id': run, 'plan_id_scheme': plan_scheme(plan),
            'plan': plan, 'source': source, 'plan_id': identity, 'status': 'SUCCEEDED', 'tasks': tasks,
            'ended_at': '2020-01-01T00:00:00Z'}
        samples = [{'sim_time': 1 + i * .1, 'frame': 'gazebo_world', 'robot': [21.5 - i*.02,22.5,0],
            'task_id': 3, 'task_status': 'RUNNING', 'navigation_status': 'RUNNING',
            'goal': {'id': 3, 'x': 14.5, 'y': 29.5}, 'segment_valid': bool(i)} for i in range(2)]
        evaluation = {'plan_id': identity, 'run_id': run, 'provenance_id': source['provenance_id'],
            'scene_id': 'inspection_world', 'scene_assets': plan['scene_assets'],
            'robot_geometry': {'type': 'configured_conservative_disk', 'radius_m': .22},
            'v4': {'tasks': tasks, 'status': 'SUCCEEDED'}, 'samples': samples,
            'metrics': {}, 'quality': {}, 'paths': [], 'bt_events': []}
        write(root / 'logs' / ('task_v4_' + run + '.json'), report)
        directory = root / 'logs/reliability'
        if name == 'D1':
            write(directory / (stem + '.json'), evaluation)
            continue
        lease = {'lease_id': 'synthetic-owner-' + name, 'entity_id': 99, 'status': 'COMPLETED',
            'cleanup': 'CLEAN', 'motion_stop': 'LATCHED_ZERO_COMMAND', 'entity_name': 'synthetic-obstacle',
            'motion_trace': [{'token': 'synthetic-owner-' + name, 'entity_id': 99,
                'entity_name': 'synthetic-obstacle', 'frame': 'gazebo_world'}]}
        journal = {**{k: evaluation[k] for k in ['plan_id','run_id','provenance_id','scene_id','scene_assets']}, **lease}
        evaluation['lease'] = lease
        raw_path = directory / (stem + '_metrics.json')
        write(raw_path, evaluation)
        final = copy.deepcopy(evaluation)
        final.update(input_evaluation_sha256=b.digest(raw_path.read_bytes()), observation_window_unchanged=True)
        write(directory / (stem + '_metrics_final.json'), final)
        write(directory / (stem + '.json'), journal)
    return root
