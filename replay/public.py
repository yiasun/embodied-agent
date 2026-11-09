"""Recompute published metrics without private raw logs or a ROS runtime.

This verifies public geometry/calculations and plan bindings. Original model
request metadata and full journal integrity require the owner's private logs;
this module does not claim to authenticate those unavailable original records.
"""
import copy
import math
from pathlib import Path

from planner.plan_identity import compute_plan_id, require_plan_id
from planner.planner_v1 import build_plan, load_scene
from planner.validation import strict_json_loads
from scenes.assets import validate_scene_assets
from . import analysis as a

ROOT = Path(__file__).resolve().parents[1]


def same(actual, expected, label):
    if isinstance(expected, dict):
        if not isinstance(actual, dict) or set(actual) != set(expected):
            raise ValueError('Public evidence keys differ: ' + label)
        for key in expected:
            same(actual[key], expected[key], label + '.' + key)
    elif isinstance(expected, list):
        if not isinstance(actual, list) or len(actual) != len(expected):
            raise ValueError('Public evidence list differs: ' + label)
        for index, value in enumerate(expected):
            same(actual[index], value, label + '.' + str(index))
    elif type(expected) in (int, float):
        if type(actual) not in (int, float) or not math.isfinite(actual) or not math.isclose(actual, expected, abs_tol=1e-10, rel_tol=1e-10):
            raise ValueError('Public evidence calculation differs: ' + label)
    elif actual != expected:
        raise ValueError('Public evidence differs: ' + label)


def verify_public(data, root=ROOT):
    if data.get('schema') != 'offline-replay-v1' or data.get('frame') != 'gazebo_world':
        raise ValueError('Unsupported public evidence')
    scene = load_scene(data['scene_id'])
    bundle = validate_scene_assets(scene, root)
    same(data['scene_assets'], bundle['identity'], 'assets')
    same(data['geometry'], {k: bundle['source'][k] for k in ['map', 'boxes', 'waypoints']}, 'geometry')
    same(data['robot_radius_m'], bundle['source']['robot_radius_m'], 'radius')
    if set(data['experiments']) != {'D1', 'D2', 'D3'}:
        raise ValueError('Missing experiments')
    samples = {}
    for name, experiment in data['experiments'].items():
        rows = []; previous = None
        for frame in experiment['frames']:
            if (frame.get('kind') != 'sample' or frame.get('task_id') != 3
                    or not a.finite_vector(frame.get('robot'), 3)
                    or type(frame.get('t')) not in (int, float) or not math.isfinite(frame['t'])
                    or type(frame.get('source_row')) is not int or frame['source_row'] < 0
                    or type(frame.get('connect_previous')) is not bool):
                raise ValueError('Invalid public sample')
            if previous is None:
                if frame['t'] != 0 or frame['connect_previous']:
                    raise ValueError('Invalid public first sample')
            elif frame['t'] <= previous['t'] or frame['source_row'] <= previous['source_row']:
                raise ValueError('Public samples are not ordered')
            if frame['connect_previous'] and (previous is None or frame['source_row'] != previous['source_row'] + 1):
                raise ValueError('Public sample bridges a source gap')
            rows.append({'sim_time': frame['t'], 'robot': frame['robot'], 'obstacle': frame['obstacle'],
                'task_id': 3, 'source_row': frame['source_row'], 'segment_valid': frame['connect_previous'],
                'geometric_clearance_m': frame['clearance_m']})
            previous = frame
        if len(rows) < 2:
            raise ValueError('Insufficient public samples')
        samples[name] = rows
        binding = experiment['binding']
        require_plan_id(binding['plan_id']); require_plan_id(binding['provenance_id'])
        plan = build_plan(scene, [t['location'] for t in experiment['tasks']])
        if compute_plan_id(plan) != binding['plan_id'] or binding['plan_id_scheme'] != 'trusted-plan-sha256-v2':
            raise ValueError('Public trusted plan mismatch')
        if binding['plan_id'] != data['experiments']['D1']['binding']['plan_id']:
            raise ValueError('Public comparison plans differ')
        for event in experiment['events']:
            if event.get('robot') is not None or event.get('robot_pose_at_event') != 'NOT_OBSERVED':
                raise ValueError('Public event invents robot pose')
            if not math.isfinite(event['t']) or event['t'] < 0:
                raise ValueError('Invalid public event time')
    reference = a.reference_segments(samples['D1'])
    computed = {}
    for name, experiment in data['experiments'].items():
        rows = a.frames(samples[name], reference, data['robot_radius_m'])
        same(experiment['frames'], rows, name + '.frames')
        gaps = [r['clearance_m'] for r in rows if r['clearance_m'] is not None]
        # Reversals are calculated from the measured obstacle positions, not
        # generated from the configured shuttle pattern.
        stats = {**a.summarize(samples[name]), 'sample_count': len(rows),
            'spatial_speed': a.spatial_speed(samples[name]),
            'maximum_sampled_deviation_m': max(r['deviation_m'] for r in rows),
            'minimum_sampled_clearance_m': min(gaps) if gaps else None,
            'observed_y_reversals': a.reversals(samples[name])}
        same(experiment['task3'], stats, name + '.task3')
        computed[name] = stats
    comparison = data['comparison']
    same(comparison['maximum_sampled_deviation_m'], computed['D3']['maximum_sampled_deviation_m'], 'comparison.deviation')
    same(comparison['spatial_speed_reduction_percent'], 100 * (1 - computed['D3']['spatial_speed']['mean_speed_m_s'] /
        computed['D1']['spatial_speed']['mean_speed_m_s']), 'comparison.speed')
    same(comparison['task3_wall_delta_s'], data['experiments']['D3']['tasks'][2]['duration_wall_s'] -
        data['experiments']['D1']['tasks'][2]['duration_wall_s'], 'comparison.wall_delta')
    return computed


def load_public(path=None):
    path = Path(path) if path is not None else ROOT / 'web/replay/data/evidence.json'
    data = strict_json_loads(path.read_text(encoding='utf-8'))
    verify_public(data)
    return copy.deepcopy(data)


if __name__ == '__main__':
    result = load_public()
    print('Public evidence verified: D1 / D2 / D3; geometry, trusted plan and derived metrics. No raw-log authentication claimed.')
