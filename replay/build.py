"""Build a sanitized, static replay from selected local historical evidence only.

No ROS imports, network, model calls or experiment execution. Generated files are
separate from original reports. Run: python3 -m replay.build [--figures].
"""
import argparse
import hashlib
import json
from pathlib import Path

from planner.execute_plan import verify_execution_report
from planner.plan_identity import compute_plan_id, validate_trusted_plan
from planner.provenance import validate_source_record
from planner.validation import strict_json_loads
from scenes.assets import validate_scene_assets
from .analysis import frames, reference_segments, reversals, spatial_speed, summarize, task_samples

ROOT = Path(__file__).resolve().parents[1]
SPECS = {
    'D1': ('D1_retry_20261109_122902_735173165', 'da23b2b26cd1492fb798b59b94dee77e'),
    'D2': ('D2_handshake_20261109_140727_574372922', '6cbabd91908f496389b4b38a7e06d6d3'),
    'D3': ('D3_heartbeat_20261109_170747_737199338', 'd15c3309d9024ceda4e161e08aa407da'),
}


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def read(path):
    return strict_json_loads(path.read_text(encoding='utf-8'))


def check_binding(evaluation, report):
    plan, coordinates = validate_trusted_plan(report['plan'])
    plan_id = compute_plan_id(plan)
    source = validate_source_record(report['source'], plan_id, plan_id_scheme=report['plan_id_scheme'])
    for key, expected in [('plan_id', plan_id), ('run_id', report['run_id']),
                          ('provenance_id', source['provenance_id']), ('scene_id', plan['scene_id']),
                          ('scene_assets', plan['scene_assets'])]:
        if evaluation.get(key) != expected:
            raise ValueError('Evidence identity mismatch: ' + key)
    if evaluation['v4']['tasks'] != report['tasks'] or evaluation['v4']['status'] != report['status']:
        raise ValueError('Final V4 summary mismatch')
    for row in evaluation['samples']:
        if row['task_id'] is None:
            if row['task_status'] == 'RUNNING':
                raise ValueError('Running sample without task')
            continue
        if type(row['task_id']) is not int or not 1 <= row['task_id'] <= len(plan['steps']):
            raise ValueError('Invalid sampled task ID')
        step = plan['steps'][row['task_id'] - 1]
        if row['task_status'] != 'RUNNING' or row['navigation_status'] != 'RUNNING' or row['goal'] != {
                'id': step['step_id'], 'x': step['x'], 'y': step['y']}:
            raise ValueError('Sample task/goal mismatch')
    return coordinates


def check_comparable(evidence):
    """Task numbers alone do not prove the same navigation task across runs."""
    baseline = evidence['D1'][0]
    for evaluation, _, _ in evidence.values():
        for key in ['plan_id', 'scene_id', 'scene_assets']:
            if evaluation[key] != baseline[key]:
                raise ValueError('Cross-experiment comparison binding mismatch: ' + key)


def load_evidence(root=ROOT):
    result = {}; sources = []
    def record(path, role, experiment):
        sources.append({'experiment': experiment, 'role': role, 'file': path.name,
                        'sha256': digest(path.read_bytes())})
    for name, (stem, run) in SPECS.items():
        folder = root / 'logs/reliability'
        evaluation_path = folder / (stem + ('' if name == 'D1' else '_metrics_final') + '.json')
        report_path = root / 'logs' / ('task_v4_' + run + '.json')
        evaluation, report = read(evaluation_path), read(report_path)
        coordinates = check_binding(evaluation, report)
        verify_execution_report(report_path, run, evaluation['plan_id'], coordinates, expected_source=report['source'])
        record(evaluation_path, 'evaluation', name); record(report_path, 'final_navigation', name)
        journal = None
        if name != 'D1':
            raw_path = folder / (stem + '_metrics.json'); raw = read(raw_path)
            if digest(raw_path.read_bytes()) != evaluation['input_evaluation_sha256'] or evaluation['observation_window_unchanged'] is not True:
                raise ValueError('Finalization source digest mismatch')
            for key in ['samples', 'metrics', 'quality', 'paths', 'bt_events']:
                if raw[key] != evaluation[key]:
                    raise ValueError('Historical observation altered by finalization')
            journal_path = folder / (stem + '.json'); journal = read(journal_path)
            for key in ['plan_id', 'run_id', 'provenance_id', 'scene_id', 'scene_assets']:
                if journal[key] != evaluation[key]:
                    raise ValueError('Lease binding mismatch: ' + key)
            lease = evaluation['lease']
            for key in ['lease_id', 'entity_id', 'status', 'cleanup', 'motion_stop', 'motion_trace']:
                if journal[key] != lease[key]:
                    raise ValueError('Lease/evaluation mismatch: ' + key)
            for state in journal['motion_trace']:
                if (state['token'] != journal['lease_id'] or state['entity_id'] != journal['entity_id']
                        or state['entity_name'] != journal['entity_name'] or state['frame'] != 'gazebo_world'):
                    raise ValueError('Native trace identity mismatch')
            record(raw_path, 'observation_before_finalization', name); record(journal_path, 'obstacle_journal', name)
        result[name] = (evaluation, report, journal)
    check_comparable(result)
    scene_path = root / 'configs/scenes/inspection_world.json'
    bundle = validate_scene_assets(read(scene_path), root)
    for evaluation, _, _ in result.values():
        if evaluation['scene_assets'] != bundle['identity']:
            raise ValueError('Scene asset binding mismatch')
        if evaluation['robot_geometry'] != {'type': 'configured_conservative_disk', 'radius_m': .22}:
            raise ValueError('Unsupported robot geometry')
    record(scene_path, 'scene_registry', 'ALL')
    for role, path in bundle['paths'].items():
        record(path, 'asset_' + role, 'ALL')
    return result, bundle['source'], sources


def safety_events(journal, selected):
    if journal is None:
        return []
    events = []; seen = set(); origin = selected[0]['sim_time']
    for index, row in enumerate(journal['motion_trace']):
        if not row['stopped'] or row['reason'] in seen:
            continue
        seen.add(row['reason'])
        # Native state and robot observation are different streams. Do not
        # extrapolate the robot at the native stop time.
        events.append({'kind': 'native_stop', 't': row['sim_time'] - origin,
            'reason': row['reason'], 'robot': None,
            'obstacle': {'position': row['position'], 'size': journal['motion']['size'], 'yaw': row['yaw']},
            'last_robot_sample_t': selected[-1]['sim_time'] - origin,
            'native_trace_row': index, 'robot_pose_at_event': 'NOT_OBSERVED',
            'motion_stop': journal['motion_stop'], 'cleanup': journal['cleanup']})
    return events


def make_dataset(root=ROOT):
    evidence, geometry, sources = load_evidence(root)
    selected = {name: task_samples(values[0]['samples']) for name, values in evidence.items()}
    reference = reference_segments(selected['D1']); experiments = {}
    for name, (evaluation, report, journal) in evidence.items():
        samples = selected[name]; rows = frames(samples, reference)
        quality = evaluation['quality']; metrics = evaluation['metrics']
        tasks = [{'id': t['id'], 'location': report['plan']['steps'][t['id']-1]['location'],
                  'status': t['status'], 'duration_wall_s': t['duration_seconds'],
                  'recovery': t['attempts'][-1]['recovery_action']} for t in report['tasks']]
        motion = None if journal is None else {k: journal['motion'][k] for k in
            ['pattern', 'speed', 'duration_sim_s', 'size', 'stop_clearance_m', 'heartbeat_timeout_s']}
        gaps = [r['clearance_m'] for r in rows if r['clearance_m'] is not None]
        events = safety_events(journal, samples)
        experiments[name] = {
            'name': name, 'title': {'D1': '无障碍基线', 'D2': '单向运动障碍', 'D3': '往返运动障碍'}[name],
            'binding': {'plan_id': evaluation['plan_id'], 'run_id': evaluation['run_id'],
                        'provenance_id': evaluation['provenance_id'], 'plan_id_scheme': report['plan_id_scheme'],
                        'lease_ref': None if journal is None else 'sha256:' + digest(journal['lease_id'].encode())},
            'provider': report['source']['provider'], 'model': report['source']['model'],
            'navigation_status': report['status'], 'tasks': tasks,
            'quality': {k: quality[k] for k in ['status', 'sample_count', 'pose_stale_messages_dropped',
                       'max_sim_interval_s', 'issues', 'full_navigation_coverage']},
            'observation_end_navigation_status': evaluation.get('v4_at_observation_end', evaluation['v4'])['status'],
            'experiment_status': 'BASELINE_OBSERVED' if journal is None else journal['status'],
            'error': evaluation['error'], 'cleanup': None if journal is None else journal['cleanup'],
            'motion_stop': None if journal is None else journal['motion_stop'], 'motion': motion,
            'contact_status': evaluation['contacts']['status'], 'sensor_latency': 'NOT_MEASURED',
            'task3': {**summarize(samples), 'sample_count': len(samples), 'spatial_speed': spatial_speed(samples),
                      'maximum_sampled_deviation_m': max(r['deviation_m'] for r in rows),
                      'minimum_sampled_clearance_m': min(gaps) if gaps else None,
                      'observed_y_reversals': reversals(samples)},
            'path_publications': metrics['path_publications'], 'path_changes_proxy': metrics['observed_same_task_path_changes'],
            'planner_invocation_count': None,
            'nav2_recovery_transitions': evaluation['nav2']['recovery_running_transitions'],
            'v4_decisions': evaluation['v4']['task_level_decisions'],
            'frames': rows, 'events': events}
    a, b = experiments['D1'], experiments['D3']
    compare = {'task_id': 3, 'reference': 'D1 task 3 valid observed polyline; nearest segment projection, no time synchronization',
        'maximum_sampled_deviation_m': b['task3']['maximum_sampled_deviation_m'],
        'spatial_speed_reduction_percent': 100 * (1 - b['task3']['spatial_speed']['mean_speed_m_s'] / a['task3']['spatial_speed']['mean_speed_m_s']),
        'task3_wall_delta_s': b['tasks'][2]['duration_wall_s'] - a['tasks'][2]['duration_wall_s']}
    return {'schema': 'offline-replay-v1', 'scene_id': geometry['scene_id'], 'revision': geometry['revision'],
        'scene_assets': evidence['D1'][0]['scene_assets'], 'frame': 'gazebo_world',
        'robot_radius_m': .22, 'geometry': {k: geometry[k] for k in ['map', 'boxes', 'waypoints']},
        'comparison': compare, 'experiments': experiments, 'sources': sources,
        'limitations': [
            '历史离线观测，无控制接口；轨迹仅含实际采样，不补齐未观测导航。',
            'D1/D3按第3站及D1参考折线空间投影对齐，绝对仿真时刻不比较；投影不是同步的D1机器人。',
            '速度为有效相邻位姿差分；空间窗口均速使用线段裁剪后的路程/仿真时长，非命令速度。',
            '偏移为D3采样点至D1观测折线的单向最近距离，不是连续轨迹上界或因果避障量。',
            '净距为0.22m保守圆与障碍有向矩形的计算间距，仅针对注入障碍，不代表真实接触证据。',
            'D3在安全门禁停止，未完成25秒运动；V4随后4/4成功与运动实验FAILED同时成立。',
            '碰撞接触与传感器延迟NOT_MEASURED；21次Path变化是代理指标，不是21次正式重规划。',
            '独立单次运行、起始条件和采集窗口不同；减速/偏移是描述性对比，不能推断因果或普遍成功率。',
            '最后机器人采样与原生停止事件相差45ms；事件处机器人位姿未观测，不外推或伪造净距。',
            '哈希用于本地完整性关联，不是数字签名；下载数据为白名单脱敏派生数据，非原始报告。']}


def write_dataset(data, output):
    output.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(data, ensure_ascii=False, separators=(',', ':'), allow_nan=False)
    (output / 'evidence.json').write_text(encoded + '\n', encoding='utf-8')
    (output / 'sources.json').write_text(json.dumps(data['sources'], ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    # Escaping '<' makes the embedded JSON safe even for future labels that
    # contain script-closing text. No fetch is needed for local file:// use.
    (output.parent / 'evidence.js').write_text('window.REPLAY_DATA=' + encoded.replace('<', '\\u003c') + ';\n', encoding='utf-8')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--figures', action='store_true', help='Also render verified figures with local Matplotlib')
    args = parser.parse_args(argv)
    data = make_dataset()
    write_dataset(data, ROOT / 'web/replay/data')
    if args.figures:
        from .plot import render
        render(data, ROOT / 'docs/evidence/stage62/verified')
    print(json.dumps({'experiments': list(data['experiments']), 'comparison': data['comparison']}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
