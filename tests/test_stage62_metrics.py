import copy
import math

import pytest

from reliability.metrics import Metrics, clearance, ttc
from reliability.safety import ObstacleError


def box(x=2, y=0, yaw=0):
    return {'position': [x, y, 0.6], 'size': [0.4, 0.4, 1.2], 'yaw': yaw}


def sample(t=1, x=0, obstacle=True):
    return {'sim_time': t, 'received_mono': t+100, 'frame': 'gazebo_world',
            'robot': [x, 0, 0], 'obstacle': box() if obstacle else None}


@pytest.mark.parametrize('point,expected', [([0, 0], 1.58), ([2, 0], -0.42), ([1.8, 0], -0.22),
                                           ([1.5, 0.2], 0.08)])
def test_geometric_clearance_is_not_center_distance(point, expected):
    assert clearance(point, 0.22, box()) == pytest.approx(expected)


def test_rotated_box_clearance():
    b = box(); b['size'] = [2, 0.4, 1.2]; b['yaw'] = math.pi/2
    assert clearance([2, 2], 0.22, b) == pytest.approx(0.78)


def test_ttc_approaching_face():
    assert ttc([0, 0], 0.22, box(), [0.5, 0], [0, 0]) == pytest.approx(3.16)


@pytest.mark.parametrize('velocity', [[-1, 0], [0, 0], [0, 1], [1, 2]])
def test_ttc_na_stationary_receding_or_miss(velocity):
    assert ttc([0, 0], 0.22, box(), velocity, [0, 0]) is None


def test_ttc_relative_velocity_and_unknown():
    assert ttc([0, 0], 0.22, box(), [0.5, 0], [0.5, 0]) is None
    assert ttc([0, 0], 0.22, box(), None, [0, 0]) is None
    assert ttc([2, 0], 0.22, box(), [1, 0], [0, 0]) == 0


def test_ttc_rounded_corner_not_square_expansion():
    # Tangent to rounded corner, outside the face strip.
    result = ttc([0, 0.4], 0.22, box(), [1, 0], [0, 0])
    assert result == pytest.approx(1.8-math.sqrt(0.22**2-0.2**2))


def test_metrics_velocity_length_ttc_and_identity():
    m = Metrics({'plan_id': 'plan', 'run_id': 'run', 'provenance_id': 'source'})
    m.sample(sample()); m.sample(sample(1.1, 0.05)); m.sample(sample(1.2, 0.1))
    r = m.report()
    assert r['metrics']['robot_observed_path_length_m'] == pytest.approx(0.1)
    assert r['metrics']['minimum_geometric_clearance_m'] == pytest.approx(1.48)
    assert r['metrics']['minimum_applicable_ttc_s'] == pytest.approx(2.96)
    assert r['samples'][0]['robot_velocity_world'] is None
    assert r['samples'][1]['robot_velocity_world'] == pytest.approx([0.5, 0])
    assert r['run_id'] == 'run' and r['provenance_id'] == 'source'
    assert r['quality']['status'] == 'VALID_OBSERVATION_WINDOW'


def test_no_contact_data_does_not_mean_no_collision_or_latency_zero():
    m = Metrics({}); m.sample(sample()); r = m.report()
    assert r['contacts']['status'] == 'NOT_MEASURED'
    assert r['contacts']['positive_contact_samples'] is None
    assert r['contacts']['overall_zero_collision_claim'] == 'UNKNOWN'
    assert r['metrics']['sensor_latency'] == {'status': 'NOT_MEASURED', 'seconds': None}


def test_positive_contact_is_evidence_not_unique_collision_count():
    m = Metrics({}); s = sample(); s['contact_evidence'] = {'status': 'MEASURED', 'robot_contacts': 2}
    m.sample(s); assert m.report()['contacts']['positive_contact_samples'] == 1


@pytest.mark.parametrize('fault', ['frame', 'nan', 'booltime', 'negative_size', 'quaternion_placeholder'])
def test_bad_sample_rejected(fault):
    s = sample()
    if fault == 'frame': s['frame'] = 'odom'
    if fault == 'nan': s['robot'][0] = float('nan')
    if fault == 'booltime': s['sim_time'] = True
    if fault == 'negative_size': s['obstacle']['size'][0] = -1
    if fault == 'quaternion_placeholder': s['robot'] = [1, 2]
    with pytest.raises(ObstacleError): Metrics({}).sample(s)


@pytest.mark.parametrize('fault', ['sim_reset', 'wall_reset', 'duplicate'])
def test_clock_fail_closed(fault):
    m = Metrics({}); m.sample(sample()); s = sample(1.1)
    if fault == 'sim_reset': s['sim_time'] = 0.9
    if fault == 'wall_reset': s['received_mono'] = 99
    if fault == 'duplicate': s['sim_time'] = 1
    with pytest.raises(ObstacleError): m.sample(s)


@pytest.mark.parametrize('fault', ['gap', 'robot_jump', 'obstacle_jump', 'wall_gap'])
def test_gaps_and_jumps_excluded_from_length(fault):
    m = Metrics({}); m.sample(sample()); s = sample(1.1)
    if fault == 'gap': s['sim_time'] = 2
    if fault == 'robot_jump': s['robot'][0] = 10
    if fault == 'obstacle_jump': s['obstacle']['position'][0] = 10
    if fault == 'wall_gap': s['received_mono'] = 110
    m.sample(s); r = m.report()
    assert r['quality']['status'] == 'INCOMPLETE'
    assert r['metrics']['robot_observed_path_length_m'] == 0
    assert r['samples'][-1]['ttc_s'] is None


def test_baseline_obstacle_metrics_null():
    m = Metrics({}); m.sample(sample(obstacle=False)); m.sample(sample(1.1, 0.01, False))
    assert m.report()['metrics']['minimum_geometric_clearance_m'] is None


def test_path_change_proxy_separate_from_invocations_and_task_change():
    m = Metrics({}); p = [[0, 0], [1, 0]]
    m.path(p, 1, 1, 1); m.path(p, 1.1, 1.1, 1)
    m.path([[0.1, 0], [1, 0]], 1.2, 1.2, 1); m.path([[1, 0], [2, 0]], 1.3, 1.3, 2)
    r = m.report()['metrics']
    assert r['path_publications'] == 4 and r['observed_same_task_path_changes'] == 1
    assert r['planner_invocation_count'] is None


@pytest.mark.parametrize('stamp,now', [(0, 1), (1, 9), (9, 1), (float('nan'), 1)])
def test_bad_path_time(stamp, now):
    with pytest.raises(ObstacleError): Metrics({}).path([[0, 0], [1, 1]], stamp, now, 1)


def test_nav2_recovery_separate_from_v4_decision():
    m = Metrics({}); m.bt_events = [{'node_name': 'Spin', 'previous_status': 'IDLE', 'current_status': 'RUNNING'},
                                   {'node_name': 'Spin', 'previous_status': 'RUNNING', 'current_status': 'SUCCESS'}]
    r = m.report({'status': 'FAILED', 'tasks': [{'attempts': [{'recovery_action': 'HALT'}]}]})
    assert r['nav2']['recovery_running_transitions'] == {'Spin': 1}
    assert r['v4']['task_level_decisions'] == {'HALT': 1}
    json_ready = copy.deepcopy(r)
    import json
    json.dumps(json_ready, allow_nan=False)


def test_v4_wall_duration_is_separate_from_sim_window():
    r = Metrics({}).report({'created_at': '2026-11-09T01:00:00+08:00', 'ended_at': '2026-11-09T01:02:00+08:00'})
    assert r['v4']['reported_run_wall_duration_s'] == 120
    assert r['metrics']['observation_sim_duration_s'] is None
