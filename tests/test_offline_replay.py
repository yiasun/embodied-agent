"""Offline numerical, evidence-integrity and visualization-data regressions."""
import copy
import json
import math
import re
from pathlib import Path

import pytest

from replay import analysis as a
from replay import build as b
from replay.public import load_public
from replay_fixtures import synthetic_bundle


def sample(x=0, y=0, t=0, **extra):
    return {'robot': [x,y,0], 'sim_time': t, 'frame': 'gazebo_world',
            'task_id': 3, 'segment_valid': True, 'source_row': 0, **extra}


@pytest.fixture(scope='module')
def dataset():
    return load_public()


@pytest.fixture
def binding_root(tmp_path, monkeypatch):
    return synthetic_bundle(tmp_path, monkeypatch)


@pytest.mark.parametrize('value', [[True,0], [math.nan,0], [math.inf,0], [0], '0,0'])
def test_nonfinite_or_wrong_geometry(value):
    with pytest.raises(ValueError):
        a.project(value, [])


@pytest.mark.parametrize('mutation', [
    {'sim_time': math.nan}, {'sim_time': True}, {'robot': [1,2,math.inf]}, {'frame': 'odom'}])
def test_invalid_samples_rejected(mutation):
    with pytest.raises(ValueError):
        a.task_samples([sample(), sample(1,t=.1, **mutation)] if 'sim_time' not in mutation
                       else [sample(), {**sample(1,t=.1), **mutation}])


@pytest.mark.parametrize('times', [[0,0], [.1,0]])
def test_non_increasing_time_rejected(times):
    with pytest.raises(ValueError):
        a.task_samples([sample(t=t) for t in times])


def test_task_isolation_and_row_lineage():
    rows=a.task_samples([sample(task_id=2),sample(1,t=.1),sample(2,t=.2),sample(3,t=.3,task_id=4)])
    assert [r['source_row'] for r in rows]==[1,2]


@pytest.mark.parametrize('changes', [{'segment_valid':False}, {'task_id':4}, {'sim_time':1}])
def test_invalid_interval_not_bridged(changes):
    rows=[sample(), {**sample(1,t=.1), **changes}]
    assert not a.interval_valid(*rows)
    assert a.summarize(rows)['observed_path_length_m']==0
    with pytest.raises(ValueError):a.reference_segments(rows)


def test_exact_segment_projection_not_nearest_sample():
    ref=a.reference_segments([sample(),sample(2,t=.1)])
    assert a.project([1,.3],ref)['distance_m']==pytest.approx(.3)
    assert a.project([1,.3],ref)['s_ref_m']==pytest.approx(1)
    assert a.project([-1,0],ref)['point']==[0,0]
    assert a.project([3,0],ref)['point']==[2,0]


@pytest.mark.parametrize('start,end,expected', [(0,2,(.25,.75)),(2,0,(.25,.75)),(1,1,(0,1)),(3,3,None),(0,.5,None)])
def test_spatial_clip(start,end,expected):
    assert a.clipped_fraction(start,end,.5,1.5)==expected


def test_invalid_spatial_window():
    with pytest.raises(ValueError):a.clipped_fraction(0,1,1,1)


def test_speed_is_duration_weighted_with_boundary_clip_and_revisits():
    rows=[sample(0),sample(2,t=.2),sample(1,t=.4)]
    result=a.spatial_speed(rows,.5,1.5)
    assert result['duration_sim_s']==pytest.approx(.2)
    assert result['distance_m']==pytest.approx(1.5)
    assert result['mean_speed_m_s']==pytest.approx(7.5)
    assert a.spatial_speed(rows,4,5)['mean_speed_m_s'] is None


@pytest.mark.parametrize('robot,yaw,expected', [([1,0,0],0,.58),([0,1,0],math.pi/2,.58),([0,0,0],0,-.42)])
def test_circle_box_signed_clearance(robot,yaw,expected):
    assert a.clearance(robot,.22,{'position':[0,0,.6],'yaw':yaw,'size':[.4,.4,1.2]})==pytest.approx(expected)


def test_reversal_and_low_speed():
    rows=[sample(0,t=i*.1,obstacle={'position':[0,y,0]}) for i,y in enumerate([0,.2,.4,.4,.2,0,.2])]
    assert a.reversals(rows)==2
    assert a.summarize(rows)['longest_observed_low_speed_sim_s']==pytest.approx(.6)


def test_frames_no_extrapolation_across_gap():
    rows=[sample(),sample(.01,t=.1),sample(1,t=1),sample(1.1,t=1.1)]
    frames=a.frames(rows,a.reference_segments(rows))
    assert frames[0]['speed_m_s'] is None
    assert frames[2]['speed_m_s'] is None and not frames[2]['connect_previous']
    assert frames[2]['s_own_m']==pytest.approx(.01)


def test_stored_gap_tamper_rejected():
    rows=[sample(obstacle={'position':[1,0,.6],'size':[.4,.4,1.2],'yaw':0},geometric_clearance_m=100),sample(.1,t=.1)]
    with pytest.raises(ValueError,match='geometry'):a.frames(rows,a.reference_segments(rows))


@pytest.mark.parametrize('field', ['plan_id','run_id','provenance_id','scene_id','scene_assets'])
def test_evidence_binding_rejected(field, binding_root):
    stem,run=b.SPECS['D1']
    evaluation=b.read(binding_root/'logs/reliability'/(stem+'.json'))
    report=b.read(binding_root/'logs'/('task_v4_'+run+'.json'))
    evaluation[field]='invalid'
    with pytest.raises(ValueError):b.check_binding(evaluation,report)


@pytest.mark.parametrize('target', ['tasks','sample_goal','source','plan'])
def test_evidence_semantics_tamper(target, binding_root):
    stem,run=b.SPECS['D1']
    evaluation=b.read(binding_root/'logs/reliability'/(stem+'.json'))
    report=b.read(binding_root/'logs'/('task_v4_'+run+'.json'))
    if target=='tasks':evaluation['v4']['tasks'][2]['status']='FAILED'
    if target=='sample_goal':evaluation['samples'][0]['goal']['x']=999
    if target=='source':report['source']['model']='tampered'
    if target=='plan':report['plan']['steps'][0]['x']=999
    with pytest.raises(ValueError):b.check_binding(evaluation,report)


@pytest.mark.parametrize('target', ['raw_hash','raw_samples','lease_id','native_token','asset_identity'])
def test_full_pipeline_rejects_altered_links(monkeypatch,target,binding_root):
    original=b.read
    def changed(path):
        value=original(path)
        if 'D3_' in path.name and path.name.endswith('_metrics_final.json'):
            if target=='raw_hash':value['input_evaluation_sha256']='0'*64
            if target=='raw_samples':value['samples'][0]['robot'][0]+=1
            if target=='asset_identity':value['scene_assets']['revision']='9.0.0'
        if path.name==b.SPECS['D3'][0]+'.json':
            if target=='lease_id':value['lease_id']='invalid'
            if target=='native_token':value['motion_trace'][0]['token']='invalid'
        return value
    monkeypatch.setattr(b,'read',changed)
    with pytest.raises(ValueError):b.load_evidence(binding_root)


def test_real_derived_results(dataset):
    d1,d3=dataset['experiments']['D1'],dataset['experiments']['D3']
    assert dataset['comparison']['maximum_sampled_deviation_m']==pytest.approx(.3866808495)
    assert d1['task3']['spatial_speed']['mean_speed_m_s']==pytest.approx(.4603114488)
    assert d3['task3']['spatial_speed']['mean_speed_m_s']==pytest.approx(.1441808810)
    assert dataset['comparison']['spatial_speed_reduction_percent']==pytest.approx(68.67753747)
    assert d3['task3']['observed_y_reversals']==3
    assert d3['task3']['minimum_sampled_clearance_m']==pytest.approx(.3529465546)
    assert d3['task3']['observed_duration_sim_s']==pytest.approx(20.394)
    assert d3['task3']['longest_observed_low_speed_sim_s']==pytest.approx(1.893)


@pytest.mark.parametrize('key',['plan_id','scene_id','scene_assets'])
def test_comparison_requires_same_trusted_task_and_space(key):
    baseline={'plan_id':'a','scene_id':'inspection_world','scene_assets':{'revision':'1'}}
    changed={**baseline,key:'different'}
    with pytest.raises(ValueError,match='comparison binding'):
        b.check_comparable({'D1':(baseline,None,None),'D3':(changed,None,None)})


@pytest.mark.parametrize('name,count,quality', [('D1',619,'INCOMPLETE'),('D2',127,'VALID_OBSERVATION_WINDOW'),('D3',378,'INCOMPLETE')])
def test_real_navigation_and_coverage_kept_distinct(dataset,name,count,quality):
    exp=dataset['experiments'][name]
    assert len(exp['frames'])==count
    assert exp['quality']['status']==quality
    assert exp['quality']['full_navigation_coverage']=='NOT_ASSERTED'
    assert [t['status'] for t in exp['tasks']]==['SUCCEEDED']*4
    assert exp['contact_status']=='NOT_MEASURED'
    assert exp['planner_invocation_count'] is None


def test_event_is_not_fabricated_robot_sample(dataset):
    d3=dataset['experiments']['D3'];event=d3['events'][-1]
    assert event['t']==pytest.approx(20.439)
    assert event['t']-d3['frames'][-1]['t']==pytest.approx(.045)
    assert event['robot'] is None
    assert 'clearance_m' not in event and 'speed_m_s' not in event
    assert event['reason']=='ROBOT_CLEARANCE_GUARD'
    assert d3['experiment_status']=='FAILED' and d3['navigation_status']=='SUCCEEDED'


def test_reference_projection_reproducible(dataset):
    for frame in dataset['experiments']['D3']['frames']:
        assert math.dist(frame['robot'][:2],frame['projection'])==pytest.approx(frame['deviation_m'])


def test_allowlist_removes_sensitive_source_fields(dataset):
    encoded=json.dumps(dataset)
    for forbidden in ['/home/', '/opt/', 'API_KEY', 'received_mono', 'plugin_path', 'world_session',
                      '597a5550d6164281a7031a28b41c85a0','3f00c421a02a4676b56c590dd81907ee',
                      '"token"', '"lease_id"', '"entity_name"', 'request_id']:
        assert forbidden not in encoded
    for source in dataset['sources']:
        assert Path(source['file']).name==source['file']
        assert re.fullmatch('[0-9a-f]{64}',source['sha256'])


def test_export_roundtrip_deterministic_and_embedding_safe(dataset,tmp_path):
    value=copy.deepcopy(dataset);value['limitations'].append('</script><secret>')
    b.write_dataset(value,tmp_path/'data')
    first=(tmp_path/'data/evidence.json').read_bytes()
    assert json.loads(first)==value
    assert '</script>' not in (tmp_path/'evidence.js').read_text()
    b.write_dataset(value,tmp_path/'data')
    assert first==(tmp_path/'data/evidence.json').read_bytes()


def test_checked_in_export_matches_sources(dataset):
    assert b.read(b.ROOT/'web/replay/data/evidence.json')==dataset
    assert b.read(b.ROOT/'web/replay/data/sources.json')==dataset['sources']


def test_private_raw_export_matches_public_when_available(dataset):
    if not (b.ROOT/'logs/reliability'/(b.SPECS['D1'][0]+'.json')).is_file():
        pytest.skip('Private original logs intentionally excluded from release')
    assert b.make_dataset()==dataset


def test_web_has_no_network_or_robot_control_dependencies():
    folder=b.ROOT/'web/replay'
    js=(folder/'app.js').read_text();html=(folder/'index.html').read_text()
    assert not re.search(r'\b(fetch|WebSocket|XMLHttpRequest|EventSource)\s*\(',js)
    assert 'connect-src \'none\'' in html
    assert 'https://' not in html and 'http://' not in html
    assert 'aria-label="观测窗口相对仿真时间"' in html or 'for="timeline"' in html
