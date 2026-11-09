import copy
import json
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock

import pytest

from planner import agent_planner, execute_plan
from planner.plan_identity import (PLAN_ID_SCHEME, LEGACY_AGENT_SCHEME, LEGACY_FILE_SCHEME,
    compute_plan_id, legacy_agent_id, legacy_file_id, validate_trusted_plan)
from planner.planner_v1 import build_plan, load_scene
from planner.provenance import load_source, make_source_record, validate_source_record
from planner.providers.base import ProviderResult
from planner.trace_audit import audit_artifacts
from day4.task_executor_v4 import main as v4_main, run_tasks


@pytest.fixture
def scene():
    return load_scene('turtlebot3_demo')


@pytest.fixture
def plan(scene):
    return build_plan(scene, ['A区', 'B区'])


def write_json(path, value, **kwargs):
    path.write_text(json.dumps(value, ensure_ascii=False, **kwargs), encoding='utf-8')
    return path


class Navigator:
    last_status = 'SUCCEEDED'
    last_reason = 'goal_reached'
    navigate = Mock(return_value=True)
    abort_active = Mock(return_value=True)
    destroy_node = Mock()


def execute_fake(tmp_path, plan, *, source=None, run_id='synthetic-run'):
    goals = [(s['x'], s['y']) for s in plan['steps']]
    path = tmp_path / (run_id + '.json')
    code, report = run_tasks(goals, Navigator, path, execution_id=run_id,
        plan_id=compute_plan_id(plan), trusted_plan=plan, source=source)
    return path, code, report


def test_generation_save_reload_validation_and_execution_same_identity(tmp_path, scene):
    result = agent_planner.plan_instruction('先去A区，再去B区', scene, env={})
    agent_planner.save_result(result, tmp_path/'bundle')
    plan_path = tmp_path/'bundle/plan.json'
    loaded, coordinates = execute_plan.read_and_validate_plan(plan_path)
    source = load_source(tmp_path/'bundle/provenance.json', loaded)
    path, code, report = execute_fake(tmp_path, loaded, source=source)
    ids = [result.provenance['plan_id'], compute_plan_id(result.plan), compute_plan_id(loaded), report['plan_id']]
    assert len(set(ids)) == 1 and code == 0
    execute_plan.verify_execution_report(path, 'synthetic-run', ids[0], coordinates, expected_source=source)
    assert report['source']['provider'] == 'rule'
    assert report['source']['input_tokens'] is None


@pytest.mark.parametrize('options', [{'indent':2}, {'indent':4, 'sort_keys':True}, {'separators':(',', ':')}])
@pytest.mark.parametrize('ending', ['', '\n', '\r\n'])
def test_json_whitespace_and_paths_do_not_change_id(tmp_path, plan, options, ending):
    path = tmp_path/'different-filename.json'
    path.write_text(json.dumps(plan, ensure_ascii=True, **options)+ending)
    loaded, _ = execute_plan.read_and_validate_plan(path)
    assert compute_plan_id(loaded) == compute_plan_id(plan)


def test_timestamps_do_not_change_id(plan):
    other = copy.deepcopy(plan); other['created_at'] = '2020-01-01T00:00:00Z'
    assert compute_plan_id(other) == compute_plan_id(plan)


def test_equivalent_numbers_do_not_change_id(plan):
    a = copy.deepcopy(plan); b = copy.deepcopy(plan)
    a['steps'][0]['x'] = 1; b['steps'][0]['x'] = 1.0
    a['steps'][0]['y'] = -0.0; b['steps'][0]['y'] = 0
    assert compute_plan_id(a) == compute_plan_id(b)


@pytest.mark.parametrize('change', ['scene', 'order', 'location', 'coordinate', 'tiny_coordinate', 'repeat'])
def test_semantic_changes_change_identity(plan, change):
    other = copy.deepcopy(plan)
    if change == 'scene': other['scene_id'] = 'other_scene'
    if change == 'order':
        other['steps'].reverse()
        for i, s in enumerate(other['steps'], 1): s['step_id'] = i
    if change == 'location': other['steps'][0]['location'] = 'B区'
    if change == 'coordinate': other['steps'][0]['x'] += 1
    if change == 'tiny_coordinate': other['steps'][0]['x'] += 0.00001
    if change == 'repeat':
        step = copy.deepcopy(other['steps'][0]); step['step_id'] = 3; other['steps'].append(step)
    assert compute_plan_id(other) != compute_plan_id(plan)


@pytest.mark.parametrize('field,value', [('action','WAIT'),('step_id',True),('x',float('nan'))])
def test_illegal_plan_cannot_be_identified(plan, field, value):
    plan['steps'][0][field] = value
    with pytest.raises(ValueError): compute_plan_id(plan)


@pytest.mark.parametrize('bad_id', ['plan', '', 'A'*64, 'f'*63, 'g'*64, '../escape', 'f'*64])
def test_invalid_or_mismatched_id_rejected_before_ros(tmp_path, plan, bad_id):
    path = write_json(tmp_path/'plan.json', plan)
    with pytest.raises(SystemExit) as caught:
        v4_main(['--plan', str(path), '--plan-id', bad_id, '--execute'])
    assert caught.value.code != 0


def test_id_alone_cannot_skip_plan_validation(tmp_path):
    with pytest.raises(SystemExit): v4_main(['0','0','--plan-id','a'*64,'--execute'])
    factory = Mock()
    with pytest.raises(ValueError):
        run_tasks([(0,0)], factory, tmp_path/'report.json', plan_id='a'*64)
    factory.assert_not_called()


@pytest.mark.parametrize('mutate', ['coordinate', 'step', 'registry', 'claim'])
def test_executor_revalidates_before_creating_navigator(tmp_path, plan, mutate):
    original_id = compute_plan_id(plan)
    if mutate == 'coordinate': plan['steps'][0]['x'] += 0.00001
    if mutate == 'step': plan['steps'][0]['action'] = 'SHELL'
    if mutate == 'registry': plan['steps'][0]['location'] = 'C区'
    factory = Mock()
    with pytest.raises(ValueError):
        run_tasks([(s['x'],s['y']) for s in plan['steps']], factory, tmp_path/'report.json',
                  trusted_plan=plan, plan_id='a'*64 if mutate=='claim' else original_id)
    factory.assert_not_called(); assert not (tmp_path/'report.json').exists()


def test_valid_v4_plan_dry_run_no_robot(tmp_path, plan, monkeypatch):
    import day4.task_executor_v4 as executor
    run = Mock(side_effect=AssertionError('no ROS allowed'))
    monkeypatch.setattr(executor, 'run_tasks', run)
    path = write_json(tmp_path/'plan.json', plan)
    assert v4_main(['--plan',str(path),'--plan-id',compute_plan_id(plan)]) == 0
    run.assert_not_called()


def test_source_metadata_links_model_request_plan_and_runs(tmp_path, scene):
    metadata = ProviderResult(json.dumps({'steps':[{'action':'NAVIGATE','location':'A区'}]}),
        {'test':'response'}, 'qwen','reported-model','configured-model','request-123',20,8,28,1.5)
    provider = Mock(); provider.generate_intent.return_value = metadata
    result = agent_planner.plan_instruction('去A区',scene,provider=provider)
    agent_planner.save_result(result,tmp_path/'bundle')
    source = load_source(tmp_path/'bundle/provenance.json',result.plan)
    p1,_,r1 = execute_fake(tmp_path,result.plan,source=source,run_id='run-1')
    p2,_,r2 = execute_fake(tmp_path,result.plan,source=source,run_id='run-2')
    assert r1['plan_id'] == r2['plan_id'] == result.provenance['plan_id']
    assert r1['run_id'] != r2['run_id']
    assert r1['source']['request_id']=='request-123' and r1['source']['provider']=='qwen'
    assert r1['source']['total_tokens']==28 and r1['source']['duration_seconds']==1.5


def test_model_response_cannot_override_reserved_identity_fields(scene):
    class ForgedResult(ProviderResult):
        def as_dict(self):
            return {'plan_id':'forged','run_id':'forged','provider':'forged'}
    response = ForgedResult(json.dumps({'steps':[{'action':'NAVIGATE','location':'A区'}]}),
        {'plan_id':'forged','provider':'forged'},'qwen','model','model','request',None,None,None,0.5)
    provider = Mock(); provider.generate_intent.return_value = response
    result = agent_planner.plan_instruction('去A区',scene,provider=provider)
    assert result.provenance['plan_id']==compute_plan_id(result.plan)
    assert result.provenance['provider']=='qwen' and 'run_id' not in result.provenance


@pytest.mark.parametrize('field,value', [('provider','ollama'),('model','changed'),('request_id','changed'),
                                       ('total_tokens',999),('plan_id','a'*64),('run_id','injected')])
def test_tampered_metadata_rejected(tmp_path, scene, field, value):
    result = agent_planner.plan_instruction('去A区',scene,env={})
    agent_planner.save_result(result,tmp_path/'bundle')
    path=tmp_path/'bundle/provenance.json'
    data=json.loads(path.read_text()); data[field]=value; write_json(path,data)
    # Top-level run_id is never copied to source or report, even in local metadata.
    if field=='run_id':
        source=load_source(path,result.plan)
        assert 'run_id' not in source
    else:
        with pytest.raises(ValueError): load_source(path,result.plan)


@pytest.mark.parametrize('change',['id','plan','source','source_missing'])
def test_report_association_rejects_tampering(tmp_path, scene, change):
    result=agent_planner.plan_instruction('先去A区，再去B区',scene,env={})
    agent_planner.save_result(result,tmp_path/'bundle')
    source=load_source(tmp_path/'bundle/provenance.json',result.plan)
    path,_,report=execute_fake(tmp_path,result.plan,source=source)
    if change=='id':report['plan_id']='b'*64
    if change=='plan':report['plan']['steps'].reverse()
    if change=='source':report['source']['model']='changed'
    if change=='source_missing':report['source']=None
    write_json(path,report)
    coordinates=[str(s[k]) for s in result.plan['steps'] for k in ('x','y')]
    with pytest.raises(ValueError):
        execute_plan.verify_execution_report(path,'synthetic-run',compute_plan_id(result.plan),coordinates,expected_source=source)


def test_legacy_ids_explicitly_mapped_not_overwritten(tmp_path, plan):
    metadata={'provider':'qwen','model':'qwen-plus','requested_model':'qwen-plus','request_id':'request',
              'input_tokens':3,'output_tokens':4,'total_tokens':7,'duration_seconds':0.3,
              'plan_id':legacy_agent_id(plan)}
    response=write_json(tmp_path/'response.json',{'plan':plan,'provenance':metadata},indent=2)
    plan_path=write_json(tmp_path/'plan.json',plan,indent=2)
    before=(response.read_bytes(),plan_path.read_bytes())
    with pytest.raises(ValueError):load_source(response,plan)
    source=load_source(response,plan,allow_legacy=True)
    assert source['plan_id']==compute_plan_id(plan)
    assert source['legacy_reference']=={'scheme':LEGACY_AGENT_SCHEME,'plan_id':metadata['plan_id']}
    association=audit_artifacts(response,plan_path)
    assert association['plan_id']==source['plan_id']
    assert association['legacy_plan_file']['scheme']==LEGACY_FILE_SCHEME
    assert before==(response.read_bytes(),plan_path.read_bytes())


def test_large_exact_integer_float_equivalence(plan):
    other = copy.deepcopy(plan)
    plan['steps'][0]['x'] = 2**60
    other['steps'][0]['x'] = float(2**60)
    assert compute_plan_id(plan) == compute_plan_id(other)


def test_stdout_response_can_be_used_as_versioned_source(tmp_path, scene, capsys):
    assert agent_planner.main(['去A区'],env={}) == 0
    data = json.loads(capsys.readouterr().out)
    path = write_json(tmp_path/'response.json',data)
    source = load_source(path,data['plan'])
    assert source['plan_id'] == data['provenance']['plan_id']
    assert data['provenance']['provenance_version'] == '2.0'


def test_new_provenance_survives_format_and_timestamp_changes(tmp_path, scene):
    result = agent_planner.plan_instruction('去A区',scene,env={})
    agent_planner.save_result(result,tmp_path/'bundle')
    other = copy.deepcopy(result.plan);other['created_at']='2000-01-01T00:00:00Z'
    source = load_source(tmp_path/'bundle/provenance.json',other)
    assert source['plan_id'] == result.provenance['plan_id']


def test_parent_child_source_handoff_and_reporting(tmp_path, scene, monkeypatch):
    result = agent_planner.plan_instruction('先去A区，再去B区',scene,env={})
    agent_planner.save_result(result,tmp_path/'bundle')
    monkeypatch.setattr(execute_plan,'ROOT',tmp_path)
    captured = {}
    def fake_child(command):
        def arg(name): return command[command.index(name)+1]
        assert command.index('--source-id') < command.index('--ros-args')
        plan = json.loads(Path(arg('--plan')).read_text())
        source = load_source(Path(arg('--provenance')),plan)
        assert arg('--source-id') == source['provenance_id']
        goals = [(s['x'],s['y']) for s in plan['steps']]
        code,report = run_tasks(goals,Navigator,Path(arg('--report')),trusted_plan=plan,
            plan_id=arg('--plan-id'),execution_id=arg('--execution-id'),source=source)
        captured.update(report)
        return code
    monkeypatch.setattr(execute_plan,'run_executor',fake_child)
    assert execute_plan.main(['--plan',str(tmp_path/'bundle/plan.json'),'--execute']) == 0
    assert captured['source']['provider']=='rule'
    assert captured['plan_id']==result.provenance['plan_id']
    assert captured['source']['provenance_id']==result.provenance['provenance_id']


def test_changed_plan_at_child_boundary_rejected_before_ros(tmp_path, scene, monkeypatch):
    result = agent_planner.plan_instruction('先去A区，再去B区',scene,env={})
    plan_path = write_json(tmp_path/'plan.json',result.plan)
    monkeypatch.setattr(execute_plan,'ROOT',tmp_path)
    def fake_child(command):
        mutated = copy.deepcopy(result.plan)
        mutated['steps'].reverse()
        for i,s in enumerate(mutated['steps'],1):s['step_id']=i
        write_json(plan_path,mutated)
        with pytest.raises(SystemExit) as caught:v4_main(command[2:])
        return caught.value.code
    monkeypatch.setattr(execute_plan,'run_executor',fake_child)
    assert execute_plan.main(['--plan',str(plan_path),'--execute']) != 0


@pytest.mark.parametrize('bad_source_id',['invalid','a'*64])
def test_bad_source_id_stops_before_ros(tmp_path, scene, bad_source_id):
    result = agent_planner.plan_instruction('去A区',scene,env={})
    agent_planner.save_result(result,tmp_path/'bundle')
    with pytest.raises(SystemExit):
        v4_main(['--plan',str(tmp_path/'bundle/plan.json'),'--source-id',bad_source_id,'--execute'])


def test_current_real_artifacts_read_only_association():
    root = Path(__file__).resolve().parents[1]
    response = root/'plans/real_api/qwen_response.json'
    plan = root/'plans/real_api/qwen_plan.json'
    report = root/'logs/task_v4_887222def3fd4c908583963a130c0766.json'
    if not all(p.exists() for p in (response,plan,report)):
        pytest.skip('Local historical artifacts are not part of a fresh checkout')
    before = [p.read_bytes() for p in (response,plan,report)]
    result = audit_artifacts(response,plan,report)
    assert result['plan_id'] == compute_plan_id(json.loads(plan.read_text()))
    assert result['source']['legacy_reference']['plan_id'] == 'acce4a922afc0b9fa6f2446f2e1e908ab790c72750097612a2b9cdf06c081ceb'
    assert result['historical_execution']['original_plan_id'] == '8b24495133f3cfe27c3de19dab7c3b4aba32fc5afeabbd64e7801f218190be9f'
    assert result['source']['request_id'] == 'chatcmpl-152b76af-0d80-9c2e-a04f-2b854f0c82c7'
    assert result['historical_execution']['run_id'] == '887222def3fd4c908583963a130c0766'
    assert result['navigation_replayed'] is False
    assert before == [p.read_bytes() for p in (response,plan,report)]
