import copy
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from unittest.mock import Mock

import pytest

from planner import agent_planner
from planner.intent_validator import validate_intent
from planner.planner_v1 import load_scene
from planner.providers import PlanningError
from planner.providers.transport import HTTPResponse

ROOT=Path(__file__).resolve().parents[1]
KEY='synthetic-planner-test-credential'


def transport_for(content):
    data={'id':'mock-id','model':'mock-model','choices':[{'finish_reason':'stop','message':{'content':content}}]}
    transport=Mock();transport.post.return_value=HTTPResponse(200,json.dumps(data).encode())
    return transport


def cloud_env():
    return {'LLM_PROVIDER':'qwen','QWEN_API_KEY':KEY}


@pytest.mark.parametrize('count',[1,2,20])
@pytest.mark.parametrize('provider',['rule','qwen','ollama'])
def test_successful_planning_uses_registry(count,provider):
    scene=load_scene('turtlebot3_demo')
    intent={'steps':[{'action':'NAVIGATE','location':'A区'}]*count}
    transport=transport_for(json.dumps(intent))
    env={**cloud_env(),'LLM_PROVIDER':provider,'OLLAMA_MODEL':'already-installed:test'}
    result=agent_planner.plan_instruction('，'.join(['去A区']*count),scene,env=env,transport=transport)
    assert len(result.plan['steps'])==count
    assert all(s['x']==0.419 and s['y']==-1.822 for s in result.plan['steps'])
    assert result.provenance['provider']==provider
    assert result.provenance['input_tokens'] is None
    if provider=='rule':transport.post.assert_not_called()
    else:transport.post.assert_called_once()


@pytest.mark.parametrize('content,expected',[
    ('not JSON','INVALID_JSON'), ('```json\n{"steps":[]}\n```','INVALID_JSON'),
    ('{"steps":[],"steps":[]}','INVALID_JSON'), ('NaN','INVALID_JSON'),
    ('[]','INVALID_INTENT_SCHEMA'), ('null','INVALID_INTENT_SCHEMA'),
    ('{}','INVALID_INTENT_SCHEMA'), ('{"steps":[]}','INVALID_INTENT_SCHEMA'),
    (json.dumps({'steps':[{'action':'NAVIGATE','location':'C区'}]}),'UNKNOWN_LOCATION'),
    (json.dumps({'steps':[{'action':'SHELL','location':'A区'}]}),'ILLEGAL_ACTION'),
    (json.dumps({'steps':[{'action':'NAVIGATE','location':'A区','x':0.419}]}),'INVALID_INTENT_SCHEMA'),
    (json.dumps({'steps':[{'action':'NAVIGATE','location':'A区','y':-1.822}]}),'INVALID_INTENT_SCHEMA'),
    (json.dumps({'steps':[{'action':'NAVIGATE','location':'A区','timeout':60}]}),'INVALID_INTENT_SCHEMA'),
    (json.dumps({'steps':[{'action':'NAVIGATE','location':'A区'}],'ros_command':'anything'}),'INVALID_INTENT_SCHEMA'),
    (json.dumps({'steps':[{'action':'NAVIGATE','location':'A区'}]*21}),'INVALID_INTENT_SCHEMA'),
    ('{"steps":[{"action":true,"location":"A区"}]}','INVALID_INTENT_SCHEMA'),
    ('{"steps":[{"action":"NAVIGATE","location":[]}]}','INVALID_INTENT_SCHEMA'),
    ('{"steps":[{"location":"A区"}]}','INVALID_INTENT_SCHEMA'),
])
def test_illegal_intent_cannot_reach_builder(content,expected,monkeypatch):
    builder=Mock(side_effect=AssertionError('invalid intent reached builder'))
    monkeypatch.setattr(agent_planner,'build_plan',builder)
    transport=transport_for(content)
    with pytest.raises(PlanningError,match=expected):
        agent_planner.plan_instruction('去A区',load_scene('turtlebot3_demo'),env=cloud_env(),transport=transport)
    builder.assert_not_called();transport.post.assert_called_once()


@pytest.mark.parametrize('instruction',[None,'','   ',42,'x'*8001])
def test_invalid_instruction_never_calls_provider(instruction):
    transport=Mock()
    with pytest.raises(PlanningError,match='INVALID_INSTRUCTION'):
        agent_planner.plan_instruction(instruction,load_scene('turtlebot3_demo'),env=cloud_env(),transport=transport)
    transport.post.assert_not_called()


def test_rule_parse_error_does_not_switch_to_cloud():
    transport=Mock()
    with pytest.raises(PlanningError,match='RULE_PARSE_ERROR'):
        agent_planner.plan_instruction('等待5秒',load_scene('turtlebot3_demo'),env={},transport=transport)
    transport.post.assert_not_called()


def test_bad_scene_before_request():
    transport=Mock()
    with pytest.raises(PlanningError,match='INVALID_SCENE'):
        agent_planner.plan_instruction('去A区',{},env=cloud_env(),transport=transport)
    transport.post.assert_not_called()


def test_cloud_failure_never_invokes_rule(monkeypatch):
    from planner.providers.rule import RuleProvider
    fallback=Mock(side_effect=AssertionError('no fallback permitted'))
    monkeypatch.setattr(RuleProvider,'generate_intent',fallback)
    transport=Mock();transport.post.return_value=HTTPResponse(429,b'{"error":{"code":"insufficient_quota"}}')
    with pytest.raises(PlanningError,match='QUOTA_EXHAUSTED'):
        agent_planner.plan_instruction('去A区',load_scene('turtlebot3_demo'),env=cloud_env(),transport=transport)
    fallback.assert_not_called();transport.post.assert_called_once()


def test_saved_plan_compatible_with_existing_validator_and_hash(tmp_path):
    from planner.execute_plan import read_and_validate_plan
    result=agent_planner.plan_instruction('去A区',load_scene('turtlebot3_demo'),env={})
    directory=tmp_path/'bundle'
    agent_planner.save_result(result,directory)
    plan,coordinates=read_and_validate_plan(directory/'plan.json')
    metadata=json.loads((directory/'provenance.json').read_text())
    assert coordinates==['0.419','-1.822']
    from planner.plan_identity import compute_plan_id, PLAN_ID_SCHEME
    assert metadata['plan_id']==compute_plan_id(plan)
    assert metadata['plan_id_scheme']==PLAN_ID_SCHEME
    assert metadata['raw_response']['steps'][0]['location']=='A区'
    assert metadata['input_tokens'] is metadata['output_tokens'] is metadata['total_tokens'] is None
    assert 'instruction' not in metadata and metadata['instruction_sha256']


def test_existing_artifact_not_overwritten(tmp_path):
    directory=tmp_path/'existing';directory.mkdir();(directory/'plan.json').write_text('historical')
    result=agent_planner.plan_instruction('去A区',load_scene('turtlebot3_demo'),env={})
    with pytest.raises(PlanningError,match='ARTIFACT_WRITE_ERROR'):
        agent_planner.save_result(result,directory)
    assert (directory/'plan.json').read_text()=='historical'


def test_artifact_failure_removes_partial_plan(tmp_path,monkeypatch):
    result=agent_planner.plan_instruction('去A区',load_scene('turtlebot3_demo'),env={})
    actual=agent_planner.atomic_write_json
    def write(path,value):
        if path.name=='plan.json':raise OSError('disk unavailable')
        actual(path,value)
    monkeypatch.setattr(agent_planner,'atomic_write_json',write)
    with pytest.raises(PlanningError,match='ARTIFACT_WRITE_ERROR'):
        agent_planner.save_result(result,tmp_path/'bundle')
    assert not (tmp_path/'bundle').exists()


def test_exported_provenance_and_console_redact_credential(tmp_path,capsys):
    transport=transport_for('{"steps":[{"action":"NAVIGATE","location":"A区"}]}')
    data=json.loads(transport.post.return_value.body)
    data['id']=KEY;data['echo']=KEY
    transport.post.return_value=HTTPResponse(200,json.dumps(data).encode())
    assert agent_planner.main(['去A区','--output-dir',str(tmp_path/'bundle')],env=cloud_env(),transport=transport)==0
    console=capsys.readouterr()
    assert KEY not in console.out+console.err
    assert KEY not in (tmp_path/'bundle'/'provenance.json').read_text()


def test_cli_failure_no_artifact_or_secret(tmp_path,capsys):
    transport=Mock();transport.post.side_effect=RuntimeError(KEY)
    output=tmp_path/'bundle'
    assert agent_planner.main(['去A区','--output-dir',str(output)],env=cloud_env(),transport=transport)==1
    console=capsys.readouterr()
    assert KEY not in console.out+console.err
    assert 'NETWORK_ERROR' in console.err and not output.exists()


def test_cli_default_is_readonly_without_cloud_config(capsys):
    transport=Mock()
    assert agent_planner.main(['先去A区，再去B区'],env={},transport=transport)==0
    data=json.loads(capsys.readouterr().out)
    assert data['provenance']['provider']=='rule' and len(data['plan']['steps'])==2
    assert 'raw_response' not in data['provenance']
    transport.post.assert_not_called()


@pytest.mark.parametrize('entry',[['-m','planner.agent_planner'],['planner/agent_planner.py']])
def test_script_and_module_rule_entry(entry):
    result=subprocess.run([sys.executable,'-B',*entry,'去A区'],cwd=ROOT,
        env={'LLM_PROVIDER':'rule','PYTHONDONTWRITEBYTECODE':'1'},capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    assert json.loads(result.stdout)['provenance']['provider']=='rule'


def test_agent_cli_has_no_execute_option():
    with pytest.raises(SystemExit) as caught:
        agent_planner.main(['去A区','--execute'],env={})
    assert caught.value.code==2


def test_unexpected_internal_error_does_not_print_secret(monkeypatch,capsys):
    monkeypatch.setattr(agent_planner,'plan_instruction',Mock(side_effect=RuntimeError(KEY)))
    assert agent_planner.main(['去A区'],env={})==1
    output=capsys.readouterr()
    assert KEY not in output.out+output.err
    assert 'PLANNING_ERROR' in output.err


def test_provider_cannot_mutate_trusted_registry():
    from planner.providers.rule import RuleProvider
    class Mutator(RuleProvider):
        def generate_intent(self,instruction,scene):
            scene['waypoints']['A区']['x']=999
            return super().generate_intent(instruction,scene)
    scene=load_scene('turtlebot3_demo')
    result=agent_planner.plan_instruction('去A区',scene,provider=Mutator())
    assert scene['waypoints']['A区']['x']==0.419
    assert result.plan['steps'][0]['x']==0.419
