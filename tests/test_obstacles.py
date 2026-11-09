"""All scene/entity operations are fake. No Gazebo or ROS transport is opened."""
import copy
import json
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from day4.reporting import atomic_write_json
from planner import agent_planner
from planner.planner_v1 import load_scene
from reliability import lifecycle,obstacles
from reliability.safety import (ObstacleError,load_context,site_profile,select_position,
    check_placement,obstacle_sdf,verify_owned_model)

ROOT=Path(__file__).resolve().parents[1]


def route():
    # Synthetic route for unit tests only: SE room -> corridor -> NW room.
    points=[]
    for a,b in [((21.5,22.5),(21,26)),((21,26),(14,26)),((14,26),(14.5,29.5))]:
        for i in range(101):points.append([a[0]+(b[0]-a[0])*i/100,a[1]+(b[1]-a[1])*i/100])
    return points


@pytest.fixture
def context(tmp_path):
    scene=load_scene('inspection_world')
    result=agent_planner.plan_instruction('先去配电间，再去仓储间，接着去机房，最后去巡检站',scene,env={})
    agent_planner.save_result(result,tmp_path/'bundle')
    from planner.provenance import load_source
    source=load_source(tmp_path/'bundle/provenance.json',result.plan)
    report={'report_version':'2.0','run_id':'synthetic-run','plan':result.plan,
        'plan_id':result.provenance['plan_id'],'plan_id_scheme':result.provenance['plan_id_scheme'],
        'source':source,'status':'RUNNING','tasks':[{'id':i,'x':s['x'],'y':s['y'],
            'status':'RUNNING' if i==3 else 'SUCCEEDED' if i<3 else 'PENDING'}
            for i,s in enumerate(result.plan['steps'],1)]}
    atomic_write_json(tmp_path/'run.json',report)
    return load_context(tmp_path/'bundle/plan.json',tmp_path/'run.json',ROOT)


class FakeGazebo:
    robot_name='turtlebot3_waffle'
    def __init__(self,context):
        self.base=ET.fromstring(context['bundle']['paths']['world'].read_bytes())
        model=ET.SubElement(self.base.find('world'),'model',name=self.robot_name)
        ET.SubElement(model,'pose').text='21.5 22.5 0.01 0 0 0'
        ET.SubElement(model,'static').text='false'
        self.models={m.get('name'):copy.deepcopy(m) for m in self.base.find('world').findall('model')}
        self.ids={name:i for i,name in enumerate(self.models,1)}
        self.session='a'*64;self.create_calls=[];self.remove_calls=[]
        self.create_fault=None;self.remove_fault=None;self.create_ack=True;self.remove_ack=True
        self.create_without_effect=False;self.remove_without_effect=False
        self.robot=[21.5,22.5,0.01];self.pose_age=0;self.epoch_change_on_cleanup=False
    def discover(self):return {'session_id':self.session,'create':{'name':'/world/default/create/blocking'}}
    def entities(self):return {n:{'name':n,'id':self.ids[n]} for n in self.models}
    def world_sdf(self):
        root=ET.Element('sdf',version='1.9');world=ET.SubElement(root,'world',name='default')
        for model in self.models.values():world.append(copy.deepcopy(model))
        return ET.tostring(root,encoding='unicode')
    def poses(self):
        poses={}
        for name,model in self.models.items():
            pose=list(map(float,model.findtext('pose','0 0 0 0 0 0').split()))
            poses[self.ids[name]]={'position':self.robot if name==self.robot_name else pose[:3],
                                  'orientation':[0,0,0,1]}
        return poses,time.monotonic()-self.pose_age
    def create(self,name,sdf):
        self.create_calls.append((name,sdf))
        if not self.create_without_effect and self.create_ack:
            self.models[name]=ET.fromstring(sdf).find('model');self.ids[name]=1000
        if self.create_fault:raise self.create_fault
        return self.create_ack
    def remove(self,entity_id):
        self.remove_calls.append(entity_id)
        if self.remove_fault:raise self.remove_fault
        if not self.remove_without_effect:
            for name in list(self.models):
                if self.ids[name]==entity_id:del self.models[name]
        return self.remove_ack


class FakeObserver:
    def __init__(self,context):self.closed=False;self.context=context
    def sample(self,target):return route()
    def close(self):self.closed=True


def lease(context,mode='temporary'):
    return obstacles.new_record(context,mode)


def run(context,tmp_path,*,backend=None,mode='temporary',writer=atomic_write_json,sleep=lambda _:None,hold=0.001):
    backend=backend or FakeGazebo(context);observer=FakeObserver(context)
    code,record=lifecycle.run_lease(context,backend,lambda _:observer,lease(context,mode),tmp_path/'lease.json',
        hold_seconds=hold,writer=writer,sleep=sleep,verification_timeout=0.02)
    return backend,observer,code,record


@pytest.mark.parametrize('mode',['temporary','persistent'])
def test_map_and_actual_path_selection(context,mode):
    profile=site_profile(context['bundle'],mode)
    assert profile['x_band']==[15.5,17.3] and profile['corridor_y']==[25,27]
    selected=select_position(context['bundle'],mode,route(),[21.5,22.5])
    assert 15.5<=selected['position'][0]<=17.3
    assert selected['position'][1:]==[26,0.6]
    assert selected['path_sha256'] and selected['required_robot_separation']>2


@pytest.mark.parametrize('mode',['temporary','persistent'])
def test_sdf_has_real_matching_collision_and_visual(context,mode):
    record=lease(context,mode)
    record['selection']=select_position(context['bundle'],mode,route(),[21.5,22.5])
    sdf=obstacle_sdf(record['entity_name'],record['selection'],record['lease_id'])
    root=ET.Element('sdf');world=ET.SubElement(root,'world',name='default');world.append(ET.fromstring(sdf).find('model'))
    assert verify_owned_model(ET.tostring(root,encoding='unicode'),record)
    assert 'collision' in sdf and 'visual' in sdf and 'plugin' not in sdf


@pytest.mark.parametrize('pose,robot,error',[
    ([0,0,0.6],[21.5,22.5],'OUTSIDE_APPROVED_TEST_SITE'),
    ([16.4,26,1.0],[21.5,22.5],'INVALID_OBSTACLE_HEIGHT'),
    ([16.4,26,0.6],[16.4,26],'ROBOT_TOO_CLOSE'),
    ([16.4,26,0.6],[17,26],'ROBOT_TOO_CLOSE'),
])
def test_bad_position_or_robot_overlap_rejected(context,pose,robot,error):
    with pytest.raises(ObstacleError,match=error):check_placement(context['bundle'],'temporary',pose,route(),robot)


def test_path_not_crossing_site_rejected(context):
    with pytest.raises(ObstacleError):select_position(context['bundle'],'temporary',[[11.5,26],[14.5,22.5]],[11.5,26])


@pytest.mark.parametrize('mode',['temporary','persistent'])
def test_verified_create_and_automatic_cleanup(context,tmp_path,mode):
    backend,observer,code,record=run(context,tmp_path,mode=mode)
    assert code==0 and record['status']=='COMPLETED' and record['cleanup']=='CLEAN'
    assert len(backend.create_calls)==1 and backend.remove_calls==[1000]
    assert record['entity_id']==1000 and record['create_confirmed_at'] and record['cleanup_confirmed_at']
    assert record['entity_name'] not in backend.entities() and observer.closed
    assert record['plan_id']==context['plan_id'] and record['provenance_id']==context['provenance_id']
    assert json.loads((tmp_path/'lease.json').read_text())['cleanup']=='CLEAN'


def test_existing_obstacle_prevents_duplicate(context,tmp_path):
    backend=FakeGazebo(context);backend.models['ea_stage61_existing']=ET.Element('model',name='ea_stage61_existing');backend.ids['ea_stage61_existing']=999
    backend,_,code,record=run(context,tmp_path,backend=backend)
    assert code!=0 and not backend.create_calls and not backend.remove_calls
    assert record['cleanup']=='NOT_NEEDED'


@pytest.mark.parametrize('mutation',['missing_wall','moved_wall','shifted_collision','foreign_model','wrong_world_pose'])
def test_wrong_world_or_geometry_rejected_before_mutation(context,tmp_path,mutation):
    backend=FakeGazebo(context)
    if mutation=='missing_wall':del backend.models['south_wall']
    if mutation=='moved_wall':backend.models['south_wall'].find('pose').text='0 0 1 0 0 0'
    if mutation=='shifted_collision':ET.SubElement(backend.models['south_wall'].find('link/collision'),'pose').text='1 0 0 0 0 0'
    if mutation=='foreign_model':backend.models['foreign']=ET.Element('model',name='foreign');backend.ids['foreign']=99
    if mutation=='wrong_world_pose':backend.pose_age=2
    backend,_,code,_=run(context,tmp_path,backend=backend)
    assert code!=0 and not backend.create_calls and not backend.remove_calls


def test_robot_too_close_prevents_create(context,tmp_path):
    backend=FakeGazebo(context);backend.robot=[16.4,26,0.01]
    backend,_,code,_=run(context,tmp_path,backend=backend)
    assert code!=0 and not backend.create_calls


def test_create_timeout_with_entity_present_cleans_exact_owner(context,tmp_path):
    backend=FakeGazebo(context);backend.create_fault=ObstacleError('timeout')
    backend,_,code,record=run(context,tmp_path,backend=backend)
    assert code==1 and record['cleanup']=='CLEAN'
    assert len(backend.create_calls)==1 and backend.remove_calls==[1000]


def test_create_timeout_without_readback_is_unknown_not_clean(context,tmp_path):
    backend=FakeGazebo(context);backend.create_fault=ObstacleError('timeout');backend.create_without_effect=True
    backend,_,code,record=run(context,tmp_path,backend=backend)
    assert code==4 and record['cleanup']=='UNKNOWN' and record['entity_id'] is None
    assert len(backend.create_calls)==1 and not backend.remove_calls


def test_create_rejection_not_claimed_success(context,tmp_path):
    backend=FakeGazebo(context);backend.create_ack=False
    backend,_,code,record=run(context,tmp_path,backend=backend)
    assert code==1 and record['cleanup']=='CLEAN' and not backend.remove_calls


def test_acknowledged_create_without_entity_is_unknown(context,tmp_path):
    backend=FakeGazebo(context);backend.create_without_effect=True
    backend,_,code,record=run(context,tmp_path,backend=backend)
    assert code==4 and record['cleanup']=='UNKNOWN'
    assert len(backend.create_calls)==1


@pytest.mark.parametrize('interruption',[KeyboardInterrupt(),SystemExit(0),RuntimeError('synthetic')])
def test_interruption_or_exception_after_create_cleans(context,tmp_path,interruption):
    def sleep(_):raise interruption
    backend,_,code,record=run(context,tmp_path,sleep=sleep,hold=2)
    assert code==(130 if isinstance(interruption,(KeyboardInterrupt,SystemExit)) else 1)
    assert record['cleanup']=='CLEAN' and backend.remove_calls==[1000]


def test_remove_timeout_reports_unknown(context,tmp_path):
    backend=FakeGazebo(context);backend.remove_fault=ObstacleError('timeout')
    backend,_,code,record=run(context,tmp_path,backend=backend)
    assert code==4 and record['cleanup']=='UNKNOWN'
    assert backend.remove_calls==[1000]


def test_remove_ack_without_absence_reports_unknown(context,tmp_path):
    backend=FakeGazebo(context);backend.remove_without_effect=True
    backend,_,code,record=run(context,tmp_path,backend=backend)
    assert code==4 and record['cleanup']=='UNKNOWN'


def test_initial_log_failure_does_not_create(context,tmp_path):
    def fail(*args):raise OSError('disk failure')
    backend,_,code,record=run(context,tmp_path,writer=fail)
    assert code==1 and not backend.create_calls and record['journal_write_failed']


def test_log_failure_after_create_still_cleans(context,tmp_path):
    def writer(path,record):
        if record['status']=='ACTIVE':raise OSError('disk failure')
        atomic_write_json(path,record)
    backend,_,code,record=run(context,tmp_path,writer=writer)
    assert code==1 and record['cleanup']=='CLEAN' and backend.remove_calls==[1000]


def prepared_cleanup(context):
    backend=FakeGazebo(context);record=lease(context)
    record.update(selection=select_position(context['bundle'],'temporary',route(),[21.5,22.5]),
                  create_sent=True,create_ack=True,entity_id=1000,world_session_id=backend.session)
    backend.create(record['entity_name'],obstacle_sdf(record['entity_name'],record['selection'],record['lease_id']))
    return backend,record


@pytest.mark.parametrize('mutation',['server_restart','reused_id','foreign_token','wrong_size','wrong_pose','invisible'])
def test_cleanup_refuses_wrong_entity(context,mutation):
    backend,record=prepared_cleanup(context)
    if mutation=='server_restart':backend.session='b'*64
    if mutation=='reused_id':backend.ids[record['entity_name']]=1001
    model=backend.models[record['entity_name']]
    if mutation=='foreign_token':model.find('link').set('name','foreign')
    if mutation=='wrong_size':model.find('link/collision/geometry/box/size').text='10 10 10'
    if mutation=='wrong_pose':model.find('pose').text='0 0 0 0 0 0'
    if mutation=='invisible':model.find('link/visual/transparency').text='1'
    assert lifecycle.cleanup(backend,record,timeout=0.01) is False
    assert record['cleanup']=='UNKNOWN' and not backend.remove_calls


def test_explicit_cleanup_idempotent(context):
    backend,record=prepared_cleanup(context)
    assert lifecycle.cleanup(backend,record)
    assert lifecycle.cleanup(backend,record)
    assert backend.remove_calls==[1000]


def test_crashed_process_cleanup_without_saved_id_uses_token_readback(context):
    backend,record=prepared_cleanup(context);record['entity_id']=None;record['create_ack']=None
    assert lifecycle.cleanup(backend,record)
    assert backend.remove_calls==[1000] and record['entity_id']==1000


@pytest.mark.parametrize('record',[[],{}, {'lease_version':'1.0','lease_id':'a'*32,'entity_name':'south_wall','scene_id':'inspection_world'}])
def test_arbitrary_removal_journal_rejected(record):
    with pytest.raises(ObstacleError):lifecycle.validate_record(record)


def test_default_dry_run_no_backend_files_or_mutation(context,tmp_path,capsys):
    backend=Mock(side_effect=AssertionError('must not contact Gazebo'))
    code=obstacles.main(['inject','--plan',str(tmp_path/'bundle/plan.json'),
        '--run-report',context['report_path'],'--journal',str(tmp_path/'must-not-exist.json')],backend_factory=backend)
    assert code==0 and not (tmp_path/'must-not-exist.json').exists()
    backend.assert_not_called()
    assert json.loads(capsys.readouterr().out)['dry_run']


def test_cleanup_default_dry_run_no_transport(context,tmp_path):
    backend,record=prepared_cleanup(context);path=tmp_path/'journal.json';atomic_write_json(path,record)
    factory=Mock(side_effect=AssertionError('no transport'))
    assert obstacles.main(['cleanup','--journal',str(path)],backend_factory=factory)==0
    factory.assert_not_called();assert backend.remove_calls==[]


def test_probe_default_no_transport():
    factory=Mock(side_effect=AssertionError('no transport'))
    assert obstacles.main(['probe'],backend_factory=factory)==0
    factory.assert_not_called()


@pytest.mark.parametrize('seconds',['0','-1','nan','181'])
def test_invalid_duration_dry_run_rejected(context,tmp_path,seconds):
    factory=Mock()
    assert obstacles.main(['inject','--plan',str(tmp_path/'bundle/plan.json'),'--run-report',context['report_path'],
                           '--hold-seconds',seconds],backend_factory=factory)!=0
    factory.assert_not_called()


def test_finished_navigation_cannot_be_injected(context,tmp_path):
    report=json.loads(Path(context['report_path']).read_text());report['status']='SUCCEEDED'
    atomic_write_json(context['report_path'],report)
    backend,_,code,_=run(context,tmp_path)
    assert code!=0 and not backend.create_calls


def test_apply_cli_uses_fake_transport_and_cleans(context,tmp_path,monkeypatch,capsys):
    from contextlib import nullcontext
    monkeypatch.setattr(obstacles,'exclusive_test_lock',lambda:nullcontext())
    backend=FakeGazebo(context)
    code=obstacles.main(['inject','--plan',str(tmp_path/'bundle/plan.json'),'--run-report',context['report_path'],
        '--journal',str(tmp_path/'cli-lease.json'),'--hold-seconds','0.001','--apply'],
        backend_factory=lambda:backend,observer_factory=lambda _:FakeObserver(context))
    assert code==0 and backend.remove_calls==[1000]
    assert json.loads(capsys.readouterr().out)['cleanup']=='CLEAN'


def test_explicit_cleanup_cli_and_backend_unavailable(context,tmp_path,monkeypatch,capsys):
    from contextlib import nullcontext
    monkeypatch.setattr(obstacles,'exclusive_test_lock',lambda:nullcontext())
    backend,record=prepared_cleanup(context);path=tmp_path/'journal.json';atomic_write_json(path,record)
    factory=Mock(side_effect=RuntimeError('transport unavailable'))
    assert obstacles.main(['cleanup','--journal',str(path),'--apply'],backend_factory=factory)==4
    assert json.loads(path.read_text())['cleanup']=='UNKNOWN'
    assert backend.remove_calls==[]
    assert obstacles.main(['cleanup','--journal',str(path),'--apply'],backend_factory=lambda:backend)==0
    assert backend.remove_calls==[1000]


def test_lock_prevents_concurrent_owners(tmp_path,monkeypatch):
    monkeypatch.setattr(obstacles,'ROOT',tmp_path)
    with obstacles.exclusive_test_lock():
        with pytest.raises(ObstacleError,match='OWNS_LOCK'):
            with obstacles.exclusive_test_lock():pass


def test_existing_journal_not_overwritten(context,tmp_path,monkeypatch):
    from contextlib import nullcontext
    monkeypatch.setattr(obstacles,'exclusive_test_lock',lambda:nullcontext())
    path=tmp_path/'historical.json';path.write_text('original bytes')
    factory=Mock()
    assert obstacles.main(['inject','--plan',str(tmp_path/'bundle/plan.json'),'--run-report',context['report_path'],
        '--journal',str(path),'--apply'],backend_factory=factory)!=0
    assert path.read_text()=='original bytes';factory.assert_not_called()


@pytest.mark.parametrize('mutation',['plan','source','target'])
def test_active_report_tampering_stops_before_create(context,tmp_path,mutation):
    data=json.loads(Path(context['report_path']).read_text())
    if mutation=='plan':data['plan']['steps'][0]['x']=0
    if mutation=='source':data['source']['model']='changed'
    if mutation=='target':data['tasks'][2]['x']=0
    atomic_write_json(context['report_path'],data)
    backend,_,code,_=run(context,tmp_path)
    assert code!=0 and not backend.create_calls


def test_model_is_read_back_not_just_boolean_ack(context,tmp_path):
    backend=FakeGazebo(context);original=backend.create
    def create(name,sdf):
        answer=original(name,sdf)
        backend.models[name].find('link/visual').set('name','foreign-visual')
        return answer
    backend.create=create
    backend,_,code,record=run(context,tmp_path,backend=backend)
    assert code==4 and record['cleanup']=='UNKNOWN' and not backend.remove_calls


def test_robot_moving_into_site_at_last_check_blocks_creation(context,tmp_path):
    backend=FakeGazebo(context);original=backend.poses;calls=0
    def poses():
        nonlocal calls
        calls+=1
        if calls==3:backend.robot=[16.4,26,0.01]
        return original()
    backend.poses=poses
    backend,_,code,_=run(context,tmp_path,backend=backend)
    assert code!=0 and not backend.create_calls


def test_probe_baseline_scene_is_read_only(context,tmp_path,capsys):
    backend=FakeGazebo(context)
    assert obstacles.main(['probe','--live','--plan',str(tmp_path/'bundle/plan.json'),
        '--run-report',context['report_path']],backend_factory=lambda:backend)==0
    assert not backend.create_calls and not backend.remove_calls
    assert json.loads(capsys.readouterr().out)['live_static_scene_matches_formal_assets']


def test_nonfinite_robot_final_pose_cannot_create(context,tmp_path):
    backend=FakeGazebo(context);original=backend.poses;calls=0
    def poses():
        nonlocal calls
        calls+=1
        if calls==3:backend.robot=[float('nan'),26,0.01]
        return original()
    backend.poses=poses
    backend,_,code,record=run(context,tmp_path,backend=backend)
    assert code!=0 and not backend.create_calls


def test_read_only_active_run_lookup_rejects_ambiguity(context,tmp_path):
    from reliability.safety import find_active_report
    plan=tmp_path/'bundle/plan.json';logs=tmp_path/'logs';logs.mkdir()
    with pytest.raises(ObstacleError):find_active_report(plan,logs,ROOT)
    data=json.loads(Path(context['report_path']).read_text())
    atomic_write_json(logs/'task_v4_one.json',data)
    assert find_active_report(plan,logs,ROOT)==(logs/'task_v4_one.json').resolve()
    atomic_write_json(logs/'task_v4_two.json',{**data,'run_id':'second'})
    with pytest.raises(ObstacleError):find_active_report(plan,logs,ROOT)


def test_explicit_target_step_refuses_other_navigation_phase(context,tmp_path):
    record=lease(context);record['target_step']=2
    backend=FakeGazebo(context)
    code,result=lifecycle.run_lease(context,backend,lambda _:FakeObserver(context),record,tmp_path/'lease.json')
    assert code!=0 and not backend.create_calls and result['cleanup']=='NOT_NEEDED'


def test_render_scene_absence_alone_not_proof_of_physics_cleanup(context):
    backend,record=prepared_cleanup(context)
    actual=backend.entities
    backend.entities=lambda:{k:v for k,v in actual().items() if k!=record['entity_name']}
    assert lifecycle.cleanup(backend,record) is False
    assert record['cleanup']=='UNKNOWN' and not backend.remove_calls
