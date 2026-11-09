"""Fake ROS messages/executors and Gazebo only; never import native ROS bindings."""
import copy
import json
import threading
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from test_obstacles import context, FakeGazebo, FakeObserver, route
from reliability import observer as module, lifecycle, moving
from reliability.observer import NavigationObserver
from reliability.safety import ObstacleError, active_task, bound_report
from reliability.trajectory import profile
from reliability.obstacles import new_record


class Clock:
    def __init__(self): self.wall=100.0; self.sim=10.0; self.events=[]
    def monotonic(self): return self.wall
    def spin_once(self,node,timeout_sec):
        self.wall+=timeout_sec;self.sim+=timeout_sec
        if self.events:self.events.pop(0)()


def message(points=None,stamp=10.0,frame='map'):
    if points is None:points=[[21.5,22.5],[14.5,29.5]]
    sec=int(stamp)
    return NS(header=NS(frame_id=frame,stamp=NS(sec=sec,nanosec=round((stamp-sec)*1e9))),
              poses=[NS(pose=NS(position=NS(x=x,y=y))) for x,y in points])


@pytest.fixture
def observed(context,monkeypatch):
    clock=Clock();monkeypatch.setattr(module,'time',NS(monotonic=clock.monotonic))
    obs=NavigationObserver.__new__(NavigationObserver)
    obs.bundle=context['bundle'];obs.map=None;obs.path=None;obs.owned_context=False
    obs._init_observation();obs.bind_context(context)
    obs.node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=round(clock.sim*1e9))),
                context=object(),destroy_node=Mock())
    obs.rclpy=NS(ok=lambda:True,spin_once=clock.spin_once)
    monkeypatch.setattr(module,'check_map_message',Mock(return_value=True))
    return obs,clock,active_task(context)


def update_report(context,phase=None,terminal=None):
    p=Path(context['report_path']);d=json.loads(p.read_text())
    if terminal:d['status']=terminal
    if phase:
        for t in d['tasks']:
            t['status']='RUNNING' if t['id']==phase else 'SUCCEEDED' if t['id']<phase else 'PENDING'
    p.write_text(json.dumps(d))


def motion_record(context):
    r=new_record(context,'motion');r['target_step']=3;r['motion']=profile(context['bundle'])
    r['motion']['plugin_path']='/tmp/fake_motion.so';return r


def test_candidate_received_before_sample_is_used(observed):
    obs,clock,target=observed;obs._receive_map(object());obs._receive_path(message())
    assert obs.sample(target)[-1]==[14.5,29.5]
    assert obs.diagnostics()['statistics']['paths_received']==1


def test_two_publishers_do_not_overwrite_matching_candidate(observed):
    obs,clock,target=observed;obs._receive_map(object())
    obs._receive_path(message()) # planner_server
    unrelated=message([[21.5,22.5],[21.5,26]]) # route_server
    obs._receive_path(unrelated)
    assert obs.path[0] is unrelated # D1 latest-message view remains intact
    assert obs.sample(target)[-1]==[14.5,29.5]
    diag=obs.diagnostics()['sample_history'][-1]
    assert diag['selected_sequence']==1 and diag['rejection_checks']['PATH_TARGET_MISMATCH']==1


@pytest.mark.parametrize('fault,reason',[
    ('map','MAP_NOT_RECEIVED'),('path','PATH_NOT_RECEIVED'),
    ('wall','PATH_WALL_EXPIRED'),('sim','PATH_SIM_TIME_MISMATCH'),
    ('goal','PATH_TARGET_MISMATCH'),('frame','PATH_FRAME_MISMATCH'),('nan','NONFINITE_PATH')])
def test_failure_diagnostics_distinguish_causes(observed,fault,reason):
    obs,clock,target=observed
    if fault!='map':obs._receive_map(object())
    if fault!='path':
        m=message()
        if fault=='sim':m=message(stamp=1)
        if fault=='goal':m=message([[21.5,22.5],[21.5,22.5]])
        if fault=='frame':m=message(frame='odom')
        if fault=='nan':m.poses[0].pose.position.x=float('nan')
        obs._receive_path(m)
        if fault=='wall':clock.wall+=2.01
    with pytest.raises(ObstacleError,match='FRESH_MATCHING_NAV2_PATH_NOT_OBSERVED'):obs.sample(target)
    assert obs.diagnostics()['sample_history'][-1]['rejection_checks'][reason]>0
    json.dumps(obs.diagnostics(),allow_nan=False)


def test_late_volatile_subscription_cannot_recover_already_published_path(observed):
    obs,clock,target=observed;obs._receive_map(object())
    # No callback: a VOLATILE message published before subscription is absent.
    with pytest.raises(ObstacleError):obs.sample(target)
    assert obs.diagnostics()['statistics'].get('paths_received',0)==0


def test_early_subscription_waits_for_requested_task_then_captures_path(context,observed):
    obs,clock,target=observed;update_report(context,phase=1);obs._receive_map(object())
    def transition():
        update_report(context,phase=3);obs._receive_path(message(stamp=clock.sim))
        obs._receive_path(message([[21.5,22.5],[21,26]],stamp=clock.sim))
    clock.events.append(transition)
    task=obs.wait_for_task(3,120)
    assert task['id']==3 and obs.sample(task)[-1]==[14.5,29.5]
    assert obs.diagnostics()['preparation']['status']=='TASK_RUNNING'


def test_wait_timeout_never_selects_previous_task(context,observed):
    obs,clock,target=observed;update_report(context,phase=1)
    with pytest.raises(ObstacleError,match='WAIT_TIMED_OUT'):obs.wait_for_task(3,0.2)
    assert obs.diagnostics()['preparation']['status']=='TASK_WAIT_TIMED_OUT'


@pytest.mark.parametrize('terminal',['SUCCEEDED','FAILED','UNKNOWN'])
def test_navigation_ends_during_read_only_wait(context,observed,terminal):
    obs,clock,target=observed;update_report(context,phase=1)
    clock.events.append(lambda:update_report(context,terminal=terminal))
    with pytest.raises(ObstacleError,match='NAVIGATION_ENDED'):obs.wait_for_task(3,120)
    assert obs.diagnostics()['preparation']['status']=='TASK_ENDED_EARLY'


def test_requested_task_already_passed_is_not_replayed(context,observed):
    obs,clock,target=observed;update_report(context,phase=4)
    with pytest.raises(ObstacleError,match='TARGET_TASK_ENDED'):obs.wait_for_task(3,120)


@pytest.mark.parametrize('mode',['run_ended','task_ended'])
def test_end_during_sampling_is_diagnosed(context,observed,mode):
    obs,clock,target=observed;obs._receive_map(object())
    clock.events.append(lambda:update_report(context,terminal='FAILED') if mode=='run_ended' else update_report(context,phase=4))
    with pytest.raises(ObstacleError,match='ENDED_BEFORE_INJECTION'):obs.sample(target)
    assert obs.diagnostics()['sample_history'][-1]['rejection_checks']['TASK_ENDED_EARLY']==1


def test_map_asset_mismatch_still_aborts(observed,monkeypatch):
    obs,clock,target=observed;obs._receive_map(object());obs._receive_path(message())
    monkeypatch.setattr(module,'check_map_message',Mock(side_effect=ValueError('asset mismatch')))
    with pytest.raises(ValueError):obs.sample(target)
    assert obs.diagnostics()['sample_history'][-1]['rejection_checks']['MAP_ASSET_MISMATCH']==1


@pytest.mark.parametrize('delta,accepted',[(0.59,True),(0.61,False)])
def test_goal_tolerance_preserved(observed,delta,accepted):
    obs,clock,target=observed;obs._receive_map(object());obs._receive_path(message([[21.5,22.5],[target['x']+delta,target['y']]]))
    if accepted:assert obs.sample(target)[-1][0]==target['x']+delta
    else:
        with pytest.raises(ObstacleError):obs.sample(target)


def test_cached_path_expires_again_before_create(observed):
    obs,clock,target=observed;obs._receive_map(object());obs._receive_path(message());obs.sample(target)
    clock.wall+=2.01
    with pytest.raises(ObstacleError,match='PATH_WALL_EXPIRED'):obs.assert_selected_fresh(target)


def test_run_ending_after_sample_still_prevents_create(context,observed):
    obs,clock,target=observed;obs._receive_map(object());obs._receive_path(message());obs.sample(target)
    update_report(context,terminal='SUCCEEDED')
    with pytest.raises(ObstacleError,match='NAVIGATION_ENDED'):obs.assert_selected_fresh(target)


@pytest.mark.parametrize('field',['run_id','plan_id','provenance_id'])
def test_wait_revalidates_identity(context,observed,field):
    obs,clock,target=observed;p=Path(context['report_path']);d=json.loads(p.read_text())
    if field=='provenance_id':d['source'][field]='0'*64
    else:d[field]='0'*64
    p.write_text(json.dumps(d))
    with pytest.raises(ObstacleError,match='IDENTITY_CHANGED'):obs.wait_for_task(3,120)


def test_bounded_candidate_buffer_reports_eviction(observed):
    obs,clock,target=observed
    for _ in range(65):obs._receive_path(message())
    assert obs.diagnostics()['candidate_count']==64 and obs.diagnostics()['statistics']['buffer_evictions']==1


def test_preparation_executor_join_precedes_foreground_spin(observed):
    obs,clock,target=observed;entered=threading.Event();release=threading.Event();events=[]
    class Executor:
        def __init__(self,context):pass
        def add_node(self,node):events.append('add')
        def spin_once(self,timeout_sec):entered.set();release.wait(0.01)
        def remove_node(self,node):events.append('remove')
        def shutdown(self,timeout_sec):events.append('shutdown')
    obs.start_preparation(Executor);assert entered.wait(1)
    release.set();obs.stop_preparation()
    assert obs._thread is None and events==['add','remove','shutdown']


def test_worker_exception_fails_closed(observed):
    obs,clock,target=observed;entered=threading.Event()
    class Executor:
        def __init__(self,context):pass
        def add_node(self,node):pass
        def spin_once(self,timeout_sec):entered.set();raise RuntimeError('private data never logged')
        def remove_node(self,node):pass
        def shutdown(self,timeout_sec):pass
    obs.start_preparation(Executor);obs._thread.join(1)
    with pytest.raises(ObstacleError,match='WORKER_FAILED'):obs.sample(target)
    assert obs.diagnostics()['worker_error']=='RuntimeError'
    obs.close();assert 'private data' not in json.dumps(obs.diagnostics())


def test_lifecycle_subscribes_and_waits_before_world_discovery(context,tmp_path):
    events=[];backend=FakeGazebo(context);old_discover=backend.discover
    backend.discover=lambda:(events.append('discover'),old_discover())[1]
    class Observer(FakeObserver):
        def bind_context(self,c):events.append('bind')
        def start_preparation(self):events.append('spin')
        def wait_for_task(self,*args):events.append('wait')
        def stop_preparation(self):events.append('stop_spin')
    def factory(bundle):events.append('subscribe');return Observer(context)
    def controller(*args):events.append('controller')
    code,r=lifecycle.run_lease(context,backend,factory,motion_record(context),tmp_path/'lease.json',
        wait_task_seconds=120,motion_controller=controller)
    assert code==0 and events[:5]==['subscribe','bind','spin','wait','discover']
    assert events.index('stop_spin')<events.index('controller') and r['cleanup']=='CLEAN'


def test_failed_observation_records_diagnostics_without_create(context,observed,tmp_path):
    obs,clock,target=observed;obs._receive_map(object());backend=FakeGazebo(context)
    # No real executors in this integration test.
    obs.start_preparation=lambda:None
    code,r=lifecycle.run_lease(context,backend,lambda bundle:obs,motion_record(context),tmp_path/'lease.json',
        wait_task_seconds=120,motion_controller=Mock())
    assert code==1 and r['create_sent'] is False and r['entity_id'] is None and r['cleanup']=='NOT_NEEDED'
    assert backend.create_calls==[] and backend.remove_calls==[]
    assert r['path_observation']['sample_history'][-1]['rejection_checks']['PATH_NOT_RECEIVED']>0
    assert json.loads((tmp_path/'lease.json').read_text())['path_observation']==r['path_observation']


def test_wait_for_task_failure_no_gazebo_discovery(context,observed,tmp_path):
    obs,clock,target=observed;obs.start_preparation=lambda:None;update_report(context,terminal='FAILED')
    backend=Mock()
    code,r=lifecycle.run_lease(context,backend,lambda bundle:obs,motion_record(context),tmp_path/'lease.json',
        wait_task_seconds=120,motion_controller=Mock())
    assert code==1 and r['cleanup']=='NOT_NEEDED';backend.discover.assert_not_called();backend.create.assert_not_called()


def test_main_exports_path_diagnostics_into_new_metrics(context,tmp_path):
    plugin=tmp_path/'plugin.so';plugin.write_bytes(b'fake, not loaded')
    class Observer(FakeObserver):
        def diagnostics(self):return {'statistics':{'paths_received':0},'sample_history':[{'rejection_checks':{'PATH_NOT_RECEIVED':1}}]}
        def sample(self,task):raise ObstacleError('FRESH_MATCHING_NAV2_PATH_NOT_OBSERVED')
    code=moving.main(['--plan',str(tmp_path/'bundle/plan.json'),'--run-report',context['report_path'],
        '--journal',str(tmp_path/'lease.json'),'--metrics',str(tmp_path/'metrics.json'),
        '--plugin',str(plugin),'--apply'],backend_factory=lambda:FakeGazebo(context),observer_factory=lambda bundle:Observer(context))
    metrics=json.loads((tmp_path/'metrics.json').read_text())
    assert code==1 and metrics['path_observation']['statistics']['paths_received']==0


@pytest.mark.parametrize('fault',['stale_path','navigation_ended'])
def test_last_gate_after_wal_still_prevents_creation(context,observed,tmp_path,fault):
    from day4.reporting import atomic_write_json
    obs,clock,target=observed;obs._receive_map(object());obs._receive_path(message(route()))
    obs.start_preparation=lambda:None;backend=FakeGazebo(context)
    def writer(path,data):
        atomic_write_json(path,data)
        if data.get('status')=='CREATING':
            if fault=='stale_path':clock.wall+=2.01
            else:update_report(context,terminal='FAILED')
    code,r=lifecycle.run_lease(context,backend,lambda bundle:obs,motion_record(context),tmp_path/'lease.json',
        wait_task_seconds=120,motion_controller=Mock(),writer=writer)
    assert code==1 and r['create_sent'] is False and r['cleanup']=='NOT_NEEDED'
    assert backend.create_calls==[] and backend.remove_calls==[]
    if fault=='stale_path':
        assert r['path_observation']['sample_history'][-1]['error']=='PATH_WALL_EXPIRED'
    else:
        assert r['path_observation']['bound_run_state']['navigation_status']=='FAILED'


def test_no_parallel_executor_during_controller_and_cleanup_on_handoff_failure(context,tmp_path):
    backend=FakeGazebo(context);controller=Mock()
    class Observer(FakeObserver):
        def start_preparation(self):pass
        def stop_preparation(self):raise ObstacleError('PATH_OBSERVER_STOP_NOT_CONFIRMED')
    code,r=lifecycle.run_lease(context,backend,lambda bundle:Observer(context),motion_record(context),tmp_path/'lease.json',
        motion_controller=controller)
    assert code==1 and r['cleanup']=='CLEAN' and backend.remove_calls==[1000]
    controller.assert_not_called()


def test_wait_checkpoint_failure_cannot_create_obstacle(context,observed,tmp_path):
    obs,clock,target=observed;update_report(context,phase=1);obs.start_preparation=lambda:None
    backend=FakeGazebo(context)
    def writer(path,data):
        if data.get('status')=='WAITING_FOR_TASK':raise OSError('not persisted')
    code,r=lifecycle.run_lease(context,backend,lambda bundle:obs,motion_record(context),tmp_path/'lease.json',
        wait_task_seconds=3,motion_controller=Mock(),writer=writer)
    assert code==1 and r['cleanup']=='NOT_NEEDED' and r['create_sent'] is False
    assert backend.create_calls==[]
