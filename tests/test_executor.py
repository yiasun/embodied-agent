import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from day4 import reporting, task_executor_v4 as executor
from day4.recovery_policy import decide
from planner import execute_plan


class Navigator:
    def __init__(self,outcomes,confirmed=True):
        self.outcomes=iter(outcomes);self.calls=[];self.confirmed=confirmed
        self.last_status='PENDING';self.destroy_node=Mock();self.abort_active=Mock(return_value=confirmed)
    def navigate(self,x,y,timeout_sec):
        self.calls.append((x,y))
        outcome=next(self.outcomes)
        if isinstance(outcome,BaseException):raise outcome
        self.last_status,self.last_reason=outcome
        return self.last_status=='SUCCEEDED'


def run(tmp_path,outcomes,goals=None,**kwargs):
    node=Navigator(outcomes,kwargs.pop('confirmed',True))
    path=tmp_path/'report.json'
    code,report=executor.run_tasks(goals or [(0,0),(1,1)],lambda:node,path,sleep=lambda _:None,**kwargs)
    return node,path,code,report


@pytest.mark.parametrize('status,reason,expected',[
    ('SUCCEEDED','goal_reached',0),('FAILED','nav2_status_6',1),('TIMED_OUT','timeout_cancel_confirmed',3),
    ('UNKNOWN','send_response_timeout',4),('REJECTED','goal_rejected',1),('CANCELLED','goal_cancelled',1)])
def test_status_and_exit_code(tmp_path,status,reason,expected):
    node,path,code,report=run(tmp_path,[(status,reason)],goals=[(0,0)],retries=0)
    assert code==expected
    assert json.loads(path.read_text())['status']==status
    node.destroy_node.assert_called_once()


def test_success_report_and_trace(tmp_path):
    from planner.planner_v1 import build_plan, load_scene
    from planner.plan_identity import compute_plan_id
    plan=build_plan(load_scene('turtlebot3_demo'),['A区','B区'])
    plan_id=compute_plan_id(plan)
    goals=[(s['x'],s['y']) for s in plan['steps']]
    node,path,code,report=run(tmp_path,[('SUCCEEDED','goal_reached')]*2,goals=goals,
                              execution_id='run',plan_id=plan_id,trusted_plan=plan)
    assert code==0 and len(node.calls)==2
    assert report['run_id']=='run' and report['plan_id']==plan_id
    assert report['tasks'][0]['attempts'][0]['recovery_action']=='CONTINUE'
    execute_plan.verify_execution_report(path,'run',plan_id,['0.419','-1.822','-0.302','0.459'])


def test_retry_exhaustion_stops_chain(tmp_path):
    node,path,code,report=run(tmp_path,[('TIMED_OUT','timeout_cancel_confirmed')]*2,retries=1)
    assert code==3 and len(node.calls)==2
    assert [a['recovery_action'] for a in report['tasks'][0]['attempts']]==['RETRY','HALT']
    assert report['tasks'][1]['status']=='SKIPPED'


def test_retry_then_success(tmp_path):
    node,_,code,report=run(tmp_path,[('TIMED_OUT','timeout_cancel_confirmed'),('SUCCEEDED','goal_reached'),('SUCCEEDED','goal_reached')])
    assert code==0 and len(node.calls)==3


@pytest.mark.parametrize('status,reason', [('UNKNOWN','send_response_timeout'),('FAILED','nav2_status_6'),('REJECTED','goal_rejected')])
def test_nonretryable_stops_chain(tmp_path,status,reason):
    node,_,code,report=run(tmp_path,[(status,reason)],retries=5)
    assert code!=0 and len(node.calls)==1
    assert report['tasks'][1]['status']=='SKIPPED'
    assert report['tasks'][0]['attempts'][0]['recovery_action']=='HALT'


@pytest.mark.parametrize('exc',[RuntimeError('error'),KeyboardInterrupt()])
@pytest.mark.parametrize('confirmed',[True,False])
def test_runner_exception_interrupt_cleanup(tmp_path,exc,confirmed):
    node,path,code,report=run(tmp_path,[exc],confirmed=confirmed)
    assert code==(4 if not confirmed else (130 if isinstance(exc,KeyboardInterrupt) else 1))
    assert report['tasks'][1]['status']=='SKIPPED'
    assert report['tasks'][0]['attempts'][0]['recovery_action']=='HALT'
    assert report['ended_at'] and path.exists()
    node.abort_active.assert_called();node.destroy_node.assert_called_once()


@pytest.mark.parametrize('failure_call',[1,2,4,5])
def test_transient_log_failure_halts_and_finalizes(tmp_path,failure_call):
    calls=0
    def writer(path,report):
        nonlocal calls
        calls+=1
        if calls==failure_call:raise OSError('disk failure')
        reporting.atomic_write_json(path,report)
    node=Navigator([('SUCCEEDED','goal_reached')]*2)
    finalize=Mock()
    code,report=executor.run_tasks([(0,0),(1,1)],lambda:node,tmp_path/'log.json',writer=writer,finalize=finalize)
    assert code!=0
    assert len(node.calls)<=1
    finalize.assert_called_once()
    if failure_call==1:node.destroy_node.assert_not_called()
    else:node.destroy_node.assert_called_once()
    assert json.loads((tmp_path/'log.json').read_text())['status']!='SUCCEEDED'


def test_permanent_log_failure_never_creates_node(tmp_path):
    create=Mock();finalize=Mock()
    def fail(*_):raise OSError('disk failure')
    code,report=executor.run_tasks([(0,0)],create,tmp_path/'log.json',writer=fail,finalize=finalize)
    assert code!=0 and report['report_write_failed']
    create.assert_not_called();finalize.assert_called_once()


def test_final_write_failure_returns_nonzero(tmp_path):
    def writer(path,report):
        if 'ended_at' in report:raise OSError('disk failure')
        reporting.atomic_write_json(path,report)
    _,_,code,report=run(tmp_path,[('SUCCEEDED','goal_reached')],goals=[(0,0)],writer=writer)
    assert code!=0 and report['report_write_failed']


def test_cleanup_failure_not_success(tmp_path):
    node=Navigator([('SUCCEEDED','goal_reached')]);node.destroy_node.side_effect=RuntimeError()
    shutdown=Mock(side_effect=RuntimeError())
    code,report=executor.run_tasks([(0,0)],lambda:node,tmp_path/'log.json',finalize=shutdown)
    assert code!=0 and report['cleanup_error']
    shutdown.assert_called_once()


def test_initialization_failure_finalizes(tmp_path):
    shutdown=Mock()
    def create():raise RuntimeError('node creation failed')
    code,report=executor.run_tasks([(0,0)],create,tmp_path/'log.json',finalize=shutdown)
    assert code==1 and report['status']=='FAILED'
    shutdown.assert_called_once()


def test_existing_log_never_overwritten(tmp_path):
    path=tmp_path/'old.json';path.write_text('original')
    create=Mock()
    with pytest.raises(ValueError):executor.run_tasks([(0,0)],create,path)
    assert path.read_text()=='original';create.assert_not_called()


def test_atomic_replace_failure_preserves_old_file(tmp_path,monkeypatch):
    path=tmp_path/'report.json';path.write_text('{"old":true}')
    def fail(*_):raise OSError('replace failed')
    monkeypatch.setattr(reporting.os,'replace',fail)
    with pytest.raises(OSError):reporting.atomic_write_json(path,{'new':True})
    assert path.read_text()=='{"old":true}'
    assert list(tmp_path.iterdir())==[path]


@pytest.mark.parametrize('status,reason,attempt,retries,action',[
    ('SUCCEEDED','goal_reached',1,1,'CONTINUE'),
    ('TIMED_OUT','timeout_cancel_confirmed',1,1,'RETRY'),
    ('TIMED_OUT','timeout_cancel_confirmed',2,1,'HALT'),
    ('TIMED_OUT','timeout_cancel_confirmed',1,0,'HALT'),
    ('TIMED_OUT','unconfirmed',1,1,'HALT'),('UNKNOWN','x',1,1,'HALT'),
    ('FAILED','x',1,1,'HALT'),('REJECTED','goal_rejected',1,1,'HALT')])
def test_recovery_policy(status,reason,attempt,retries,action):
    assert decide(status,reason,attempt,retries).action==action


def test_v4_preview_no_ros_or_node(monkeypatch):
    run_mock=Mock(side_effect=AssertionError('must not execute'))
    monkeypatch.setattr(executor,'run_tasks',run_mock)
    assert executor.main(['0','0'])==0
    run_mock.assert_not_called()


@pytest.mark.parametrize('args',[['0'],['nan','0'],['0','0','--timeout','nan'],['0','0','--retries','-1'],['0']*42])
def test_v4_invalid_cli(args):
    with pytest.raises(SystemExit) as exc:executor.main(args)
    assert exc.value.code!=0


@pytest.mark.parametrize('mode',['success','failed_report','missing_report','bad_json','wrong_id','wrong_plan','wrong_count','wrong_coordinates','missing_decision','exit_failure'])
def test_parent_requires_exit_code_and_matching_success_report(tmp_path,monkeypatch,mode):
    monkeypatch.setattr(execute_plan,'ROOT',tmp_path)
    def child(command):
        # This is a fake subprocess; no ROS is loaded or invoked.
        assert '--execute' in command
        def arg(name):return command[command.index(name)+1]
        path=Path(arg('--report'))
        code,report=executor.run_tasks([(0.419,-1.822),(-0.302,0.459)],
            lambda:Navigator([('SUCCEEDED','goal_reached')]*2),path,
            execution_id=arg('--execution-id'),plan_id=arg('--plan-id'),
            trusted_plan=json.loads(Path(arg('--plan')).read_text()))
        if mode=='failed_report':report['status']='FAILED'
        if mode=='wrong_id':report['run_id']='other'
        if mode=='wrong_plan':report['plan_id']='other'
        if mode=='wrong_count':report['tasks'].pop()
        if mode=='wrong_coordinates':report['tasks'][0]['x']=99
        if mode=='missing_decision':report['tasks'][0]['attempts']=[]
        reporting.atomic_write_json(path,report)
        if mode=='missing_report':path.unlink()
        if mode=='bad_json':path.write_text('{')
        return 1 if mode=='exit_failure' else 0
    monkeypatch.setattr(execute_plan,'run_executor',child)
    plan=Path(__file__).resolve().parents[1]/'plans/plan_nl_v1.json'
    assert execute_plan.main(['--plan',str(plan),'--execute'])==(0 if mode=='success' else 1)


@pytest.mark.parametrize('times_out',[False,True])
def test_parent_forwards_interrupt_and_waits(monkeypatch,times_out):
    import subprocess
    import signal
    child=Mock()
    child.wait.side_effect=[KeyboardInterrupt(),subprocess.TimeoutExpired('executor',40) if times_out else 130]
    child.poll.return_value=None
    monkeypatch.setattr(execute_plan.subprocess,'Popen',Mock(return_value=child))
    error=execute_plan.ExecutorStateUnknown if times_out else KeyboardInterrupt
    with pytest.raises(error):execute_plan.run_executor(['fake-executor'])
    child.send_signal.assert_called_once_with(signal.SIGINT)
    child.wait.assert_called_with(timeout=40)
    child.kill.assert_not_called()


def test_inconsistent_client_cannot_advance(tmp_path):
    node=Navigator([])
    def inconsistent(*args,**kwargs):
        node.calls.append(args)
        node.last_status='SUCCEEDED';node.last_reason='goal_reached'
        return False
    node.navigate=inconsistent
    code,report=executor.run_tasks([(0,0),(1,1)],lambda:node,tmp_path/'report.json')
    assert code==4 and len(node.calls)==1
    assert report['tasks'][1]['status']=='SKIPPED'


def test_final_cancel_exception_still_destroys_and_saves(tmp_path):
    node=Navigator([('UNKNOWN','send_response_timeout')])
    node.abort_active.side_effect=RuntimeError('cancel failed')
    code,report=executor.run_tasks([(0,0)],lambda:node,tmp_path/'report.json')
    assert code==4 and report['status']=='UNKNOWN'
    node.destroy_node.assert_called_once()
    assert json.loads((tmp_path/'report.json').read_text())['status']=='UNKNOWN'


def test_system_exit_zero_cannot_claim_success(tmp_path):
    _,path,code,report=run(tmp_path,[SystemExit(0)])
    assert code==130
    assert json.loads(path.read_text())['status']=='INTERRUPTED'


def test_parent_wait_exception_cancels_child(monkeypatch):
    child=Mock();child.wait.side_effect=[OSError('wait failed'),1];child.poll.return_value=None
    monkeypatch.setattr(execute_plan.subprocess,'Popen',Mock(return_value=child))
    with pytest.raises(OSError):execute_plan.run_executor(['fake-executor'])
    child.send_signal.assert_called_once()
    child.wait.assert_called_with(timeout=40)
