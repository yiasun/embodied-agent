"""Virtual monotonic time, fake physics/transport, real isolated I/O workers."""
import copy
import json
import shutil
import subprocess
import threading
import time
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from test_obstacles import context
from test_stage62_motion import record as motion_record
from reliability import moving, gazebo
from reliability.safety import ObstacleError
from reliability.scheduling import AsyncTask, RuntimeIO, Timings, freeze_report
from day4.reporting import atomic_write_json


class Clock:
    def __init__(self):self.t=100.0;self.condition=threading.Condition();self.closed=False
    def now(self):
        with self.condition:return self.t
    def advance(self,seconds):
        with self.condition:self.t+=seconds;self.condition.notify_all()
        time.sleep(0.0005) # yield to I/O workers; not simulated safety time
    def wait_virtual(self,seconds):
        until=self.now()+seconds
        with self.condition:
            while self.t<until and not self.closed:self.condition.wait(timeout=0.1)
    def finish(self):
        with self.condition:self.closed=True;self.condition.notify_all()


class Channel:
    def __init__(self,clock,duration=3):
        self.clock=clock;self.start=clock.now();self.duration=duration;self.received=[];self.sent=[]
        self.closed=False;self.stopped=False;self.reason='WAITING_FOR_HEARTBEAT';self.sequence=0;self.drop=False
        self.expired_at=None;self.stale=False
    def state(self):
        now=self.clock.now();self.sequence+=1
        if self.received and not self.stopped and now-self.received[-1]>=0.5:
            self.stopped=True;self.reason='HEARTBEAT_EXPIRED';self.expired_at=now
        if now-self.start>=self.duration and not self.stopped:
            self.stopped=True;self.reason='DURATION_COMPLETE'
        return ({'sequence':self.sequence,'sim_time':20+now-self.start,'stopped':self.stopped,
            'reason':self.reason,'commanded_vy':0 if self.stopped else 0.2,
            'heartbeat_received_count':len(self.received),
            'heartbeat_age_wall_s':now-self.received[-1] if self.received else None},now-(0.3 if self.stale else 0))
    def send(self,command,**kwargs):
        now=self.clock.now();self.sent.append((command,now,threading.current_thread().name))
        if command=='heartbeat' and not self.drop:
            self.received.append(now);self.reason='MOVING'
        if command=='stop':self.stopped=True;self.reason='EXPLICIT_STOP'
    def close(self):self.closed=True


class Capture:
    def __init__(self,clock):
        self.clock=clock;self.start=clock.now();self.metrics=NS(samples=[])
        self.execution={'status':'RUNNING','tasks':[{'id':3,'status':'RUNNING'}]}
        self.closed=False;self.slow_at=None;self.bad_pose=False;self.inactive=False
    def tick(self,state):
        if self.slow_at is not None and len(self.metrics.samples)==self.slow_at:self.clock.advance(0.65)
        elapsed=self.clock.now()-self.start
        self.metrics.samples.append({'obstacle':{'position':[16,25.5+elapsed*0.1,0.6]},'segment_valid':True,
            'received_mono':self.clock.now()-(0.3 if self.bad_pose else 0),'geometric_clearance_m':4.6,
            'task_id':3,'task_status':'SUCCEEDED' if self.inactive else 'RUNNING'})
        return True
    def report(self,error=None):
        return {'error':error,'samples':self.metrics.samples,'paths':[],'bt_events':[],
                'quality':{'issues':[],'sample_count':len(self.metrics.samples)}}
    def close(self):self.closed=True


def make_case(context,tmp_path,monkeypatch,*,duration=3,probe_delay=0,write_delay=0,checkpoint_delay=0):
    clock=Clock();monkeypatch.setattr(moving,'time',NS(monotonic=clock.now))
    channel=Channel(clock,duration);capture=Capture(clock)
    lease=motion_record(context);lease.update(active_task_id=3,world_session_id='a'*64)
    probes=[];checkpoints=[];writes=[]
    def probe():
        probes.append(clock.now());clock.wait_virtual(probe_delay)
        return {'session_id':'a'*64}
    backend=NS(discover=lambda:{'session_id':'a'*64},session_probe=lambda:probe)
    def writer(path,data):
        writes.append((str(path),clock.now()))
        if str(path).endswith('.progress.json'):clock.wait_virtual(write_delay)
        atomic_write_json(path,data)
    def checkpoint(report,*,progress=False):
        if progress:clock.wait_virtual(checkpoint_delay)
        checkpoints.append((copy.deepcopy(report),progress,clock.now()))
    def call(sleep=None):
        try:
            return moving.controller(context,backend,None,lease,tmp_path/'lease.json',writer,
                channel_factory=lambda *a:channel,capture_factory=lambda *a:capture,
                sleep=sleep or clock.advance,checkpoint=checkpoint)
        finally:clock.finish() # release virtual-time waiters after bounded worker shutdown
    return NS(clock=clock,channel=channel,capture=capture,lease=lease,probe=probe,backend=backend,
              writer=writer,checkpoint=checkpoint,call=call,probes=probes,writes=writes,checkpoints=checkpoints)


def heartbeats(case):return [stamp for command,stamp,_thread in case.channel.sent if command=='heartbeat']


def test_normal_sustained_heartbeat_is_foreground_only(context,tmp_path,monkeypatch):
    c=make_case(context,tmp_path,monkeypatch);c.call()
    beats=heartbeats(c);assert len(beats)>50 and max(b-a for a,b in zip(beats,beats[1:]))<0.1
    assert all(thread=='MainThread' for cmd,_stamp,thread in c.channel.sent if cmd=='heartbeat')
    assert c.channel.closed and c.capture.closed and c.lease['motion_stop']=='LATCHED_ZERO_COMMAND'
    stats=c.lease['controller_diagnostics']['statistics']
    assert stats['capture_tick']['count']>50 and stats['heartbeat_send_interval']['max_s']<0.1
    assert c.lease['controller_diagnostics']['transport_one_way_latency']['status']=='NOT_MEASURED'


def test_slow_discovery_does_not_block_keepalive(context,tmp_path,monkeypatch):
    c=make_case(context,tmp_path,monkeypatch,probe_delay=0.65);c.call()
    beats=heartbeats(c);assert max(b-a for a,b in zip(beats,beats[1:]))<0.1
    assert c.lease['controller_diagnostics']['statistics']['gazebo_discovery']['max_s']>=0.65
    assert len(c.probes)>1 and c.channel.expired_at is None


@pytest.mark.parametrize('operation',['writer','checkpoint'])
def test_slow_persistence_does_not_block_keepalive(context,tmp_path,monkeypatch,operation):
    c=make_case(context,tmp_path,monkeypatch,write_delay=0.65 if operation=='writer' else 0,
            checkpoint_delay=0.65 if operation=='checkpoint' else 0)
    c.call();beats=heartbeats(c)
    assert max(b-a for a,b in zip(beats,beats[1:]))<0.1 and c.channel.expired_at is None
    stat=c.lease['controller_diagnostics']['statistics']['json_writer' if operation=='writer' else 'checkpoint']
    assert stat['max_s']>=0.65


def test_capture_timeout_revokes_heartbeat(context,tmp_path,monkeypatch):
    c=make_case(context,tmp_path,monkeypatch);c.capture.slow_at=5
    with pytest.raises(ObstacleError,match='CAPTURE_TICK_TIMED_OUT'):c.call()
    assert len(heartbeats(c))==5 and c.lease['controller_diagnostics']['statistics']['capture_tick']['max_s']>=0.65
    assert c.channel.sent[-1][0]=='stop' and c.lease['motion_stop']=='LATCHED_ZERO_COMMAND'


def test_main_loop_stall_has_no_background_heartbeat(context,tmp_path,monkeypatch):
    c=make_case(context,tmp_path,monkeypatch);first=True
    def stalled(seconds):
        nonlocal first
        c.clock.advance(0.65 if first else seconds);first=False
    with pytest.raises(ObstacleError,match='NATIVE_STOP_HEARTBEAT_EXPIRED'):c.call(stalled)
    assert len(heartbeats(c))==1 and c.channel.expired_at is not None


def test_packet_loss_expires_native_watchdog(context,tmp_path,monkeypatch):
    c=make_case(context,tmp_path,monkeypatch)
    def lose_after_first(seconds):
        c.channel.drop=True;c.clock.advance(seconds)
    with pytest.raises(ObstacleError,match='NATIVE_STOP_HEARTBEAT_EXPIRED'):c.call(lose_after_first)
    assert len(c.channel.received)==1 and len(heartbeats(c))>5
    assert c.channel.expired_at-c.channel.received[-1]>=0.5


def test_no_delivery_of_initial_heartbeat_fails_closed(context,tmp_path,monkeypatch):
    c=make_case(context,tmp_path,monkeypatch);c.channel.drop=True
    with pytest.raises(ObstacleError,match='NATIVE_HEARTBEAT_ACK_TIMEOUT'):c.call()
    assert c.channel.received==[]


@pytest.mark.parametrize('fault',['pose','task','state','clearance'])
def test_no_heartbeat_with_expired_or_invalid_health(context,tmp_path,monkeypatch,fault):
    c=make_case(context,tmp_path,monkeypatch)
    if fault=='pose':c.capture.bad_pose=True
    if fault=='task':c.capture.inactive=True
    if fault=='state':c.channel.stale=True
    if fault=='clearance':
        tick=c.capture.tick
        def unsafe(state):
            result=tick(state);c.capture.metrics.samples[-1]['geometric_clearance_m']=0.34;return result
        c.capture.tick=unsafe
    with pytest.raises(ObstacleError):c.call()
    assert heartbeats(c)==[] and c.channel.sent[-1][0]=='stop'


def test_blocked_discovery_times_out_without_indefinite_keepalive(context,tmp_path,monkeypatch):
    c=make_case(context,tmp_path,monkeypatch);release=threading.Event()
    c.backend.session_probe=lambda:lambda:(release.wait(),{'session_id':'a'*64})[1]
    try:
        with pytest.raises(ObstacleError,match='DISCOVERY_TIMED_OUT'):c.call()
        assert heartbeats(c)[-1]-heartbeats(c)[0]<1.5
        assert c.lease['controller_diagnostics']['io']['shutdown']['discovery']['unfinished']
    finally:release.set()


def test_late_blocked_progress_write_cannot_overwrite_final_journal(context,tmp_path,monkeypatch):
    c=make_case(context,tmp_path,monkeypatch,duration=5);release=threading.Event();finished=threading.Event()
    def writer(path,data):
        if str(path).endswith('.progress.json'):
            release.wait();atomic_write_json(path,data);finished.set()
        else:atomic_write_json(path,data)
    def call():
        moving.controller(context,c.backend,None,c.lease,tmp_path/'lease.json',writer,
            channel_factory=lambda *a:c.channel,capture_factory=lambda *a:c.capture,
            sleep=c.clock.advance,checkpoint=c.checkpoint)
    try:
        with pytest.raises(ObstacleError,match='PERSISTENCE_TIMED_OUT'):call()
        assert c.lease['motion_stop']=='LATCHED_ZERO_COMMAND'
        assert c.lease['controller_diagnostics']['io']['shutdown']['persistence']['unfinished']
        atomic_write_json(tmp_path/'lease.json',{'status':'FINAL','cleanup':'CLEAN'})
    finally:release.set()
    assert finished.wait(1)
    assert json.loads((tmp_path/'lease.json').read_text())=={'status':'FINAL','cleanup':'CLEAN'}
    assert json.loads((tmp_path/'lease.json.progress.json').read_text())['progress']['progress_only'] is True


def test_changed_session_is_rejected_without_waiting_for_expiry(context,tmp_path,monkeypatch):
    c=make_case(context,tmp_path,monkeypatch)
    c.backend.session_probe=lambda:lambda:{'session_id':'b'*64}
    with pytest.raises(ObstacleError,match='WORLD_SESSION_CHANGED_DURING_MOTION'):c.call()
    assert c.channel.sent[-1][0]=='stop'


def test_session_probe_uses_dedicated_backend(monkeypatch):
    from reliability import probing
    original=gazebo.GazeboBackend.__new__(gazebo.GazeboBackend);original.world='default';original.robot_name='robot'
    private=Mock(return_value={'session_id':'a'*64});factory=Mock(return_value=private)
    monkeypatch.setattr(probing,'SessionProbe',factory)
    probe=original.session_probe();assert probe()['session_id']=='a'*64
    factory.assert_called_once_with('default','robot');private.assert_called_once()


def test_discovery_duration_cannot_be_hidden_by_late_completion():
    clock=Clock();worker=AsyncTask('discovery',clock.now);completed=threading.Event()
    def slow():clock.advance(1.1);completed.set();return {'session_id':'a'*64}
    worker.submit(slow);assert completed.wait(1)
    worker.thread.join(1)
    with pytest.raises(ObstacleError,match='DISCOVERY_TIMED_OUT'):worker.take(1.0)
    worker.close()


def test_session_proof_age_includes_the_query_time():
    clock=Clock();stats=Timings(clock.now);io=RuntimeIO(lambda:None,'a'*64,clock.now(),stats,clock.now)
    clock.advance(2.6)
    with pytest.raises(ObstacleError,match='WORLD_SESSION_PROOF_STALE'):io.poll()
    io.close()


def test_report_snapshot_freezes_append_only_collections():
    source={'samples':[{'robot':[1,2,0]}],'paths':[{'points':[[0,0],[1,1]]}],
            'bt_events':[],'quality':{'issues':[]}}
    snapshot=freeze_report(source);source['samples'].append({'robot':[3,4,0]});source['quality']['issues'].append('gap')
    assert len(snapshot['samples'])==1 and snapshot['quality']['issues']==[]
    assert snapshot['paths'][0]['points'] is source['paths'][0]['points']


def test_sanitized_async_failure_and_bounded_queue():
    clock=Clock();worker=AsyncTask('persistence',clock.now)
    def failed():raise OSError('private sentinel')
    worker.submit(failed);worker.thread.join(1)
    with pytest.raises(ObstacleError,match='PERSISTENCE_FAILED_OSError') as exc:worker.take(2)
    assert 'private sentinel' not in str(exc.value)
    worker.close()


def test_error_completed_during_shutdown_is_not_lost():
    clock=Clock();io=RuntimeIO(lambda:None,'a'*64,clock.now(),Timings(clock.now),clock.now)
    release=threading.Event()
    def failed():release.wait();raise OSError('not logged')
    io.persistence.submit(failed);original_close=io.persistence.close
    def close():release.set();return original_close()
    io.persistence.close=close
    result=io.close()
    assert result['final_error']=='PERSISTENCE_FAILED_OSError'


def fake_probe_server(connection,world,robot):
    """No SDK: optional native GIL hold is isolated in the spawned test process."""
    connection.send({'ready':True})
    try:
        while connection.recv()=='probe':
            if world=='gil':
                import ctypes
                ctypes.PyDLL(None).usleep(600000) # keeps THIS child interpreter's GIL
            if world=='blocked':time.sleep(10)
            connection.send({'session_id':'bad' if world=='invalid' else 'a'*64})
    except (EOFError,BrokenPipeError):pass
    finally:connection.close()


def test_native_gil_block_in_probe_process_does_not_stall_parent():
    from reliability.probing import SessionProbe
    probe=SessionProbe('gil','robot',target=fake_probe_server);result=[]
    worker=threading.Thread(target=lambda:result.append(probe.startup_check()))
    try:
        worker.start();beats=[];deadline=time.monotonic()+4
        while worker.is_alive() and time.monotonic()<deadline:
            beats.append(time.monotonic());time.sleep(0.01)
        worker.join(0.1)
        assert result==[{'session_id':'a'*64}] and len(beats)>30
        assert max(b-a for a,b in zip(beats,beats[1:]))<0.5
    finally:probe.close();worker.join(1)
    assert not probe.process.is_alive()


def test_blocked_probe_child_has_bounded_timeout_and_is_terminated():
    from reliability.probing import SessionProbe
    probe=SessionProbe('blocked','robot',target=fake_probe_server,timeout=0.3)
    try:
        with pytest.raises(ObstacleError,match='TIMED_OUT'):probe()
    finally:probe.close()
    assert not probe.process.is_alive()


def test_probe_child_response_is_validated_and_close_idempotent():
    from reliability.probing import SessionProbe
    probe=SessionProbe('invalid','robot',target=fake_probe_server)
    try:
        with pytest.raises(ObstacleError,match='INVALID_SESSION_PROBE_RESPONSE'):probe.startup_check()
    finally:probe.close();probe.close()
    with pytest.raises(ObstacleError,match='SESSION_PROBE_CLOSED'):probe()


def test_probe_shutdown_failure_is_not_silently_successful():
    clock=Clock()
    class Probe:
        def __call__(self):return {'session_id':'a'*64}
        def close(self):raise OSError('not logged')
    io=RuntimeIO(Probe(),'a'*64,clock.now(),Timings(clock.now),clock.now)
    assert io.close()['final_error']=='SESSION_PROBE_SHUTDOWN_FAILED'


@pytest.fixture(scope='module')
def native_watchdog(tmp_path_factory):
    compiler=shutil.which('g++')
    if not compiler:pytest.skip('C++ compiler unavailable')
    directory=tmp_path_factory.mktemp('watchdog');src=directory/'watchdog.cc'
    src.write_text(r'''
#include "HeartbeatWatchdog.hh"
#include "BoundedProfile.hh"
#include <cassert>
int main(int, char **argv) {
 embodied::HeartbeatWatchdog hb;
 const std::string mode=argv[1];assert(!hb.Fresh(1));assert(hb.Age(1)==-1);
 hb.Receive(1);assert(hb.Fresh(1.499));assert(!hb.Fresh(1.5));
 if(mode=="normal") {for(int i=1;i<=100;i++){double t=1+i*.03;hb.Receive(t);assert(hb.Fresh(t+.01));}}
 if(mode=="gap") {hb.Receive(1.7);assert(hb.lastGap>.69);assert(hb.maxGap>.69);assert(hb.received==2);}
 if(mode=="stall" || mode=="loss") {
   embodied::BoundedProfile p;p.x=16;
   p.Step(1,16,25.5,.6,0,21,22,hb.Fresh(1),false,0);
   p.Step(1.003,16,25.5,.6,0,21,22,hb.Fresh(1.5),false,.5);
   assert(p.stopped && p.velocity==0 && p.reason=="HEARTBEAT_EXPIRED");
   hb.Receive(1.6);assert(p.Step(1.006,16,25.5,.6,0,21,22,hb.Fresh(1.6),false,.6)==0);
 }
 if(mode=="sim_pause") {assert(!hb.Fresh(2));} // unchanged sim clock cannot refresh steady time
 if(mode=="queued") {
   embodied::HeartbeatWatchdog guarded;guarded.Published(1,1);
   assert(guarded.ReceiveProof(1.2,1,1));assert(!guarded.Fresh(1.5));
   assert(!guarded.ReceiveProof(1.45,2,1));assert(!guarded.Fresh(1.5));
   assert(guarded.received==2 && guarded.authorized==1 && guarded.rejected==1);
 }
 if(mode=="replay") {
   embodied::HeartbeatWatchdog guarded;guarded.Published(1,1);
   assert(guarded.ReceiveProof(1.01,1,1));assert(!guarded.ReceiveProof(1.02,1,1));
 }
 if(mode=="wire") {
   std::uint64_t command=0,state=0;
   assert(embodied::ParseHeartbeat("heartbeat:1:23",command,state));assert(command==1 && state==23);
   for(const auto &bad:{"heartbeat","heartbeat:0:1","heartbeat:1:0","heartbeat:-1:1", "heartbeat:1:2junk", "heartbeat:1:2:3"})
      assert(!embodied::ParseHeartbeat(bad,command,state));
 }
}
''')
    binary=directory/'watchdog';root=Path(__file__).resolve().parents[1]
    subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Werror','-I',str(root/'reliability/native'),
                    str(src),'-o',str(binary)],check=True,capture_output=True)
    return binary


@pytest.mark.parametrize('mode',['normal','gap','stall','loss','sim_pause','queued','replay','wire'])
def test_compiled_native_watchdog_keeps_500ms_and_latched_stop(native_watchdog,mode):
    subprocess.run([str(native_watchdog),mode],check=True,capture_output=True)


def test_sender_rejects_stale_native_heartbeat_credential():
    from test_stage62_motion import channel_fixture
    channel,node,_callback,_data=channel_fixture()
    received=channel.latest[1];channel.clock=lambda:received+0.251
    with pytest.raises(ObstacleError,match='HEARTBEAT_NATIVE_PROOF_STALE'):channel.send('heartbeat')
    node.advertise.return_value.publish.assert_not_called();channel.close()


def test_new_native_packet_cannot_refresh_stalled_supervisor_proof():
    from test_stage62_motion import channel_fixture
    channel,node,callback,data=channel_fixture();approved=channel.latest
    channel.clock=lambda:approved[1]+0.3
    data.update(sequence=2,sim_time=1.1)
    callback(NS(data=json.dumps(data))) # fresh packet arrives while supervisor is stalled
    with pytest.raises(ObstacleError,match='HEARTBEAT_NATIVE_PROOF_STALE'):
        channel.send('heartbeat',proof=approved)
    node.advertise.return_value.publish.assert_not_called();channel.close()
