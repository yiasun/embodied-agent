import copy
import json
import math
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from test_obstacles import context, FakeGazebo, FakeObserver, route
from reliability import moving, evaluate, lifecycle
from reliability.obstacles import new_record
from reliability.safety import ObstacleError, verify_owned_model
from reliability.trajectory import profile, select_trajectory, motion_sdf
from reliability.streams import decode_poses, PoseStream, MotionChannel
from reliability.capture import Capture, read_execution

ROOT = Path(__file__).resolve().parents[1]


def record(context):
    r = new_record(context, 'motion'); r['motion'] = profile(context['bundle'])
    r['motion']['plugin_path'] = '/tmp/libea_bounded_motion.so'
    r['selection'] = select_trajectory(context['bundle'], route(), [21.5, 22.5], r['motion'])
    return r


def world_sdf(r):
    root = ET.Element('sdf'); world = ET.SubElement(root, 'world', name='default')
    world.append(ET.fromstring(motion_sdf(r)).find('model'))
    return ET.tostring(root, encoding='unicode')


def test_trajectory_derives_free_swept_geometry(context):
    r = record(context)
    assert r['selection']['swept_bounds'][1::2] == [25.3, 26.7]
    assert 15.5 <= r['selection']['position'][0] <= 17.3
    assert r['motion']['y_min'] == 25.5


@pytest.mark.parametrize('kwargs', [{'speed': 0}, {'speed': 1}, {'speed': float('nan')}, {'speed': True},
                                     {'duration': 100}, {'duration': -1}, {'pattern': 'teleport'}])
def test_invalid_motion_profile(context, kwargs):
    with pytest.raises(ObstacleError): profile(context['bundle'], **kwargs)


@pytest.mark.parametrize('fault', ['near_robot', 'not_on_path', 'wall_route', 'nan_path'])
def test_route_generation_fail_closed(context, fault):
    config = profile(context['bundle']); path = route(); robot = [21.5, 22.5]
    if fault == 'near_robot': robot = [16, 26]
    if fault == 'not_on_path': path = [[21, 22], [21, 23]]
    if fault == 'wall_route': config['y_min'] = 24.9
    if fault == 'nan_path': path[0][0] = float('nan')
    with pytest.raises(ObstacleError): select_trajectory(context['bundle'], path, robot, config)


def test_dynamic_sdf_real_collision_inertia_and_plugin(context):
    r = record(context); model = ET.fromstring(motion_sdf(r)).find('model')
    assert model.findtext('static') == 'false'
    assert model.find('link/collision/geometry/box') is not None
    assert model.findtext('link/inertial/mass') == '10'
    assert model.find('plugin').get('name') == 'embodied::BoundedMotion'
    assert model.find('link/sensor').get('type') == 'contact'
    verify_owned_model(world_sdf(r), r)


@pytest.mark.parametrize('fault', ['pose', 'static', 'plugin', 'token', 'size', 'config'])
def test_dynamic_ownership_rejected(context, fault):
    r = record(context); root = ET.fromstring(world_sdf(r)); model = root.find('world/model')
    if fault == 'pose': model.find('pose').text = '100 100 0.6 0 0 0'
    if fault == 'static': model.find('static').text = 'true'
    if fault == 'plugin': model.find('plugin').set('name', 'unowned')
    if fault == 'token': model.find('link').set('name', 'not-owned')
    if fault == 'size': model.find('link/collision/geometry/box/size').text = '1 1 1'
    if fault == 'config': model.find('plugin/speed').text = '10'
    with pytest.raises(ObstacleError): verify_owned_model(ET.tostring(root), r)


def test_dynamic_owned_pose_can_move_within_bounded_route(context):
    r = record(context); root = ET.fromstring(world_sdf(r)); model = root.find('world/model')
    model.find('pose').text = f"{r['selection']['position'][0]} 26.2 0.6 0 0 0"
    verify_owned_model(ET.tostring(root), r)


def test_common_stage61_cleanup_removes_exact_dynamic_entity(context):
    r = record(context); backend = FakeGazebo(context); backend.create(r['entity_name'], motion_sdf(r))
    r.update(create_sent=True, create_ack=True, entity_id=1000, world_session_id=backend.session)
    assert lifecycle.cleanup(backend, r)
    assert r['cleanup'] == 'CLEAN' and backend.remove_calls == [1000]


def test_common_lease_runs_motion_hook_and_cleans(context, tmp_path):
    r = record(context); r.pop('selection'); backend = FakeGazebo(context); callback = Mock()
    code, result = lifecycle.run_lease(context, backend, lambda bundle: FakeObserver(context), r,
                                      tmp_path/'lease.json', motion_controller=callback)
    assert code == 0 and result['cleanup'] == 'CLEAN'
    callback.assert_called_once(); assert len(backend.create_calls) == 1


@pytest.mark.parametrize('fault', [KeyboardInterrupt(), ObstacleError('MOTION_STATE_UNKNOWN'), OSError()])
def test_motion_hook_error_uses_existing_cleanup(context, tmp_path, fault):
    r = record(context); r.pop('selection'); backend = FakeGazebo(context)
    code, result = lifecycle.run_lease(context, backend, lambda bundle: FakeObserver(context), r,
        tmp_path/'lease.json', motion_controller=Mock(side_effect=fault))
    assert code != 0 and result['cleanup'] == 'CLEAN' and backend.remove_calls == [1000]


def test_motion_hook_failure_cleanup_unknown(context, tmp_path):
    r = record(context); r.pop('selection'); backend = FakeGazebo(context); backend.remove_fault = OSError()
    code, result = lifecycle.run_lease(context, backend, lambda bundle: FakeObserver(context), r,
        tmp_path/'lease.json', motion_controller=Mock(side_effect=ObstacleError('UNKNOWN')))
    assert code == 4 and result['cleanup'] == 'UNKNOWN'


def args(context, tmp_path):
    plan = tmp_path/'bundle/plan.json'
    return ['--plan', str(plan), '--run-report', context['report_path']]


def test_moving_default_dry_run_no_files_or_transport(context, tmp_path, capsys):
    factory = Mock(side_effect=AssertionError('No live calls'))
    assert moving.main(args(context, tmp_path)+['--journal', str(tmp_path/'new.json'),
        '--metrics', str(tmp_path/'metrics.json')], backend_factory=factory) == 0
    factory.assert_not_called(); assert not (tmp_path/'new.json').exists()
    assert json.loads(capsys.readouterr().out)['dry_run']


def test_evaluate_default_no_live_calls(context, tmp_path):
    factory = Mock(side_effect=AssertionError('No live calls'))
    assert evaluate.main(args(context, tmp_path)+['--output', str(tmp_path/'new.json')], backend_factory=factory) == 0
    factory.assert_not_called(); assert not (tmp_path/'new.json').exists()


def test_existing_history_not_overwritten(context, tmp_path):
    p = tmp_path/'existing.json'; p.write_text('historical')
    with pytest.raises(FileExistsError): moving.reserve(p)
    assert p.read_text() == 'historical'


def test_read_execution_revalidates_identity(context):
    p = Path(context['report_path']); d = json.loads(p.read_text()); d['run_id'] = 'other'; p.write_text(json.dumps(d))
    with pytest.raises(ObstacleError): read_execution(context)


def pose_message(t=1):
    return NS(header=NS(stamp=NS(sec=t, nsec=0)), pose=[NS(id=1, position=NS(x=1, y=2, z=0),
        orientation=NS(x=0, y=0, z=0, w=1))])


def test_pose_decoding_uses_sim_stamp_and_normalized_quaternion():
    stamp, poses = decode_poses(pose_message()); assert stamp == 1 and poses[1]['yaw'] == 0


@pytest.mark.parametrize('fault', ['zero_stamp', 'bad_quat', 'nan', 'duplicate'])
def test_pose_stream_invalid_evidence(fault):
    msg = pose_message()
    if fault == 'zero_stamp': msg.header.stamp.sec = 0
    if fault == 'bad_quat': msg.pose[0].orientation.w = 0
    if fault == 'nan': msg.pose[0].position.x = math.nan
    if fault == 'duplicate': msg.pose.append(msg.pose[0])
    with pytest.raises(ObstacleError): decode_poses(msg)


class Stream:
    def __init__(self, batches): self.batches = list(batches); self.closed = False
    def drain(self): return self.batches.pop(0) if self.batches else []
    def close(self): self.closed = True


def capture_fixture(context, batches):
    backend = FakeGazebo(context)
    observer = NS(poll=Mock(return_value=None), sim_now=1.2)
    stream = Stream(batches)
    return Capture(context, backend, observer, stream_factory=lambda b: stream), stream


def ground_truth(context, t, x=21.5, age=0):
    b = FakeGazebo(context); robot_id = b.entities()[b.robot_name]['id']
    return (t, {robot_id: {'position': [x, 22.5, 0.01], 'yaw': 0}}, time.monotonic()-age)


def test_capture_records_goal_and_state_without_actions(context):
    a = ground_truth(context, 1, age=0.1); b = ground_truth(context, 1.1, 21.51)
    cap, stream = capture_fixture(context, [[a, b]])
    assert cap.tick()
    s = cap.report()['samples'][-1]
    assert s['goal'] == {'id': 3, 'x': 14.5, 'y': 29.5}
    assert s['task_status'] == 'RUNNING' and s['robot_velocity_world'][0] == pytest.approx(0.1)
    cap.close(); assert stream.closed


def test_capture_rejects_ros_gazebo_time_mismatch(context):
    cap, _ = capture_fixture(context, [[ground_truth(context, 3)]])
    with pytest.raises(ObstacleError, match='CLOCK_MISMATCH'): cap.tick()


def test_capture_rejects_stale_pose(context):
    cap, _ = capture_fixture(context, [[ground_truth(context, 1, age=1)]])
    with pytest.raises(ObstacleError, match='STALE'): cap.tick()


def test_downsampling_still_detects_raw_clock_reset(context):
    a = ground_truth(context, 1, age=0.1); b = ground_truth(context, 1.02, age=0.05)
    c = ground_truth(context, 1.01)
    cap, _ = capture_fixture(context, [[a, b, c]])
    with pytest.raises(ObstacleError, match='RESET'): cap.tick()


def test_capture_robot_jump_marks_incomplete_and_stops(context):
    cap, _ = capture_fixture(context, [[ground_truth(context, 1, age=0.1), ground_truth(context, 1.1, 15)]])
    with pytest.raises(ObstacleError, match='JUMP'): cap.tick()
    assert cap.report()['quality']['status'] == 'INCOMPLETE'


def test_capture_terminal_run_keeps_real_failure(context):
    p = Path(context['report_path']); d = json.loads(p.read_text()); d['status'] = 'FAILED'; p.write_text(json.dumps(d))
    cap, _ = capture_fixture(context, [[ground_truth(context, 1)]])
    assert cap.tick() is False
    assert cap.report()['v4']['status'] == 'FAILED'


def test_pose_callback_errors_not_swallowed_or_logged_as_raw_messages():
    node = Mock(); node.subscribe.return_value = True
    backend = NS(node=node, bindings={'pose_topic': '/world/default/pose/info'}, types={'Pose_V': object})
    stream = PoseStream(backend); callback = node.subscribe.call_args.args[2]
    callback(pose_message(0))
    with pytest.raises(ObstacleError, match='STAMP'): stream.drain()


class Clock:
    def __init__(self): self.t = 100
    def monotonic(self): self.t += 0.001; return self.t


@pytest.mark.parametrize('fault', ['none', 'state_stale', 'native_fault', 'capture_error', 'interrupt', 'no_motion', 'stop_unknown'])
def test_controller_heartbeat_stop_and_diagnostics(context, tmp_path, monkeypatch, fault):
    clock = Clock(); monkeypatch.setattr(moving, 'time', NS(monotonic=clock.monotonic))
    r = record(context); r.update(active_task_id=3, world_session_id='a'*64)
    class Channel:
        def __init__(self): self.calls=0; self.commands=[]; self.stop=False; self.closed=False
        def state(self):
            self.calls += 1
            stopped = (self.stop or self.calls >= 4) and fault != 'stop_unknown'
            return ({'sequence': self.calls, 'sim_time': 1+self.calls*0.1, 'stopped': stopped,
                     'reason': 'ROBOT_CLEARANCE_GUARD' if fault == 'native_fault' else 'TRAJECTORY_COMPLETE' if stopped else 'MOVING',
                     'commanded_vy': 0 if stopped else 0.15}, clock.monotonic()-(1 if fault == 'state_stale' else 0))
        def send(self, value, **kwargs): self.commands.append(value); self.stop = self.stop or value == 'stop'
        def close(self): self.closed=True
    channel = Channel()
    class Recorder:
        def __init__(self): self.metrics=NS(samples=[]); self.execution={'tasks': [{'id': 3, 'status': 'RUNNING'}]}; self.closed=False
        def tick(self, state):
            if fault == 'capture_error': raise ObstacleError('BAD_SAMPLE')
            if fault == 'interrupt': raise KeyboardInterrupt
            n = len(self.metrics.samples)
            self.metrics.samples.append({'obstacle': {'position': [16, 25.5+(0 if fault == 'no_motion' else n*0.1), 0.6]}, 'segment_valid': True,
                'received_mono':clock.monotonic(),'task_id':3,'task_status':'RUNNING','geometric_clearance_m':4.0})
            return True
        def report(self, error=None): return {'error': error}
        def close(self): self.closed=True
    recorder=Recorder(); checkpoint=Mock(); backend=NS(discover=lambda: {'session_id': 'a'*64})
    call = lambda: moving.controller(context, backend, None, r, tmp_path/'lease.json', Mock(),
        channel_factory=lambda *a: channel, capture_factory=lambda *a: recorder, sleep=lambda t: None, checkpoint=checkpoint)
    if fault == 'none': call()
    else:
        with pytest.raises((ObstacleError, KeyboardInterrupt)): call()
    assert channel.commands[-1] == 'stop' and channel.closed and recorder.closed
    assert r['motion_stop'] == ('UNKNOWN' if fault in ('stop_unknown', 'state_stale') else 'LATCHED_ZERO_COMMAND')
    checkpoint.assert_called()


def test_apply_path_uses_shared_lock_and_never_launches_navigation(context, tmp_path, monkeypatch):
    backend=FakeGazebo(context); plugin=tmp_path/'plugin.so'; plugin.write_bytes(b'offline fake, not loaded')
    callback=Mock(); monkeypatch.setattr(moving, 'ROOT', ROOT)
    code=moving.main(args(context, tmp_path)+['--journal', str(tmp_path/'motion.json'), '--metrics', str(tmp_path/'metrics.json'),
        '--plugin', str(plugin), '--apply'], backend_factory=lambda: backend,
        observer_factory=lambda bundle: FakeObserver(context), controller_fn=callback)
    assert code==0 and len(backend.create_calls)==1 and backend.remove_calls==[1000]
    d=json.loads((tmp_path/'metrics.json').read_text())
    assert d['lease']['cleanup']=='CLEAN' and d['lease']['journal']==str(tmp_path/'motion.json')
    assert json.loads((tmp_path/'motion.json').read_text())['navigation_started_by_tool'] is False


def test_existing_lock_blocks_motion_apply_before_creation(context, tmp_path):
    from reliability.obstacles import exclusive_test_lock
    plugin=tmp_path/'plugin.so'; plugin.write_bytes(b'fake')
    backend=Mock()
    with exclusive_test_lock():
        code=moving.main(args(context, tmp_path)+['--journal', str(tmp_path/'motion.json'), '--metrics', str(tmp_path/'metrics.json'),
            '--plugin', str(plugin), '--apply'], backend_factory=backend)
    assert code==1; backend.assert_not_called(); assert not (tmp_path/'motion.json').exists()


def test_offline_finalize_keeps_window_and_input_unchanged(context, tmp_path):
    from reliability.metrics import Metrics
    from reliability.capture import identity
    p=Path(context['report_path']); d=json.loads(p.read_text()); d['status']='FAILED'; p.write_text(json.dumps(d))
    data=Metrics(identity(context)).report({'status': 'RUNNING'}); source=tmp_path/'observation.json'
    source.write_text(json.dumps(data)); old=source.read_bytes()
    factory=Mock(side_effect=AssertionError('offline only'))
    code=evaluate.main(args(context, tmp_path)+['--output', str(tmp_path/'final.json'),
        '--from-evaluation', str(source), '--record'], backend_factory=factory)
    assert code==0; factory.assert_not_called()
    result=json.loads((tmp_path/'final.json').read_text())
    assert result['v4']['status']=='FAILED' and result['v4_at_observation_end']['status']=='RUNNING'
    assert source.read_bytes()==old and result['observation_window_unchanged'] is True


def test_finalize_rejects_wrong_run_identity(context, tmp_path):
    from reliability.metrics import Metrics
    from reliability.capture import identity
    data=Metrics(identity(context)).report(); data['run_id']='another-run'
    p=tmp_path/'observation.json'; p.write_text(json.dumps(data))
    with pytest.raises(ObstacleError, match='IDENTITY'): evaluate.finalize(context, p)


def channel_fixture():
    node=Mock(); publisher=NS(msg_type_name=lambda: 'gz.msgs.StringMsg')
    node.topic_info.side_effect=lambda topic:([publisher],[]) if topic.endswith('/state') else ([],[publisher])
    record={'lease_id': 'a'*32, 'entity_id': 125,'entity_name':'ea_stage61_'+'a'*32}
    data={'protocol':MotionChannel.PROTOCOL,'configured':True,'configure_status':'READY','pose_valid':True,
          'entity_name':record['entity_name'],'frame':'gazebo_world',
          'token': 'a'*32, 'entity_id': 125, 'sequence': 1, 'sim_time': 1.0,
          'position': [16,25.5,0.6], 'yaw': 0, 'commanded_vy': 0, 'stopped': False,
          'reason': 'WAITING_FOR_HEARTBEAT', 'contact_measured': False, 'robot_contacts': 0}
    def subscribe(cls,topic,callback):callback(NS(data=json.dumps(data)));return True
    node.subscribe.side_effect=subscribe
    channel=MotionChannel(NS(node=node), record, message_type=NS)
    return channel,node,node.subscribe.call_args.args[2],data


def test_native_channel_publishes_only_heartbeat_and_stop():
    channel,node,cb,data=channel_fixture()
    assert channel.state()[0]['entity_id']==125
    channel.send('heartbeat'); channel.send('stop')
    assert [c.args[0].data for c in node.advertise.return_value.publish.call_args_list]==['heartbeat:1:1','stop']
    with pytest.raises(ObstacleError): channel.send('set_pose')
    channel.close(); node.unsubscribe.assert_called_once()


@pytest.mark.parametrize('fault',['token','entity','sequence','nan','position'])
def test_native_state_corruption_latches_diagnostic(fault):
    channel,node,cb,data=channel_fixture()
    data['sequence']=2;data['sim_time']=1.1
    if fault=='token':data['token']='b'*32
    if fault=='entity':data['entity_id']=999
    if fault=='sequence':data['sequence']=1
    if fault=='nan':data['yaw']=float('nan')
    if fault=='position':data['position']=[16,25]
    cb(NS(data=json.dumps(data)))
    with pytest.raises(ObstacleError):channel.state()


def test_ambiguous_native_topic_no_commands_or_subscription():
    node=Mock();p=NS(msg_type_name=lambda:'gz.msgs.StringMsg')
    node.topic_info.side_effect=[([p,p],[]),([],[p])]
    with pytest.raises(ObstacleError):MotionChannel(NS(node=node),{'lease_id':'a'*32},message_type=NS)
    node.advertise.assert_not_called();node.subscribe.assert_not_called()


@pytest.fixture(scope='module')
def native_harness(tmp_path_factory):
    compiler = shutil.which('g++')
    if not compiler: pytest.skip('C++ compiler unavailable; report this separately')
    directory = tmp_path_factory.mktemp('native_profile'); source = directory/'test.cc'
    source.write_text(r'''
#include "BoundedProfile.hh"
#include <cassert>
#include <limits>
int main(int argc,char **argv) {
  const std::string mode=argv[1]; embodied::BoundedProfile p; p.x=16;
  double x=16,y=25.5,z=.6,yaw=0,rx=21,ry=22,t=1,wall=0; bool hb=true,contact=false;
  p.Step(t,x,y,z,yaw,rx,ry,hb,contact,wall);
  if(mode=="heartbeat") hb=false;
  if(mode=="contact") contact=true;
  if(mode=="jump") x+=.1;
  if(mode=="near") { rx=x;ry=y; }
  if(mode=="reset") t-=.1;
  if(mode=="clockgap") t+=1;
  if(mode=="attitude") yaw=.1;
  if(mode=="nan") x=std::numeric_limits<double>::quiet_NaN();
  if(mode=="wall") wall=61;
  if(mode!="cross" && mode!="shuttle") {
    p.Step(t+.003,x,y,z,yaw,rx,ry,hb,contact,wall); assert(p.stopped);
    assert(p.Step(t+.006,16,25.5,.6,0,21,22,true,false,0)==0); return 0;
  }
  p.shuttle=mode=="shuttle"; bool reversed=false; double last=0;
  for(int i=0;i<11000 && !p.stopped;i++) {
    t+=.003; const double v=p.Step(t,x,y,z,yaw,rx,ry,true,false,0);
    assert(std::abs(v)<=p.speed+1e-9); y+=v*.003;
    assert(y>=25.46 && y<=26.54); if(v<0) reversed=true; last=v;
  }
  assert(p.stopped); assert(p.velocity==0);
  if(p.shuttle) assert(reversed); else assert(y>26.45);
}
''')
    exe = directory/'profile_test'
    subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Wno-unused-parameter', '-Wno-unused-but-set-variable',
        '-I', str(ROOT/'reliability/native'), str(source), '-o', str(exe)], check=True, capture_output=True)
    return exe


@pytest.mark.parametrize('case', ['cross', 'shuttle', 'heartbeat', 'contact', 'jump', 'near', 'reset',
                                  'clockgap', 'attitude', 'nan', 'wall'])
def test_compiled_native_profile_deterministic_guards(native_harness, case):
    subprocess.run([str(native_harness), case], check=True, capture_output=True)
