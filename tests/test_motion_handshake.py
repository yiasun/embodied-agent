"""Native initialization contract with fake transport/time; no Gazebo or ROS node."""
import copy
import json
import shutil
import subprocess
import time
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from test_obstacles import context, FakeGazebo, FakeObserver
from reliability import moving
from reliability.streams import MotionChannel
from reliability.safety import ObstacleError


class Clock:
    def __init__(self): self.t=100.0;self.hook=lambda:None
    def now(self):return self.t
    def sleep(self,seconds):self.t+=seconds;self.hook()


def record():
    return {'lease_id':'a'*32,'entity_id':117,'entity_name':'ea_stage61_'+'a'*32,
            'selection':{'position':[16,25.5,0.6]}}


def state(lease):
    return {'protocol':MotionChannel.PROTOCOL,'frame':'gazebo_world','configured':True,
        'configure_status':'READY','pose_valid':True,'token':lease['lease_id'],
        'entity_name':lease['entity_name'],'entity_id':lease['entity_id'],
        'sequence':1,'sim_time':20.0,'position':[16,25.5,0.6],'yaw':0,'commanded_vy':0,
        'stopped':False,'reason':'WAITING_FOR_HEARTBEAT','contact_measured':False,'robot_contacts':0}


class Node:
    def __init__(self,clock,lease):
        self.clock=clock;self.data=state(lease);self.ready_after=0;self.state_after=0
        self.publishers=1;self.subscribers=1;self.state_type=self.control_type='gz.msgs.StringMsg'
        self.callback=None;self.sent=False;self.calls=0;self.query_error=None
        self.unsubscribe=Mock();self.advertise=Mock(return_value=NS(valid=lambda:True,publish=Mock(return_value=True)))
        self.subscribe=Mock(side_effect=self._subscribe);clock.hook=self.tick
    def topic_info(self,topic):
        self.calls+=1
        if self.query_error:raise self.query_error
        p=self.publishers if self.clock.t>=100+self.ready_after else 0
        s=self.subscribers if self.clock.t>=100+self.ready_after else 0
        return ([NS(msg_type_name=lambda:self.state_type)]*p,[]) if topic.endswith('/state') else ([],[NS(msg_type_name=lambda:self.control_type)]*s)
    def _subscribe(self,cls,topic,callback):
        self.callback=callback;self.tick();return True
    def tick(self):
        if self.callback and not self.sent and self.state_after is not None and self.clock.t>=100+self.state_after:
            self.sent=True;self.callback(NS(data=json.dumps(self.data)))


def connect(node,clock,lease,timeout=0.3):
    return MotionChannel(NS(node=node),lease,message_type=NS,ready_timeout=timeout,
                         clock=clock.now,sleep=clock.sleep)


def test_normal_handshake_idle_identity_before_control_publication():
    lease=record();clock=Clock();node=Node(clock,lease)
    channel=connect(node,clock,lease)
    assert lease['motion_channel']['phase']=='READY'
    assert lease['motion_channel']['initial_state_received'] is True
    assert channel.state()[0]['entity_id']==117
    node.advertise.return_value.publish.assert_not_called()
    channel.send('heartbeat');channel.send('stop');channel.close();channel.close()
    assert [c.args[0].data for c in node.advertise.return_value.publish.call_args_list]==['heartbeat:1:1','stop']
    node.unsubscribe.assert_called_once()


def test_delayed_plugin_registration_polling_succeeds_without_heartbeat():
    lease=record();clock=Clock();node=Node(clock,lease);node.ready_after=0.15
    channel=connect(node,clock,lease)
    diagnostic=lease['motion_channel']
    assert diagnostic['attempts']>1 and 0.1<diagnostic['elapsed_wall_s']<0.3
    assert diagnostic['observations'][0]['state_publishers']==0
    assert diagnostic['state_publishers']==diagnostic['control_subscribers']==1
    node.advertise.return_value.publish.assert_not_called();channel.close()


def test_endpoints_without_initial_message_are_not_ready():
    lease=record();clock=Clock();node=Node(clock,lease);node.state_after=None
    with pytest.raises(ObstacleError,match='HANDSHAKE_TIMEOUT'):connect(node,clock,lease)
    d=lease['motion_channel']
    assert d['state_publishers']==d['control_subscribers']==1
    assert d['initial_state_received'] is False and d['last_error']=='INITIAL_STATE_NOT_RECEIVED'
    node.advertise.assert_not_called();node.unsubscribe.assert_called_once()


def test_delayed_initial_message_after_valid_discovery():
    lease=record();clock=Clock();node=Node(clock,lease);node.state_after=0.15
    channel=connect(node,clock,lease)
    assert lease['motion_channel']['initial_state_received'] and lease['motion_channel']['attempts']>1
    channel.close()


def test_missing_endpoints_timeout_has_actual_counts_and_types():
    lease=record();clock=Clock();node=Node(clock,lease);node.publishers=node.subscribers=0
    with pytest.raises(ObstacleError,match='HANDSHAKE_TIMEOUT'):connect(node,clock,lease)
    d=lease['motion_channel'];assert d['elapsed_wall_s']>=0.3-1e-9
    assert d['state_publishers']==d['control_subscribers']==0 and d['state_types']==d['control_types']==[]
    assert d['last_error']=='STATE_PUBLISHER_MISSING+CONTROL_SUBSCRIBER_MISSING'
    node.advertise.assert_not_called();node.subscribe.assert_not_called()


@pytest.mark.parametrize('duplicate',['publishers','subscribers'])
def test_duplicate_endpoint_fails_immediately(duplicate):
    lease=record();clock=Clock();node=Node(clock,lease);setattr(node,duplicate,2)
    with pytest.raises(ObstacleError,match='DUPLICATE_ENDPOINT'):connect(node,clock,lease)
    assert lease['motion_channel']['attempts']==1
    node.advertise.assert_not_called();node.subscribe.assert_not_called()


@pytest.mark.parametrize('wrong',['state_type','control_type'])
def test_wrong_endpoint_type_fails_immediately(wrong):
    lease=record();clock=Clock();node=Node(clock,lease);setattr(node,wrong,'gz.msgs.Pose')
    with pytest.raises(ObstacleError,match='WRONG_TYPE'):connect(node,clock,lease)
    assert 'gz.msgs.Pose' in lease['motion_channel']['state_types']+lease['motion_channel']['control_types']
    node.advertise.assert_not_called()


@pytest.mark.parametrize('identity',['token','entity_id','entity_name'])
def test_wrong_initial_identity_never_sends_control(identity):
    lease=record();clock=Clock();node=Node(clock,lease)
    node.data[identity]=999 if identity=='entity_id' else 'b'*32
    with pytest.raises(ObstacleError,match='IDENTITY_MISMATCH'):connect(node,clock,lease)
    assert lease['motion_channel']['last_error']=='NATIVE_MOTION_IDENTITY_MISMATCH'
    node.advertise.assert_not_called();node.unsubscribe.assert_called_once()


def test_old_library_protocol_requires_rebuild():
    lease=record();clock=Clock();node=Node(clock,lease);del node.data['protocol']
    with pytest.raises(ObstacleError,match='PROTOCOL_MISMATCH'):connect(node,clock,lease)
    node.advertise.assert_not_called()


@pytest.mark.parametrize('reason',['INVALID_CONFIG','MISSING_ENTITIES','CONTROL_SUBSCRIPTION_FAILED'])
def test_plugin_reports_configure_failure_even_without_control_endpoint(reason):
    lease=record();clock=Clock();node=Node(clock,lease);node.subscribers=0
    node.data.update(configured=False,configure_status='FAILED',stopped=True,reason=reason)
    with pytest.raises(ObstacleError,match='NATIVE_CONFIGURE_FAILED_'+reason):connect(node,clock,lease)
    assert lease['motion_channel']['last_native_state']['reason']==reason
    assert lease['motion_channel']['control_subscribers']==0
    node.advertise.assert_not_called()


@pytest.mark.parametrize('fault,reason',[
    ('moving','INITIAL_STATE_NOT_IDLE'),('stopped','INITIAL_STOP'),
    ('pose','INITIAL_POSE_MISMATCH'),('unknown_pose','INITIAL_POSE_NOT_VALID'),
    ('bool_id','IDENTITY_MISMATCH'),('bool_time','INVALID_NATIVE_MOTION_TELEMETRY')])
def test_initial_state_must_be_valid_idle_and_owned(fault,reason):
    lease=record();clock=Clock();node=Node(clock,lease)
    if fault=='moving':node.data.update(commanded_vy=0.1,reason='MOVING')
    if fault=='stopped':node.data.update(stopped=True,reason='ROBOT_CLEARANCE_GUARD')
    if fault=='pose':node.data['position'][0]+=1
    if fault=='unknown_pose':node.data['pose_valid']=False
    if fault=='bool_id':node.data['entity_id']=True
    if fault=='bool_time':node.data['sim_time']=True
    with pytest.raises(ObstacleError,match=reason):connect(node,clock,lease)
    node.advertise.assert_not_called()


def test_late_ready_state_cannot_extend_deadline():
    lease=record();clock=Clock();node=Node(clock,lease);node.state_after=0.4
    with pytest.raises(ObstacleError,match='HANDSHAKE_TIMEOUT'):connect(node,clock,lease)
    assert lease['motion_channel']['phase']=='FAILED';node.advertise.assert_not_called()


def test_endpoint_duplicate_appears_at_final_recheck():
    lease=record();clock=Clock();node=Node(clock,lease);base=node.topic_info
    def info(topic):
        if node.calls>=2:node.subscribers=2
        return base(topic)
    node.topic_info=info
    with pytest.raises(ObstacleError,match='DUPLICATE_ENDPOINT'):connect(node,clock,lease)
    node.advertise.assert_not_called()


def test_discovery_error_is_bounded_and_exception_text_is_not_logged():
    lease=record();clock=Clock();node=Node(clock,lease);node.query_error=RuntimeError('private sentinel')
    with pytest.raises(ObstacleError,match='HANDSHAKE_TIMEOUT'):connect(node,clock,lease)
    assert lease['motion_channel']['last_error']=='TOPIC_INFO_RuntimeError'
    assert 'private sentinel' not in json.dumps(lease)


def test_stale_initial_state_cannot_pass_final_recheck():
    lease=record();clock=Clock();node=Node(clock,lease);base=node.topic_info
    def info(topic):
        if node.calls==2:clock.t+=0.4
        return base(topic)
    node.topic_info=info
    with pytest.raises(ObstacleError,match='HANDSHAKE_TIMEOUT'):connect(node,clock,lease)
    assert lease['motion_channel']['last_error']=='NATIVE_INITIAL_STATE_STALE'
    node.advertise.assert_not_called()


def test_handshake_diagnostics_survive_lease_cleanup_and_metrics(context,tmp_path):
    backend=FakeGazebo(context);capture=Mock(side_effect=AssertionError('capture before readiness'))
    def controller(*args,checkpoint=None):
        def factory(backend,lease):
            clock=Clock();node=Node(clock,lease);node.publishers=node.subscribers=0
            return connect(node,clock,lease,timeout=0.1)
        moving.controller(*args,channel_factory=factory,capture_factory=capture,checkpoint=checkpoint)
    plugin=tmp_path/'offline.so';plugin.write_bytes(b'offline only')
    code=moving.main(['--plan',str(tmp_path/'bundle/plan.json'),'--run-report',context['report_path'],
        '--journal',str(tmp_path/'lease.json'),'--metrics',str(tmp_path/'metrics.json'),
        '--plugin',str(plugin),'--apply'],backend_factory=lambda:backend,
        observer_factory=lambda bundle:FakeObserver(context),controller_fn=controller)
    lease=json.loads((tmp_path/'lease.json').read_text());metrics=json.loads((tmp_path/'metrics.json').read_text())
    assert code==1 and lease['create_sent'] and lease['cleanup']=='CLEAN' and lease['entity_id']==1000
    assert lease['motion_stop']=='NOT_STARTED' and lease['error']=='MOTION_HANDSHAKE_TIMEOUT'
    assert metrics['quality']['sample_count']==0 and metrics['motion_channel']==lease['motion_channel']
    assert backend.remove_calls==[1000];capture.assert_not_called()


def test_changed_world_session_after_handshake_never_sends_heartbeat(context,tmp_path):
    lease={'world_session_id':'a'*64};capture=Mock()
    channel=NS(send=Mock(),close=Mock(),state=lambda:({'stopped':True,'commanded_vy':0},time.monotonic()))
    with pytest.raises(ObstacleError,match='WORLD_SESSION_CHANGED_DURING_HANDSHAKE'):
        moving.controller(context,NS(discover=lambda:{'session_id':'b'*64}),None,lease,tmp_path/'lease.json',Mock(),
                          channel_factory=lambda *a:channel,capture_factory=capture)
    assert [c.args[0] for c in channel.send.call_args_list]==['stop']
    capture.assert_not_called();channel.close.assert_called_once()


@pytest.mark.parametrize('plugin_class',[None,'embodied::BoundedMotionHandshakeV1'])
def test_versioned_plugin_identity_and_legacy_cleanup_compatibility(context,plugin_class):
    import xml.etree.ElementTree as ET
    from test_stage62_motion import record as make_record,world_sdf
    from reliability.trajectory import motion_sdf
    from reliability.safety import verify_owned_model
    lease=make_record(context)
    if plugin_class:lease['motion']['plugin_class']=plugin_class
    name=ET.fromstring(motion_sdf(lease)).find('model/plugin').get('name')
    assert name==(plugin_class or 'embodied::BoundedMotion')
    verify_owned_model(world_sdf(lease),lease)


def test_unknown_plugin_class_rejected_before_sdf_creation(context):
    from test_stage62_motion import record as make_record
    from reliability.trajectory import motion_sdf
    lease=make_record(context);lease['motion']['plugin_class']='untrusted::Plugin'
    with pytest.raises(ObstacleError,match='INVALID_MOTION_OWNERSHIP_PROFILE'):motion_sdf(lease)


@pytest.fixture(scope='module')
def native_startup(tmp_path_factory):
    compiler=shutil.which('g++')
    if not compiler:pytest.skip('C++ compiler unavailable')
    d=tmp_path_factory.mktemp('startup');source=d/'startup.cc'
    source.write_text(r'''
#include "StartupCheck.hh"
#include <cassert>
#include <limits>
int main(int, char **argv) {
 const std::string mode=argv[1];
 const std::string token(32,'a');
 assert(embodied::ValidMotionConfig(token,15.5,25.5,26.5,.15,15,"cross"));
 if(mode=="invalid") {
   assert(!embodied::ValidMotionConfig(token,15.5,25.5,26.5,.31,15,"cross"));
   assert(!embodied::ValidMotionConfig(token,15.5,25.5,26.5,.15,31,"cross"));
   assert(!embodied::ValidMotionConfig(token,15.5,25.5,26.5,std::numeric_limits<double>::quiet_NaN(),15,"cross"));
   assert(embodied::StartupResult(false,true,true,true,true,true)=="INVALID_CONFIG");
 } else if(mode=="link") assert(embodied::StartupResult(true,true,false,true,true,true)=="MISSING_ENTITIES");
 else if(mode=="robot") assert(embodied::StartupResult(true,true,true,false,true,true)=="MISSING_ENTITIES");
 else if(mode=="name") assert(embodied::StartupResult(true,false,true,true,true,true)=="ENTITY_NAME_MISMATCH");
 else if(mode=="publish") assert(embodied::StartupResult(true,true,true,true,false,true)=="STATE_PUBLICATION_FAILED");
 else if(mode=="subscribe") assert(embodied::StartupResult(true,true,true,true,true,false)=="CONTROL_SUBSCRIPTION_FAILED");
 else assert(embodied::StartupResult(true,true,true,true,true,true)=="READY");
}
''')
    binary=d/'startup';root=Path(__file__).resolve().parents[1]
    subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Werror','-I',str(root/'reliability/native'),
                    str(source),'-o',str(binary)],check=True,capture_output=True)
    return binary


@pytest.mark.parametrize('case',['invalid','link','robot','name','publish','subscribe','ready'])
def test_compiled_native_startup_paths(native_startup,case):
    subprocess.run([str(native_startup),case],check=True,capture_output=True)
