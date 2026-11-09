"""Test the audited native adapter with fake Node/messages, not a live graph."""
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from reliability.gazebo import GazeboBackend
from reliability.safety import ObstacleError


class Message:
    MODEL=2


class Node:
    def __init__(self):
        self.types={
            '/world/default/create':('gz.msgs.EntityFactory','gz.msgs.Boolean'),
            '/world/default/remove':('gz.msgs.Entity','gz.msgs.Boolean'),
            '/world/default/scene/info':('gz.msgs.Empty','gz.msgs.Scene'),
            '/world/default/generate_world_sdf':('gz.msgs.SdfGeneratorConfig','gz.msgs.StringMsg'),
            '/world/default/set_pose':('gz.msgs.Pose','gz.msgs.Boolean')}
        self.extra_publisher=False;self.request=Mock(return_value=(True,NS(data=True)))
        self.unsubscribe=Mock()
    def service_list(self):return list(self.types)
    def service_info(self,name):
        req,rep=self.types[name]
        pub=NS(req_type_name=lambda:req,rep_type_name=lambda:rep,socket_id=lambda:'server-'+name)
        return [pub,pub] if self.extra_publisher else [pub]
    def topic_list(self):return ['/world/default/pose/info']
    def topic_info(self,name):return ([NS(msg_type_name=lambda:'gz.msgs.Pose_V')],[])
    def subscribe(self,cls,topic,callback):
        callback(NS(pose=[NS(id=10,position=NS(x=1,y=2,z=0.01),orientation=NS(x=0,y=0,z=0,w=1))]))


def backend():
    result=GazeboBackend.__new__(GazeboBackend)
    result.node=Node();result.world='default';result.robot_name='turtlebot3_waffle'
    result.types={k:Message for k in ['Empty','Boolean','EntityFactory','Entity','Scene','SdfGeneratorConfig','StringMsg','Pose_V']}
    result.bindings=None
    return result


def test_discovery_actual_names_types_and_audit_only_movement():
    instance=backend();info=instance.discover()
    assert info['create']['name']=='/world/default/create'
    assert info['remove']['request_type']=='gz.msgs.Entity'
    assert info['movement_services'][0]['types'][0]['request_type']=='gz.msgs.Pose'
    assert len(info['session_id'])==64
    instance.node.request.assert_not_called()


def test_prefers_verified_blocking_service_when_advertised():
    instance=backend();instance.node.types['/world/default/create/blocking']=('gz.msgs.EntityFactory','gz.msgs.Boolean')
    assert instance.discover()['create']['name'].endswith('/blocking')


@pytest.mark.parametrize('fault',['missing','wrong_type','ambiguous'])
def test_discovery_failure_never_sends_request(fault):
    instance=backend()
    if fault=='missing':del instance.node.types['/world/default/create']
    if fault=='wrong_type':instance.node.types['/world/default/create']=('gz.msgs.StringMsg','gz.msgs.Boolean')
    if fault=='ambiguous':instance.node.extra_publisher=True
    with pytest.raises(ObstacleError):instance.discover()
    instance.node.request.assert_not_called()


def test_create_request_uses_inline_real_sdf_no_renaming_and_bounded_deadline():
    instance=backend();instance.discover()
    assert instance.create('owned','<sdf/>')
    args=instance.node.request.call_args.args
    assert args[0]=='/world/default/create' and args[1].name=='owned'
    assert args[1].sdf=='<sdf/>' and args[1].allow_renaming is False
    assert args[-1]==2000


def test_remove_request_uses_exact_model_id_not_blanket_name():
    instance=backend();instance.discover();assert instance.remove(123)
    args=instance.node.request.call_args.args
    assert args[0]=='/world/default/remove' and args[1].id==123 and args[1].type==2
    assert not hasattr(args[1],'name')


def test_timeout_does_not_retry_create():
    instance=backend();instance.discover();instance.node.request.return_value=(False,NS(data=False))
    with pytest.raises(ObstacleError,match='STATE_UNKNOWN'):instance.create('owned','<sdf/>')
    instance.node.request.assert_called_once()


def test_fresh_pose_subscription_and_unsubscribe():
    instance=backend();instance.discover()
    poses,received=instance.poses()
    assert poses[10]['position']==[1,2,0.01]
    instance.node.unsubscribe.assert_called_once_with('/world/default/pose/info')
