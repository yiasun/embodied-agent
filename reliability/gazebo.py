"""Lazy Gazebo Harmonic transport adapter. No guessed calls or model downloads."""
import hashlib
import json
import threading
import time

from .safety import ObstacleError


def value(obj,name):
    item=getattr(obj,name)
    return item() if callable(item) else item


class GazeboBackend:
    def __init__(self,world='default',robot_name='turtlebot3_waffle'):
        # Requires the existing ROS setup environment; never installs bindings.
        from gz.transport13 import Node
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        from gz.msgs10.entity_pb2 import Entity
        from gz.msgs10.scene_pb2 import Scene
        from gz.msgs10.sdf_generator_config_pb2 import SdfGeneratorConfig
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.node=Node();self.world=world;self.robot_name=robot_name
        self.types={'Empty':Empty,'Boolean':Boolean,'EntityFactory':EntityFactory,'Entity':Entity,
                    'Scene':Scene,'SdfGeneratorConfig':SdfGeneratorConfig,'StringMsg':StringMsg,'Pose_V':Pose_V}
        self.bindings=None

    def session_probe(self):
        """A private process: native metadata calls may hold the Python GIL."""
        from .probing import SessionProbe
        return SessionProbe(self.world,self.robot_name)

    def discover(self):
        root='/world/'+self.world
        available=set(self.node.service_list());bindings={};session=[]
        definitions={'create':(['create/blocking','create'],'gz.msgs.EntityFactory','gz.msgs.Boolean'),
            'remove':(['remove/blocking','remove'],'gz.msgs.Entity','gz.msgs.Boolean'),
            'scene':(['scene/info'],'gz.msgs.Empty','gz.msgs.Scene'),
            'sdf':(['generate_world_sdf'],'gz.msgs.SdfGeneratorConfig','gz.msgs.StringMsg')}
        for key,(suffixes,req,rep) in definitions.items():
            found=None
            for suffix in suffixes:
                name=root+'/'+suffix
                if name not in available:continue
                publishers=self.node.service_info(name)
                if len(publishers)!=1:raise ObstacleError('AMBIGUOUS_GAZEBO_SERVICE')
                pub=publishers[0]
                if value(pub,'req_type_name')!=req or value(pub,'rep_type_name')!=rep:
                    raise ObstacleError('GAZEBO_SERVICE_TYPE_MISMATCH')
                found={'name':name,'request_type':req,'response_type':rep}
                session.append([name,str(value(pub,'socket_id'))]);break
            if found is None:raise ObstacleError('REQUIRED_GAZEBO_SERVICE_NOT_DISCOVERED')
            bindings[key]=found
        pose_topic=root+'/pose/info'
        if pose_topic not in set(self.node.topic_list()):raise ObstacleError('POSE_TOPIC_NOT_DISCOVERED')
        publishers,_=self.node.topic_info(pose_topic)
        if len(publishers)!=1 or value(publishers[0],'msg_type_name')!='gz.msgs.Pose_V':
            raise ObstacleError('POSE_TOPIC_TYPE_OR_PUBLISHER_MISMATCH')
        bindings['pose_topic']=pose_topic
        # Movement is audit-only in Stage 6.1: no set_pose call is implemented.
        bindings['movement_services']=[]
        for suffix in ['set_pose','set_pose/blocking','set_pose_vector','set_pose_vector/blocking']:
            name=root+'/'+suffix
            if name in available:
                info=self.node.service_info(name)
                bindings['movement_services'].append({'name':name,'publishers':len(info),
                    'types':[{'request_type':value(p,'req_type_name'),'response_type':value(p,'rep_type_name')} for p in info]})
        bindings['session_id']=hashlib.sha256(json.dumps(sorted(session)).encode()).hexdigest()
        self.bindings=bindings
        return bindings

    def _request(self,key,request,response_type):
        if self.bindings is None:raise ObstacleError('DISCOVERY_REQUIRED')
        ok,response=self.node.request(self.bindings[key]['name'],request,type(request),response_type,2000)
        if not ok:raise ObstacleError('GAZEBO_REQUEST_STATE_UNKNOWN')
        return response

    def entities(self):
        response=self._request('scene',self.types['Empty'](),self.types['Scene'])
        result={}
        for model in response.model:
            if not model.name or not model.id or model.name in result:raise ObstacleError('AMBIGUOUS_SCENE_ENTITY')
            result[model.name]={'id':int(model.id),'name':model.name}
        return result

    def world_sdf(self):
        response=self._request('sdf',self.types['SdfGeneratorConfig'](),self.types['StringMsg'])
        if len(response.data)>4*1024*1024:raise ObstacleError('WORLD_SDF_TOO_LARGE')
        return response.data

    def poses(self):
        event=threading.Event();received=[]
        def callback(message):
            if received:return
            received.append(({int(p.id):{'position':[p.position.x,p.position.y,p.position.z],
                'orientation':[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]}
                for p in message.pose},time.monotonic()))
            event.set()
        topic=self.bindings['pose_topic']
        self.node.subscribe(self.types['Pose_V'],topic,callback)
        try:
            if not event.wait(2.0):raise ObstacleError('FRESH_GAZEBO_POSE_TIMEOUT')
            return received[0]
        finally:self.node.unsubscribe(topic)

    def create(self,name,sdf):
        request=self.types['EntityFactory']();request.name=name;request.sdf=sdf;request.allow_renaming=False
        return bool(self._request('create',request,self.types['Boolean']).data)

    def remove(self,entity_id):
        request=self.types['Entity']();request.id=entity_id;request.type=self.types['Entity'].MODEL
        return bool(self._request('remove',request,self.types['Boolean']).data)
