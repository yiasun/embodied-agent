"""Continuous read-only pose stream plus lease-scoped native control, lazy imports."""
import math
import copy
import threading
import time
from collections import deque

from planner.validation import strict_json_loads
from .gazebo import value
from .safety import ObstacleError


def decode_poses(message):
    stamp = message.header.stamp.sec + message.header.stamp.nsec / 1e9
    if not math.isfinite(stamp) or stamp <= 0:
        raise ObstacleError('INVALID_GAZEBO_SIM_STAMP')
    poses = {}
    for pose in message.pose:
        p, q = pose.position, pose.orientation
        numbers = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if any(not math.isfinite(v) for v in numbers) or abs(sum(v*v for v in numbers[3:])-1) > 0.01:
            raise ObstacleError('INVALID_GAZEBO_POSE_OR_QUATERNION')
        if pose.id in poses:
            raise ObstacleError('DUPLICATE_GAZEBO_ENTITY_POSE')
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        poses[int(pose.id)] = {'position': [p.x, p.y, p.z], 'yaw': yaw}
    return stamp, poses


class PoseStream:
    def __init__(self, backend):
        self.backend = backend; self.lock = threading.Lock(); self.queue = deque(); self.error = None
        self.topic = backend.bindings['pose_topic']
        def callback(message):
            received = time.monotonic()  # callback entry, not decoding completion
            with self.lock:
                try:
                    if len(self.queue) >= 300:
                        raise ObstacleError('POSE_BUFFER_OVERFLOW')
                    stamp, poses = decode_poses(message)
                    self.queue.append((stamp, poses, received))
                except Exception as exc:
                    self.error = str(exc) if isinstance(exc, ObstacleError) else type(exc).__name__
        if not backend.node.subscribe(backend.types['Pose_V'], self.topic, callback):
            raise ObstacleError('POSE_SUBSCRIPTION_FAILED')

    def drain(self):
        with self.lock:
            if self.error:
                raise ObstacleError(self.error)
            result = list(self.queue); self.queue.clear(); return result

    def close(self):
        self.backend.node.unsubscribe(self.topic)


class MotionChannel:
    PROTOCOL='ea-motion-heartbeat-v2'

    def __init__(self, backend, record, message_type=None, *, ready_timeout=3.0,
                 poll_interval=0.05, clock=time.monotonic, sleep=time.sleep):
        """No heartbeat until endpoint discovery AND an owned, idle initial state.

        Failed constructors retain diagnostics in the lease and release their
        subscription. A discovery timeout never implies a plugin load failure.
        """
        if message_type is None:
            from gz.msgs10.stringmsg_pb2 import StringMsg
            message_type = StringMsg
        self.cls = message_type; self.backend = backend; self.record = record
        self.lock = threading.Lock(); self.latest = None; self.error = None
        self.clock=clock;self._ready=False;self._subscribed=False;self._closed=False;self.publisher=None
        self._heartbeat_sequence=0
        root = '/ea_stage62/' + record['lease_id']; self.topic = root + '/state';self.control=root+'/control'
        started=clock();deadline=started+ready_timeout
        self._diagnostic={'protocol':self.PROTOCOL,'phase':'DISCOVERING','timeout_wall_s':ready_timeout,
            'poll_interval_wall_s':poll_interval,'attempts':0,'state_topic':self.topic,'control_topic':self.control,
            'state_publishers':None,'control_subscribers':None,'state_types':[],'control_types':[],
            'last_error':None,'observations':[],'initial_state_received':False}
        self._diagnostic['plugin_startup']='NOT_CONFIRMED'
        record['motion_channel']=self._diagnostic
        try:
            if (not math.isfinite(ready_timeout) or not 0<ready_timeout<=10
                    or not math.isfinite(poll_interval) or not 0<poll_interval<=0.1):
                raise ObstacleError('INVALID_MOTION_HANDSHAKE_DEADLINE')
            while clock()<deadline:
                self._diagnostic['attempts']+=1
                endpoints=self._endpoints()
                if self._diagnostic['state_publishers']==1 and not self._subscribed:
                    self._subscribed=bool(backend.node.subscribe(message_type,self.topic,self._callback))
                    if not self._subscribed:raise ObstacleError('MOTION_STATE_SUBSCRIPTION_FAILED')
                state=self._idle_initial_state()
                if state and endpoints and clock()<deadline:
                    # Recheck discovery AND telemetry after asynchronous registration.
                    if self._endpoints():
                        state=self._idle_initial_state()
                        if state and clock()<deadline:
                            self.publisher=backend.node.advertise(self.control,message_type)
                            if not self.publisher.valid():raise ObstacleError('MOTION_CONTROL_ADVERTISE_FAILED')
                            if clock()>=deadline:raise ObstacleError('MOTION_HANDSHAKE_TIMEOUT')
                            self._ready=True;self._diagnostic.update(phase='READY',initial_sequence=state['sequence'])
                            break
                elif endpoints and state is None and self.latest is None:
                    self._diagnostic.update(phase='WAITING_INITIAL_STATE',last_error='INITIAL_STATE_NOT_RECEIVED')
                sleep(min(poll_interval,max(0,deadline-clock())))
            else:raise ObstacleError('MOTION_HANDSHAKE_TIMEOUT')
        except BaseException as exc:
            failure=str(exc) if isinstance(exc,ObstacleError) else type(exc).__name__
            self._diagnostic.update(phase='FAILED',failure=failure)
            if failure!='MOTION_HANDSHAKE_TIMEOUT':self._diagnostic['last_error']=failure
            try:self.close()
            except BaseException as close_error:self._diagnostic['close_error']=type(close_error).__name__
            raise
        finally:self._diagnostic['elapsed_wall_s']=clock()-started

    def _idle_initial_state(self):
        observation=self.state()
        if observation is None:return None
        state,received=observation;self._diagnostic['initial_state_received']=True
        self._diagnostic['plugin_startup']=state['configure_status']
        self._diagnostic['last_native_state']={k:state[k] for k in
            ('configure_status','reason','stopped','commanded_vy','sequence','sim_time','entity_id','entity_name')}
        if not state['configured'] or state['configure_status']!='READY':
            raise ObstacleError('NATIVE_CONFIGURE_FAILED_'+state['reason'])
        if state['stopped']:raise ObstacleError('NATIVE_INITIAL_STOP_'+state['reason'])
        if abs(state['commanded_vy'])>1e-9 or state['reason']!='WAITING_FOR_HEARTBEAT':
            raise ObstacleError('NATIVE_INITIAL_STATE_NOT_IDLE')
        if not state['pose_valid']:raise ObstacleError('NATIVE_INITIAL_POSE_NOT_VALID')
        if 'selection' in self.record and (math.dist(state['position'],self.record['selection']['position'])>0.04
                or abs(state['yaw'])>0.02):
            raise ObstacleError('NATIVE_INITIAL_POSE_MISMATCH')
        if not 0<=self.clock()-received<=0.25:
            self._diagnostic['last_error']='NATIVE_INITIAL_STATE_STALE';return None
        return state

    def _endpoints(self):
        try:
            publishers,_=self.backend.node.topic_info(self.topic)
            _,subscribers=self.backend.node.topic_info(self.control)
            types=[value(p,'msg_type_name') for p in publishers]
            controls=[value(s,'msg_type_name') for s in subscribers]
        except Exception as exc:
            self._diagnostic['last_error']='TOPIC_INFO_'+type(exc).__name__
            return False
        self._diagnostic.update(state_publishers=len(publishers),control_subscribers=len(subscribers),
                                state_types=types,control_types=controls)
        snapshot={k:copy.deepcopy(self._diagnostic[k]) for k in
                  ('state_publishers','control_subscribers','state_types','control_types')}
        self._diagnostic['observations']=(self._diagnostic['observations']+[snapshot])[-16:]
        if len(publishers)>1 or len(subscribers)>1:raise ObstacleError('MOTION_CHANNEL_DUPLICATE_ENDPOINT')
        if any(t!='gz.msgs.StringMsg' for t in types+controls):raise ObstacleError('MOTION_CHANNEL_WRONG_TYPE')
        if not publishers or not subscribers:
            missing=[]
            if not publishers:missing.append('STATE_PUBLISHER_MISSING')
            if not subscribers:missing.append('CONTROL_SUBSCRIBER_MISSING')
            self._diagnostic['last_error']='+'.join(missing)
            return False
        return True

    def _callback(self,message):
        received=self.clock()
        with self.lock:
            if self._closed:return
            try:
                if len(message.data)>8192:raise ObstacleError('NATIVE_STATE_TOO_LARGE')
                d=strict_json_loads(message.data)
                if not isinstance(d,dict):raise ObstacleError('INVALID_NATIVE_MOTION_TELEMETRY')
                if d.get('protocol')!=self.PROTOCOL:raise ObstacleError('NATIVE_PROTOCOL_MISMATCH')
                if (d.get('token')!=self.record['lease_id'] or type(d.get('entity_id')) is not int
                        or d['entity_id']!=self.record['entity_id']
                        or d.get('entity_name')!=self.record['entity_name']):
                    raise ObstacleError('NATIVE_MOTION_IDENTITY_MISMATCH')
                numbers=[d.get('sim_time'),d.get('yaw'),d.get('commanded_vy')]
                position=d.get('position')
                if (not isinstance(position,list) or len(position)!=3
                        or any(type(v) not in (int,float) or not math.isfinite(v) for v in position+numbers)
                        or type(d.get('sequence')) is not int or d['sequence']<=0 or d['sim_time']<=0
                        or any(type(d.get(k)) is not bool for k in ('stopped','configured','pose_valid','contact_measured'))
                        or type(d.get('robot_contacts')) is not int or d['robot_contacts']<0
                        or d.get('configure_status') not in ('READY','FAILED')
                        or not isinstance(d.get('reason'),str) or not d['reason'].isascii()
                        or not 1<=len(d['reason'])<=80 or any(c not in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ_0123456789' for c in d['reason'])
                        or d.get('frame')!='gazebo_world'):
                    raise ObstacleError('INVALID_NATIVE_MOTION_TELEMETRY')
                if self.latest and (d['sequence']<=self.latest[0]['sequence'] or d['sim_time']<=self.latest[0]['sim_time']):
                    raise ObstacleError('MOTION_TELEMETRY_CLOCK_OR_SEQUENCE_RESET')
                for key in ('heartbeat_age_wall_s','heartbeat_receive_gap_wall_s','heartbeat_receive_max_gap_wall_s',
                            'preupdate_gap_wall_s','preupdate_max_gap_wall_s','heartbeat_authorization_age_wall_s'):
                    if key in d and d[key] is not None and (type(d[key]) not in (int,float) or not math.isfinite(d[key]) or d[key]<0):
                        raise ObstacleError('INVALID_NATIVE_HEARTBEAT_DIAGNOSTIC')
                for key in ('heartbeat_received_count','heartbeat_authorized_count','heartbeat_rejected_count'):
                    if key in d and (type(d[key]) is not int or d[key]<0):raise ObstacleError('INVALID_NATIVE_HEARTBEAT_DIAGNOSTIC')
                self.latest=(d,received)
            except Exception as exc:
                self.error=str(exc) if isinstance(exc,ObstacleError) else type(exc).__name__

    def send(self, command, *, proof=None):
        if command not in ('heartbeat', 'stop'):
            raise ObstacleError('INVALID_MOTION_COMMAND')
        if self._closed or not self._ready:raise ObstacleError('MOTION_CHANNEL_NOT_READY')
        msg = self.cls(); msg.data = command
        if command=='heartbeat':
            # The controller supplies the exact native observation checked in
            # its health pass; a later callback cannot refresh a stalled pass.
            self.state() # preserve latched decode/identity errors
            observation=proof if proof is not None else self.latest
            if observation is None or not 0<=self.clock()-observation[1]<=0.25:
                raise ObstacleError('HEARTBEAT_NATIVE_PROOF_STALE')
            self._heartbeat_sequence+=1
            msg.data=f'heartbeat:{self._heartbeat_sequence}:{observation[0]["sequence"]}'
        if not self.publisher.publish(msg):
            raise ObstacleError('MOTION_COMMAND_NOT_PUBLISHED')

    def state(self):
        with self.lock:
            if self.error:
                raise ObstacleError(self.error)
            return self.latest

    def close(self):
        with self.lock:
            if self._closed:return
            self._closed=True;self._ready=False
        if self._subscribed:self.backend.node.unsubscribe(self.topic)
        self.publisher=None
