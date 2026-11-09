"""Read-only path observation; early spinning is opt-in for moving experiments."""
import copy
import math
import threading
import time
from collections import Counter, deque

from .safety import ObstacleError, active_task, bound_report
from scenes.runtime_guard import check_map_message


class NavigationObserver:
    def __init__(self,bundle):
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
        from nav_msgs.msg import Path,OccupancyGrid
        self.rclpy=rclpy;self.owned_context=not rclpy.ok()
        if self.owned_context:rclpy.init(args=[])
        self.node=Node('ea_obstacle_observer',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.bundle=bundle;self.map=None;self.path=None
        self._init_observation()
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(OccupancyGrid,'/map',self._receive_map,qos)
        path_qos=QoSProfile(depth=64,durability=DurabilityPolicy.VOLATILE,reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(Path,'/plan',self._receive_path,path_qos)

    def _init_observation(self):
        self._condition=threading.Condition();self._candidates=deque(maxlen=64)
        self._stats=Counter();self._history=deque(maxlen=8);self._selected=None
        self._context=None;self._thread=None;self._worker_error=None;self._stop=threading.Event()
        self._preparation={'status':'NOT_REQUESTED'};self._started=time.monotonic()

    def _receive_map(self,message):
        with self._condition:
            self.map=message;self._stats['maps_received']+=1;self._condition.notify_all()

    def _receive_path(self,message):
        received=time.monotonic()
        with self._condition:
            # Preserve the latest-message view used by D1 EvaluationObserver.poll.
            self.path=(message,received);self._stats['paths_received']+=1
            if len(message.poses)>10000:
                self._stats['oversized_messages_discarded']+=1
            else:
                if len(self._candidates)==self._candidates.maxlen:self._stats['buffer_evictions']+=1
                self._candidates.append((message,received,self._stats['paths_received']))
            self._condition.notify_all()

    def bind_context(self,context):
        self._context=context

    def start_preparation(self,executor_factory=None):
        """D2/D3 only: spin through blocking Gazebo RPCs; join before foreground poll."""
        if self._thread is not None:raise ObstacleError('PATH_OBSERVER_ALREADY_SPINNING')
        if executor_factory is None:
            from rclpy.executors import SingleThreadedExecutor
            executor_factory=SingleThreadedExecutor
        executor=executor_factory(context=self.node.context);executor.add_node(self.node)
        self._executor=executor
        def spin():
            try:
                while not self._stop.is_set() and self.rclpy.ok():executor.spin_once(timeout_sec=0.05)
            except BaseException as exc:
                with self._condition:
                    self._worker_error=type(exc).__name__;self._condition.notify_all()
        self._thread=threading.Thread(target=spin,name='ea_path_preparation',daemon=True)
        try:self._thread.start()
        except BaseException:
            executor.remove_node(self.node);executor.shutdown(timeout_sec=1.0);self._thread=None
            raise

    def stop_preparation(self):
        if self._thread is None:return
        self._stop.set();self._thread.join(timeout=2.0)
        if self._thread.is_alive():raise ObstacleError('PATH_OBSERVER_STOP_NOT_CONFIRMED')
        self._executor.remove_node(self.node);self._executor.shutdown(timeout_sec=1.0)
        self._thread=None

    def _pump(self,timeout=0.05):
        if self._worker_error:raise ObstacleError('PATH_OBSERVER_WORKER_FAILED')
        if not self.rclpy.ok():raise ObstacleError('PATH_OBSERVER_CONTEXT_STOPPED')
        if self._thread is not None:
            with self._condition:self._condition.wait(timeout=timeout)
        else:self.rclpy.spin_once(self.node,timeout_sec=timeout)

    def _guard(self,target=None):
        if self._context is None:return
        report=bound_report(self._context)
        if report.get('status')!='RUNNING':
            self._preparation.update(status='TASK_ENDED_EARLY',navigation_status=report.get('status'))
            raise ObstacleError('NAVIGATION_ENDED_BEFORE_INJECTION')
        if target is not None:
            matches=[t for t in report.get('tasks',[]) if type(t.get('id')) is int and t['id']==target['id']]
            if len(matches)!=1 or matches[0].get('status')!='RUNNING':
                self._preparation.update(status='TASK_ENDED_EARLY')
                raise ObstacleError('TARGET_TASK_ENDED_BEFORE_INJECTION')
            task=active_task(self._context)
            if task['id']!=target['id'] or task['x']!=target['x'] or task['y']!=target['y']:
                self._preparation.update(status='TASK_ENDED_EARLY')
                raise ObstacleError('TARGET_TASK_ENDED_BEFORE_INJECTION')
        return report

    def wait_for_task(self,step,timeout,checkpoint=None):
        if self._context is None:raise ObstacleError('PATH_CONTEXT_BINDING_REQUIRED')
        if type(step) is not int or not 1<=step<=len(self._context['plan']['steps']):
            raise ObstacleError('INVALID_TARGET_STEP')
        if not math.isfinite(timeout) or not 0<timeout<=300:raise ObstacleError('INVALID_TASK_WAIT_TIMEOUT')
        start=time.monotonic();deadline=start+timeout;last_checkpoint=start
        self._preparation={'status':'WAITING_FOR_TASK','target_step':step,'wait_limit_wall_s':timeout}
        try:
            while self.rclpy.ok():
                report=self._guard()
                tasks=[t for t in report.get('tasks',[]) if type(t.get('id')) is int and t['id']==step]
                if len(tasks)!=1:raise ObstacleError('WAIT_TARGET_TASK_MISSING_OR_AMBIGUOUS')
                target=tasks[0]
                if target.get('status')=='RUNNING':
                    task=active_task(self._context)
                    if task['id']!=step:raise ObstacleError('ACTIVE_TASK_AMBIGUOUS')
                    self._preparation.update(status='TASK_RUNNING',waited_wall_s=time.monotonic()-start)
                    return task
                if target.get('status')!='PENDING':
                    self._preparation.update(status='TASK_ENDED_EARLY',task_status=target.get('status'))
                    raise ObstacleError('TARGET_TASK_ENDED_BEFORE_INJECTION')
                if time.monotonic()>=deadline:
                    self._preparation['status']='TASK_WAIT_TIMED_OUT'
                    raise ObstacleError('TARGET_TASK_WAIT_TIMED_OUT')
                if checkpoint and time.monotonic()-last_checkpoint>=1:
                    checkpoint();last_checkpoint=time.monotonic()
                self._pump(min(0.05,max(0,deadline-time.monotonic())))
            raise ObstacleError('PATH_OBSERVER_CONTEXT_STOPPED')
        finally:self._preparation['waited_wall_s']=time.monotonic()-start

    def _check_candidate(self,candidate,target,sim_now):
        message,received,sequence=candidate
        age=time.monotonic()-received
        if not 0<=age<=2.0:return None,'PATH_WALL_EXPIRED'
        stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
        if not math.isfinite(stamp) or not math.isfinite(sim_now) or stamp<=0 or not -0.25<=sim_now-stamp<=3.0:
            return None,'PATH_SIM_TIME_MISMATCH'
        if message.header.frame_id!='map':return None,'PATH_FRAME_MISMATCH'
        if not 2<=len(message.poses)<=10000:return None,'INVALID_PATH_SIZE'
        points=[[p.pose.position.x,p.pose.position.y] for p in message.poses]
        if any(not math.isfinite(v) for p in points for v in p):return None,'NONFINITE_PATH'
        if math.hypot(points[-1][0]-target['x'],points[-1][1]-target['y'])>0.6:
            return None,'PATH_TARGET_MISMATCH'
        return points,None

    def sample(self,target):
        started=time.monotonic();deadline=started+5;checks=Counter();result='FAILED';error=None
        summary={'target_step':target.get('id'),'target':{k:target[k] for k in ('x','y')}}
        try:
            while self.rclpy.ok():
                self._guard(target)
                if self._worker_error:raise ObstacleError('PATH_OBSERVER_WORKER_FAILED')
                with self._condition:
                    candidates=list(self._candidates);map_message=self.map
                if map_message is None:checks['MAP_NOT_RECEIVED']+=1
                else:
                    try:check_map_message(map_message,self.bundle)
                    except ValueError:
                        checks['MAP_ASSET_MISMATCH']+=1;raise
                if not candidates:
                    checks['PATH_NOT_RECEIVED' if not self._stats['paths_received'] else 'PATH_NO_RETAINED_CANDIDATES']+=1
                if map_message is not None:
                    sim_now=self.node.get_clock().now().nanoseconds/1e9
                    # Search retained candidates, not only the latest publisher sample.
                    for candidate in reversed(candidates):
                        points,reason=self._check_candidate(candidate,target,sim_now)
                        if reason:checks[reason]+=1;continue
                        self._guard(target)
                        self._selected=(candidate,dict(target));result='SELECTED'
                        summary.update(selected_sequence=candidate[2],endpoint=points[-1],
                                       selected_wall_age_s=time.monotonic()-candidate[1],
                                       selected_sim_age_s=sim_now-(candidate[0].header.stamp.sec+candidate[0].header.stamp.nanosec/1e9))
                        return points
                if time.monotonic()>=deadline:break
                self._pump(min(0.05,max(0,deadline-time.monotonic())))
            raise ObstacleError('FRESH_MATCHING_NAV2_PATH_NOT_OBSERVED')
        except BaseException as exc:
            error=str(exc) if isinstance(exc,ObstacleError) else type(exc).__name__
            if 'ENDED_BEFORE_INJECTION' in error:checks['TASK_ENDED_EARLY']+=1
            raise
        finally:
            summary.update(outcome=result,error=error,elapsed_wall_s=time.monotonic()-started,
                           rejection_checks=dict(checks),map_received=self.map is not None)
            with self._condition:self._history.append(summary)

    def assert_selected_fresh(self,target):
        """Recheck the exact geometry-approved candidate immediately before create."""
        self._guard(target)
        if self._worker_error:raise ObstacleError('PATH_OBSERVER_WORKER_FAILED')
        if self._selected is None:raise ObstacleError('NO_SELECTED_PATH')
        candidate,selected_target=self._selected
        if selected_target!=target:raise ObstacleError('SELECTED_PATH_TARGET_CHANGED')
        check_map_message(self.map,self.bundle)
        _,reason=self._check_candidate(candidate,target,self.node.get_clock().now().nanoseconds/1e9)
        if reason:
            with self._condition:self._history.append({'outcome':'PRE_CREATE_REJECTED','error':reason,'rejection_checks':{reason:1}})
            raise ObstacleError(reason)

    def diagnostics(self):
        with self._condition:
            result=copy.deepcopy({'diagnostics_version':'1.0','statistics':dict(self._stats),
                'map_received':self.map is not None,'candidate_count':len(self._candidates),
                'preparation':self._preparation,'sample_history':list(self._history),
                'worker_error':self._worker_error,'candidate_limit':64,
                'observer_uptime_wall_s':time.monotonic()-self._started,
                'freshness_limits':{'wall_age_s':2.0,'sim_age_s':[-0.25,3.0],'target_tolerance_m':0.6}})
            recent=list(self._candidates)[-4:]
        result['recent_candidates']=[]
        for message,received,sequence in recent:
            point=message.poses[-1].pose.position if message.poses else None
            endpoint=[point.x,point.y] if point is not None and math.isfinite(point.x) and math.isfinite(point.y) else None
            stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
            result['recent_candidates'].append({'sequence':sequence,'endpoint':endpoint,
                'sim_stamp':stamp if math.isfinite(stamp) else None,'wall_age_s':max(0,time.monotonic()-received)})
        if self._context is not None:
            try:
                report=bound_report(self._context)
                result['bound_run_state']={'navigation_status':report.get('status'),
                    'tasks':[{'id':t.get('id'),'status':t.get('status')} for t in report.get('tasks',[])]}
            except (ValueError,OSError,KeyError,TypeError) as exc:
                result['binding_error']=str(exc) if isinstance(exc,ObstacleError) else type(exc).__name__
        return result

    def close(self):
        self.stop_preparation()
        self.node.destroy_node()
        if self.owned_context and self.rclpy.ok():self.rclpy.shutdown()
