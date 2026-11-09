"""Reusable quantitative observation window. ROS subscriptions are read-only."""
import time
import math
from pathlib import Path

from planner.validation import strict_json_loads
from planner.plan_identity import compute_plan_id, ASSET_PLAN_ID_SCHEME
from planner.provenance import validate_source_record
from .safety import ObstacleError, active_task
from .streams import PoseStream
from .metrics import Metrics


def identity(context):
    return {k: context[k] for k in ('plan_id', 'run_id', 'provenance_id')} | {
        'scene_id': 'inspection_world', 'scene_assets': context['plan']['scene_assets']}


def read_execution(context):
    report = strict_json_loads(Path(context['report_path']).read_text())
    if (report.get('run_id') != context['run_id'] or report.get('plan_id') != context['plan_id']
            or compute_plan_id(report.get('plan')) != context['plan_id']):
        raise ObstacleError('EVALUATION_RUN_OR_PLAN_CHANGED')
    source = validate_source_record(report.get('source'), context['plan_id'], plan_id_scheme=ASSET_PLAN_ID_SCHEME)
    if source['provenance_id'] != context['provenance_id']:
        raise ObstacleError('EVALUATION_PROVENANCE_CHANGED')
    return report


def select_fresh_pose_batch(batch, last_stamp, now, max_age=0.25):
    """Drop old *received* frames, never hide timestamp resets or a stale latest frame.

    Returns fresh frames, stale-frame count, last source stamp, maximum drain age.
    Missing frames remain visible through quality counters and the existing gap checks.
    """
    if not batch:
        return [], 0, last_stamp, None
    previous = last_stamp
    for stamp, _poses, _received in batch:
        if previous is not None and stamp <= previous:
            raise ObstacleError('EVALUATION_SIM_CLOCK_RESET')
        previous = stamp
    maximum_age = max(max(0.0, now - received) for _stamp, _poses, received in batch)
    if now - batch[-1][2] > max_age:
        raise ObstacleError('EVALUATION_POSE_STREAM_STALE')
    fresh = [row for row in batch if now - row[2] <= max_age]
    return fresh, len(batch) - len(fresh), previous, maximum_age


class Capture:
    def __init__(self, context, backend, observer, record=None, stream_factory=PoseStream):
        self.context = context; self.backend = backend; self.observer = observer; self.record = record
        self.metrics = Metrics(identity(context), context['bundle']['source']['robot_radius_m'])
        entities = backend.entities(); self.robot_id = entities[backend.robot_name]['id']
        self.stream = stream_factory(backend); self.started_wall = time.monotonic()
        self.last_received = self.started_wall; self.last_sim = None; self.last_source_sim = None; self.execution = context['report']
        self.odom = None

    def tick(self, motion_state=None):
        self.execution = read_execution(self.context)
        if self.execution['status'] != 'RUNNING':
            return False  # terminal run: do not turn successful execution into stale-data error
        running = [t for t in self.execution.get('tasks', []) if t.get('status') == 'RUNNING']
        task = active_task(self.context) if len(running) == 1 else None
        self.odom = self.observer.poll(self.metrics, task)
        frames, dropped, self.last_source_sim, maximum_age = select_fresh_pose_batch(
            self.stream.drain(), self.last_source_sim, time.monotonic())
        self.metrics.pose_stale_dropped += dropped
        if maximum_age is not None:
            self.metrics.pose_max_drain_age_s = max(self.metrics.pose_max_drain_age_s, maximum_age)
        for stamp, poses, received in frames:
            if self.observer.sim_now is None or abs(self.observer.sim_now-stamp) > 0.5:
                raise ObstacleError('ROS_GAZEBO_SIM_CLOCK_MISMATCH')
            # Deliberate 20Hz downsample; source ordering/reset still checked below.
            self.last_received = received
            if self.last_sim is not None and stamp-self.last_sim < 0.049:
                continue
            self.last_sim = stamp
            robot = poses.get(self.robot_id)
            if robot is None:
                raise ObstacleError('EVALUATION_ROBOT_POSE_MISSING')
            box = None
            if self.record:
                entity = poses.get(self.record['entity_id'])
                if entity is None:
                    raise ObstacleError('EVALUATION_OBSTACLE_POSE_MISSING')
                box = {**entity, 'size': self.record['selection']['size']}
                if motion_state and abs(motion_state['sim_time']-stamp) <= 0.15:
                    allowance = 0.04 + 0.3*abs(motion_state['sim_time']-stamp)
                    if math.dist(entity['position'], motion_state['position']) > allowance:
                        raise ObstacleError('NATIVE_AND_GAZEBO_POSE_DISAGREE')
            evidence = {'status': 'NOT_MEASURED', 'robot_contacts': None}
            if motion_state and abs(motion_state['sim_time']-stamp) <= 0.1 and motion_state['contact_measured']:
                evidence = {'status': 'MEASURED', 'robot_contacts': motion_state['robot_contacts'],
                            'stamp_sim': motion_state['sim_time'], 'source': 'native_ContactSensorData'}
            self.metrics.sample({'sim_time': stamp, 'received_mono': received, 'frame': 'gazebo_world',
                'robot': [*robot['position'][:2], robot['yaw']], 'obstacle': box, 'odometry': self.odom,
                'task_id': task['id'] if task else None, 'task_status': task['status'] if task else None,
                'navigation_status': self.execution['status'],
                'goal': {k: task[k] for k in ('id', 'x', 'y')} if task else None,
                'contact_evidence': evidence})
            if box and self.metrics.samples[-1]['geometric_clearance_m'] < 0:
                raise ObstacleError('POSSIBLE_GEOMETRIC_OVERLAP')
        if time.monotonic()-self.last_received > 0.5:
            raise ObstacleError('EVALUATION_POSE_STREAM_TIMEOUT_OR_PAUSE')
        if self.metrics.issues:
            raise ObstacleError('EVALUATION_DATA_GAP_OR_POSITION_JUMP')
        return self.execution['status'] == 'RUNNING'

    def report(self, error=None):
        reference = None if not self.record else {k: self.record.get(k) for k in
            ('lease_id', 'entity_id', 'entity_name', 'status', 'cleanup', 'world_session_id', 'motion')}
        return self.metrics.report(self.execution, reference, error)

    def close(self):
        self.stream.close()
