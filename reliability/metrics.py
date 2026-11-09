"""Offline, deterministic metrics. Absence of contacts is never zero collisions."""
import hashlib
import json
import math
from collections import Counter
from datetime import datetime

from .safety import ObstacleError


def local_point(point, box):
    dx, dy = point[0] - box['position'][0], point[1] - box['position'][1]
    c, s = math.cos(box.get('yaw', 0)), math.sin(box.get('yaw', 0))
    return [c * dx + s * dy, -s * dx + c * dy]


def clearance(robot, radius, box):
    """Signed circle/OBB separation (robot uses configured conservative disk)."""
    p = local_point(robot, box)
    q = [abs(p[i]) - box['size'][i] / 2 for i in range(2)]
    return math.hypot(max(q[0], 0), max(q[1], 0)) + min(max(q), 0) - radius


def ttc(robot, radius, box, robot_velocity, obstacle_velocity):
    """Exact constant-velocity disk/box sweep for fixed box yaw; no extrapolated turns."""
    if robot_velocity is None or obstacle_velocity is None:
        return None
    p = local_point(robot, box)
    vx, vy = [robot_velocity[i] - obstacle_velocity[i] for i in range(2)]
    c, s = math.cos(box.get('yaw', 0)), math.sin(box.get('yaw', 0))
    v = [c * vx + s * vy, -s * vx + c * vy]
    if clearance(robot, radius, box) <= 0:
        return 0.0
    hx, hy = [v / 2 for v in box['size'][:2]]
    nearest = [max(abs(p[0]) - hx, 0) * (1 if p[0] >= 0 else -1),
               max(abs(p[1]) - hy, 0) * (1 if p[1] >= 0 else -1)]
    if math.hypot(*v) < 1e-6 or sum(a * b for a, b in zip(nearest, v)) >= -1e-9:
        return None
    candidates = []
    for axis, half, other_half in [(0, hx, hy), (1, hy, hx)]:
        if abs(v[axis]) < 1e-12:
            continue
        for sign in (-1, 1):
            t = (sign * (half + radius) - p[axis]) / v[axis]
            if t >= 0 and abs(p[1-axis] + t*v[1-axis]) <= other_half:
                candidates.append(t)
    a = sum(x*x for x in v)
    for x in (-hx, hx):
        for y in (-hy, hy):
            dx, dy = p[0]-x, p[1]-y
            b = 2*(dx*v[0]+dy*v[1]); d = b*b-4*a*(dx*dx+dy*dy-radius*radius)
            if d >= 0:
                t = (-b-math.sqrt(d))/(2*a)
                if t >= 0:
                    candidates.append(t)
    return min(candidates) if candidates else None


def finite_vector(value, length):
    return (isinstance(value, list) and len(value) == length
            and all(type(v) in (float, int) and math.isfinite(v) for v in value))


class Metrics:
    def __init__(self, identity, radius=0.22):
        self.identity = identity; self.radius = radius
        self.samples = []; self.paths = []; self.bt_events = []; self.issues = []
        self.pose_stale_dropped = 0; self.pose_max_drain_age_s = 0.0

    def sample(self, sample):
        if (sample.get('frame') != 'gazebo_world' or not finite_vector(sample.get('robot'), 3)
                or type(sample.get('sim_time')) not in (int, float) or not math.isfinite(sample['sim_time'])
                or sample['sim_time'] <= 0 or type(sample.get('received_mono')) not in (int, float)
                or not math.isfinite(sample['received_mono'])):
            raise ObstacleError('INVALID_EVALUATION_SAMPLE')
        box = sample.get('obstacle')
        if box is not None and (not finite_vector(box.get('position'), 3) or not finite_vector(box.get('size'), 3)
                or any(v <= 0 for v in box['size']) or not finite_vector([box.get('yaw')], 1)):
            raise ObstacleError('INVALID_EVALUATION_BOX')
        sample = dict(sample); sample.update(robot_velocity_world=None, obstacle_velocity_world=None,
                                             geometric_clearance_m=None, ttc_s=None, segment_valid=False)
        if box:
            sample['geometric_clearance_m'] = clearance(sample['robot'], self.radius, box)
        if self.samples:
            prev = self.samples[-1]; dt = sample['sim_time'] - prev['sim_time']
            wall = sample['received_mono'] - prev['received_mono']
            if dt <= 0 or wall <= 0:
                raise ObstacleError('EVALUATION_CLOCK_RESET_OR_DUPLICATE')
            dr = math.dist(sample['robot'][:2], prev['robot'][:2])
            oldbox = prev.get('obstacle')
            do = math.dist(box['position'][:2], oldbox['position'][:2]) if box and oldbox else 0
            valid = dt <= 0.25 and wall <= 0.5 and dr <= 0.8*dt+0.05 and do <= 0.3*dt+0.03
            if not valid:
                self.issues.append({'sample': len(self.samples), 'reason': 'GAP_OR_POSITION_JUMP', 'sim_dt': dt, 'wall_dt': wall})
            else:
                sample['segment_valid'] = True
                sample['robot_velocity_world'] = [(sample['robot'][i] - prev['robot'][i])/dt for i in range(2)]
                if box and oldbox and box['size'] == oldbox['size'] and abs(box['yaw']-oldbox['yaw']) < 0.01:
                    sample['obstacle_velocity_world'] = [(box['position'][i]-oldbox['position'][i])/dt for i in range(2)]
                    sample['ttc_s'] = ttc(sample['robot'], self.radius, box,
                                           sample['robot_velocity_world'], sample['obstacle_velocity_world'])
        self.samples.append(sample)

    def path(self, points, sim_stamp, sim_now, task_id):
        if (not 2 <= len(points) <= 10000 or any(not finite_vector(p, 2) for p in points)
                or not math.isfinite(sim_stamp) or sim_stamp <= 0 or not -0.25 <= sim_now-sim_stamp <= 3):
            raise ObstacleError('INVALID_EVALUATION_PATH_TIME_OR_POINTS')
        digest = hashlib.sha256(json.dumps(points, separators=(',', ':')).encode()).hexdigest()
        self.paths.append({'sim_stamp': sim_stamp, 'task_id': task_id, 'sha256': digest,
                           'length_m': sum(math.dist(a, b) for a, b in zip(points, points[1:])), 'points': points})

    def report(self, execution=None, lease=None, error=None):
        intervals = list(zip(self.samples, self.samples[1:]))
        valid = [(a, b) for a, b in intervals if b['segment_valid']]
        gaps = [b['sim_time']-a['sim_time'] for a, b in intervals]
        distances = [s['geometric_clearance_m'] for s in self.samples if s['geometric_clearance_m'] is not None]
        ttcs = [s['ttc_s'] for s in self.samples if s['ttc_s'] is not None]
        measured = [s for s in self.samples if s.get('contact_evidence', {}).get('status') == 'MEASURED']
        contact_samples = sum(s['contact_evidence']['robot_contacts'] > 0 for s in measured)
        decisions = Counter(a.get('recovery_action') for t in (execution or {}).get('tasks', [])
                            for a in t.get('attempts', []) if a.get('recovery_action'))
        changes = sum(a['task_id'] == b['task_id'] and a['sha256'] != b['sha256'] for a, b in zip(self.paths, self.paths[1:]))
        recoveries = Counter(e['node_name'] for e in self.bt_events if e['current_status'] == 'RUNNING'
                             and e['previous_status'] != 'RUNNING' and e['node_name'] in
                             ('Spin', 'BackUp', 'Wait', 'ClearEntireCostmap', 'ClearLocalCostmap', 'ClearGlobalCostmap'))
        wall_duration = None
        try:
            start = datetime.fromisoformat(execution['created_at']); end = datetime.fromisoformat(execution['ended_at'])
            if start.tzinfo is not None and end.tzinfo is not None and end >= start:
                wall_duration = (end-start).total_seconds()
        except (KeyError, TypeError, ValueError):
            pass
        return {'evaluation_version': '1.0', **self.identity, 'error': error,
                'coordinate_basis': 'Gazebo world; map/world identity checked against immutable static scene',
                'robot_geometry': {'type': 'configured_conservative_disk', 'radius_m': self.radius},
                'time_basis': {'motion_and_geometry': 'simulation', 'freshness_and_watchdog': 'monotonic_wall',
                               'bt_event_internal_timestamp': 'wall_or_unspecified; never mixed into simulation deltas'},
                'quality': {'status': 'INCOMPLETE' if error or self.issues or self.pose_stale_dropped or len(self.samples) < 2 else 'VALID_OBSERVATION_WINDOW',
                            'sample_count': len(self.samples), 'pose_stale_messages_dropped': self.pose_stale_dropped,
                             'max_pose_drain_age_s': self.pose_max_drain_age_s or None,
                             'max_sim_interval_s': max(gaps) if gaps else None,
                            'issues': self.issues, 'full_navigation_coverage': 'NOT_ASSERTED'},
                'metrics': {'observation_sim_duration_s': self.samples[-1]['sim_time']-self.samples[0]['sim_time'] if self.samples else None,
                            'robot_observed_path_length_m': sum(math.dist(a['robot'][:2], b['robot'][:2]) for a, b in valid),
                            'valid_interval_duration_s': sum(b['sim_time']-a['sim_time'] for a, b in valid),
                            'minimum_geometric_clearance_m': min(distances) if distances else None,
                            'minimum_applicable_ttc_s': min(ttcs) if ttcs else None,
                            'ttc_model': 'constant relative translation, fixed box yaw; null means NOT_APPLICABLE',
                            'path_publications': len(self.paths), 'observed_same_task_path_changes': changes,
                            'planner_invocation_count': None, 'replanning_measurement': 'PATH_CHANGE_PROXY_ONLY',
                            'sensor_latency': {'status': 'NOT_MEASURED', 'seconds': None}},
                'contacts': {'scope': 'injected_obstacle_vs_robot_only', 'status': 'PARTIAL_EVIDENCE' if measured else 'NOT_MEASURED',
                             'measured_samples': len(measured), 'positive_contact_samples': contact_samples if measured else None,
                             'overall_zero_collision_claim': 'UNKNOWN'},
                'nav2': {'bt_status_events_observed': len(self.bt_events), 'recovery_running_transitions': dict(recoveries),
                         'observed_planning_action_running_transitions': sum(e['current_status'] == 'RUNNING'
                             and e['previous_status'] != 'RUNNING' and e['node_name'] in ('ComputePathToPose', 'ComputePathThroughPoses')
                             for e in self.bt_events),
                         'status': 'OBSERVED_EVENTS_ONLY' if self.bt_events else 'NOT_MEASURED'},
                'v4': {'status': (execution or {}).get('status'), 'task_level_decisions': dict(decisions),
                       'reported_run_wall_duration_s': wall_duration,
                       'tasks': (execution or {}).get('tasks', []), 'created_at': (execution or {}).get('created_at'),
                       'ended_at': (execution or {}).get('ended_at'), 'duration_basis': 'V4 wall-clock report; separate from sim window'},
                'lease': lease, 'samples': self.samples, 'paths': self.paths, 'bt_events': self.bt_events}
