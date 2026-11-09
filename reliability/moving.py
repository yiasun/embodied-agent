"""Stage 6.2 runtime motion, opt-in only; reuses the Stage 6.1 lease lifecycle."""
import argparse
import hashlib
import json
import math
import os
import signal
import sys
import time
from pathlib import Path

from day4.reporting import atomic_write_json
from .obstacles import ROOT, exclusive_test_lock, new_record
from .safety import ObstacleError, load_context, bound_report
from .lifecycle import run_lease
from .trajectory import profile
from .capture import Capture
from .streams import MotionChannel
from .scheduling import Timings, RuntimeIO


def reserve(path):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600); os.close(fd)


def controller(context, backend, observer, record, path, writer, *, channel_factory=MotionChannel,
               capture_factory=Capture, sleep=time.sleep, checkpoint=None,io_factory=RuntimeIO):
    capture = None; channel = None; error = None; previous_sequence = None
    io=None;probe=None;timings=Timings(lambda:time.monotonic());last_heartbeat=None;first_heartbeat=None
    record.update(motion_stop='NOT_STARTED', motion_trace=[])
    try:
        channel = channel_factory(backend, record)
        probe=timings.call('discovery_setup',backend.session_probe) if hasattr(backend,'session_probe') else backend.discover
        verified_at=time.monotonic()
        initial_probe=probe.startup_check if hasattr(probe,'startup_check') else backend.discover
        if timings.call('gazebo_discovery',initial_probe)['session_id']!=record['world_session_id']:
            raise ObstacleError('WORLD_SESSION_CHANGED_DURING_HANDSHAKE')
        # Pose capture starts after readiness; discovery backlog is not motion data.
        timings.call('json_writer',writer,path,record) # initial WAL, before first heartbeat
        capture = capture_factory(context, backend, observer, record)
        io=io_factory(probe,record['world_session_id'],verified_at,timings,lambda:time.monotonic())
        start = time.monotonic(); last_audit = start
        while time.monotonic()-start < record['motion']['wall_limit_s']:
            observation = channel.state()
            state = observation[0] if observation else None
            if observation and time.monotonic()-observation[1] > 0.25:
                raise ObstacleError('NATIVE_MOTION_STATE_STALE')
            tick_start=time.monotonic()
            running = timings.call('capture_tick',capture.tick,state)
            if time.monotonic()-tick_start>0.2:raise ObstacleError('CAPTURE_TICK_TIMED_OUT')
            # A blocking tick cannot authorize a heartbeat from an old state.
            observation=channel.state();state=observation[0] if observation else None
            if observation and time.monotonic()-observation[1]>0.25:raise ObstacleError('NATIVE_MOTION_STATE_STALE')
            if state and state['sequence'] != previous_sequence:
                record['motion_trace'].append({**state,'python_received_mono':observation[1]}); previous_sequence = state['sequence']
            if state and state['stopped']:
                if state['reason'] not in ('DURATION_COMPLETE', 'TRAJECTORY_COMPLETE'):
                    raise ObstacleError('NATIVE_STOP_' + state['reason'])
                record['release_reason'] = state['reason']; break
            if not running:
                record['release_reason'] = 'NAVIGATION_ENDED'; break
            tasks = [t for t in capture.execution.get('tasks', []) if t.get('status') == 'RUNNING']
            if tasks and tasks[0]['id'] != record['active_task_id']:
                record['release_reason'] = 'NAVIGATION_TASK_CHANGED'; break
            timings.call('io_poll',io.poll) # nonblocking results/deadlines only; no background heartbeat
            # Foreground safety supervision remains the only source of keepalive.
            if state and capture.metrics.samples:
                sample=capture.metrics.samples[-1]
                if not 0<=time.monotonic()-sample['received_mono']<=0.25:
                    raise ObstacleError('HEARTBEAT_POSE_PROOF_STALE')
                if sample.get('task_id')!=record['active_task_id'] or sample.get('task_status')!='RUNNING':
                    raise ObstacleError('HEARTBEAT_TASK_PROOF_NOT_ACTIVE')
                if sample.get('geometric_clearance_m') is None or sample['geometric_clearance_m']<0.35:
                    raise ObstacleError('HEARTBEAT_CLEARANCE_GUARD')
                now=time.monotonic()
                if (first_heartbeat is not None and now-first_heartbeat>0.5
                        and state.get('heartbeat_authorized_count',state.get('heartbeat_received_count'))==0):
                    raise ObstacleError('NATIVE_HEARTBEAT_ACK_TIMEOUT')
                if last_heartbeat is not None:
                    gap=now-last_heartbeat;timings.observe('heartbeat_send_interval',gap)
                    if gap>=0.5:raise ObstacleError('HEARTBEAT_SCHEDULE_MISSED')
                timings.call('heartbeat_send',channel.send,'heartbeat',proof=observation);last_heartbeat=now
                if first_heartbeat is None:first_heartbeat=now
                if time.monotonic()-now>0.1:raise ObstacleError('HEARTBEAT_SEND_TIMED_OUT')
            elif time.monotonic()-start > 0.5:
                raise ObstacleError('MOTION_START_READBACK_NOT_OBSERVED')
            if time.monotonic()-last_audit > 1:
                snapshot_start=time.monotonic()
                report=timings.call('report_prepare',capture.report)
                report['controller_diagnostics']=timings.snapshot()
                io.queue_progress(path,record,report,writer,checkpoint)
                duration=time.monotonic()-snapshot_start;timings.observe('progress_snapshot',duration)
                if duration>0.2:raise ObstacleError('PROGRESS_SNAPSHOT_TIMED_OUT')
                last_audit = time.monotonic()
            sleep(0.03)
        else:
            raise ObstacleError('MOTION_WALL_DEADLINE')
        observed = [s for s in capture.metrics.samples if s.get('obstacle')]
        distance = sum(math.dist(a['obstacle']['position'][:2], b['obstacle']['position'][:2])
                       for a, b in zip(observed, observed[1:]) if b.get('segment_valid'))
        record['observed_obstacle_travel_m'] = distance
        if distance < 0.05:
            raise ObstacleError('CONTINUOUS_MOTION_NOT_OBSERVED')
        record['motion_observation'] = 'BOUNDED_SAMPLED_VELOCITY_MOTION; physical continuity still requires live acceptance'
    except BaseException as exc:
        error = str(exc) if isinstance(exc, ObstacleError) else type(exc).__name__
        raise
    finally:
        if io:io.cancel.set()
        if channel:
            record['motion_stop'] = 'UNKNOWN'
            try:
                timings.call('stop_send',channel.send,'stop'); until = time.monotonic()+1.2
                while time.monotonic() < until:
                    value = channel.state()
                    if value and time.monotonic()-value[1] <= 0.25 and value[0]['stopped'] and abs(value[0]['commanded_vy']) < 1e-9:
                        record['motion_stop'] = 'LATCHED_ZERO_COMMAND'; break
                    sleep(0.03)
            except BaseException:
                record['motion_stop'] = 'UNKNOWN'
            finally:
                channel.close()
        final_io_error=None
        if io:
            final_io_error=io.close().get('final_error')
            if error is None and final_io_error:error=final_io_error
        elif probe is not None and hasattr(probe,'close'):probe.close()
        record['controller_diagnostics']={**timings.snapshot(),'io':io.diagnostics() if io else None,
            'capture_deadline_wall_s':0.2,'pose_max_age_wall_s':0.25,'native_state_max_age_wall_s':0.25,
            'heartbeat_threshold_wall_s':0.5,'heartbeat_sender':'foreground_controller_only',
            'transport_one_way_latency':{'status':'NOT_MEASURED','seconds':None}}
        if capture:
            try:
                if checkpoint:
                    report=capture.report(error);report['controller_diagnostics']=record['controller_diagnostics']
                    timings.call('checkpoint_final',checkpoint,report)
            finally:
                capture.close()
                record['controller_diagnostics'].update(timings.snapshot())
        if final_io_error and error==final_io_error:raise ObstacleError(final_io_error)
        if error is None and record['motion_stop'] == 'UNKNOWN':
            raise ObstacleError('MOTION_STOP_NOT_CONFIRMED')


def main(argv=None, *, backend_factory=None, observer_factory=None, controller_fn=controller):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', required=True, type=Path)
    parser.add_argument('--run-report', required=True, type=Path)
    parser.add_argument('--pattern', choices=['cross', 'shuttle'], default='cross')
    parser.add_argument('--speed', type=float, default=0.15)
    parser.add_argument('--duration', type=float, default=15)
    parser.add_argument('--target-step', type=int, default=3)
    parser.add_argument('--wait-task-seconds', type=float, default=120,
                        help='Subscribe early, then wait read-only for target task; 0 requires it already RUNNING')
    parser.add_argument('--plugin', type=Path, default=ROOT/'build/stage62_heartbeat/libea_bounded_motion.so')
    parser.add_argument('--journal', type=Path, required=True)
    parser.add_argument('--metrics', type=Path, required=True)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args(argv); old_term = None
    try:
        context = load_context(args.plan, args.run_report, ROOT)
        config = profile(context['bundle'], args.speed, args.duration, args.pattern)
        if not 1 <= args.target_step <= len(context['plan']['steps']):
            raise ObstacleError('INVALID_MOTION_TARGET_STEP')
        if not math.isfinite(args.wait_task_seconds) or not 0<=args.wait_task_seconds<=300:
            raise ObstacleError('INVALID_TASK_WAIT_TIMEOUT')
        if args.journal.resolve() == args.metrics.resolve():
            raise ObstacleError('DISTINCT_NEW_OUTPUT_FILES_REQUIRED')
        if not args.apply:
            print(json.dumps({'dry_run': True, 'profile': config, 'plan_id': context['plan_id'],
                              'target_step':args.target_step,'wait_task_seconds':args.wait_task_seconds,
                              'live_path_selected': False, 'files_written': False}, indent=2)); return 0
        if bound_report(context).get('status')!='RUNNING':raise ObstacleError('NAVIGATION_NOT_ACTIVE')
        plugin = args.plugin.resolve(strict=True)
        config.update(plugin_path=str(plugin), plugin_sha256=hashlib.sha256(plugin.read_bytes()).hexdigest(),
                      plugin_class='embodied::BoundedMotionHeartbeatV2')
        if backend_factory is None:
            from .gazebo import GazeboBackend
            backend_factory = GazeboBackend
        if observer_factory is None:
            from .eval_observer import EvaluationObserver
            observer_factory = EvaluationObserver
        def interrupted(signum, frame): raise KeyboardInterrupt
        old_term = signal.signal(signal.SIGTERM, interrupted)
        with exclusive_test_lock():
            reserve(args.metrics); reserve(args.journal)
            reserve(Path(str(args.journal)+'.progress.json'));reserve(Path(str(args.metrics)+'.progress.json'))
            record = new_record(context, 'motion'); record.update(motion=config, target_step=args.target_step,
                evaluation_path=str(args.metrics.resolve()))
            atomic_write_json(args.journal, {**record, 'create_sent': False, 'status': 'PREPARING', 'cleanup': 'NOT_NEEDED'})
            latest = {}
            def checkpoint(report,*,progress=False):
                if progress:
                    atomic_write_json(Path(str(args.metrics)+'.progress.json'),report);return
                latest.clear(); latest.update(report); atomic_write_json(args.metrics, latest)
            def run(*values):
                controller_fn(*values, checkpoint=checkpoint)
            code, record = run_lease(context, backend_factory(), observer_factory, record, args.journal,
                hold_seconds=args.duration, motion_controller=run,wait_task_seconds=args.wait_task_seconds)
            if not latest:
                from .metrics import Metrics
                from .capture import identity
                latest.update(Metrics(identity(context)).report(error=record.get('error', 'NO_OBSERVATION')))
            latest['lease'] = {k: record.get(k) for k in ('lease_id', 'entity_id', 'status', 'cleanup',
                'cleanup_error', 'motion_stop', 'motion', 'motion_trace', 'world_session_id',
                'observed_obstacle_travel_m', 'motion_observation', 'release_reason')}
            latest['lease']['journal'] = str(args.journal.resolve())
            if record.get('error'):
                latest['error'] = record['error']; latest['quality']['status'] = 'INCOMPLETE'
            if 'path_observation' in record:latest['path_observation']=record['path_observation']
            if 'motion_channel' in record:latest['motion_channel']=record['motion_channel']
            if 'controller_diagnostics' in record:latest['controller_diagnostics']=record['controller_diagnostics']
            atomic_write_json(args.metrics, latest)
        print(json.dumps({'status': record['status'], 'cleanup': record['cleanup'], 'error': record.get('error'),
                          'journal': str(args.journal), 'metrics': str(args.metrics)}, indent=2)); return code
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc) if isinstance(exc, ObstacleError) else type(exc).__name__
        print(json.dumps({'error': error, 'navigation_started': False}), file=sys.stderr)
        return 130 if isinstance(exc, KeyboardInterrupt) else 1
    finally:
        if old_term is not None: signal.signal(signal.SIGTERM, old_term)


if __name__ == '__main__':
    raise SystemExit(main())
