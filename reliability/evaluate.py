"""Baseline observation CLI. Default preview; --record only observes and writes logs."""
import argparse
import copy
import hashlib
import json
import math
import signal
import sys
import time
from pathlib import Path

from day4.reporting import atomic_write_json
from .obstacles import ROOT, exclusive_test_lock
from .safety import ObstacleError, load_context, active_task
from .lifecycle import verify_live_base
from .moving import reserve
from .capture import Capture, identity
from .metrics import Metrics
from planner.validation import strict_json_loads
from .capture import read_execution
from .lifecycle import utc


def finalize(context, source_path):
    original = Path(source_path).read_bytes()
    report = strict_json_loads(original.decode())
    if report.get('evaluation_version') != '1.0' or any(report.get(k) != v for k, v in identity(context).items()):
        raise ObstacleError('EVALUATION_IDENTITY_MISMATCH')
    execution = read_execution(context)
    if execution.get('status') not in ('SUCCEEDED', 'FAILED', 'TIMED_OUT', 'UNKNOWN', 'INTERRUPTED', 'CANCELED'):
        raise ObstacleError('TERMINAL_EXECUTION_REPORT_REQUIRED')
    result = copy.deepcopy(report)
    result.update(v4_at_observation_end=report['v4'], v4=Metrics(identity(context)).report(execution)['v4'],
                  finalized_at=utc(), input_evaluation_sha256=hashlib.sha256(original).hexdigest(),
                  observation_window_unchanged=True)
    return result


def main(argv=None, *, backend_factory=None, observer_factory=None, capture_factory=Capture):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', required=True, type=Path)
    parser.add_argument('--run-report', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--wall-seconds', type=float, default=180)
    parser.add_argument('--record', action='store_true')
    parser.add_argument('--from-evaluation', type=Path, help='Offline final V4 linkage; --record writes a new output only')
    args = parser.parse_args(argv); old_term = None; observer = capture = None; report = None; error = None
    try:
        context = load_context(args.plan, args.run_report, ROOT)
        if not math.isfinite(args.wall_seconds) or not 1 <= args.wall_seconds <= 300:
            raise ObstacleError('INVALID_OBSERVATION_WALL_LIMIT')
        if args.from_evaluation:
            report = finalize(context, args.from_evaluation)
            if args.record:
                if args.output.resolve() == args.from_evaluation.resolve():
                    raise ObstacleError('FINALIZE_REQUIRES_NEW_OUTPUT')
                reserve(args.output); atomic_write_json(args.output, report)
            print(json.dumps({'dry_run': not args.record, **identity(context), 'final_navigation_status': report['v4']['status']}))
            return 0
        if not args.record:
            print(json.dumps({'dry_run': True, **identity(context), 'files_written': False})); return 0
        active_task(context)
        if backend_factory is None:
            from .gazebo import GazeboBackend
            backend_factory = GazeboBackend
        if observer_factory is None:
            from .eval_observer import EvaluationObserver
            observer_factory = EvaluationObserver
        def interrupted(signum, frame): raise KeyboardInterrupt
        old_term = signal.signal(signal.SIGTERM, interrupted)
        with exclusive_test_lock():
            reserve(args.output)
            try:
                backend = backend_factory(); backend.discover(); verify_live_base(context, backend)
                observer = observer_factory(context['bundle']); observer.sample(active_task(context))
                capture = capture_factory(context, backend, observer)
                started = audit = time.monotonic()
                while time.monotonic()-started < args.wall_seconds:
                    if not capture.tick(): break
                    if time.monotonic()-audit > 1:
                        atomic_write_json(args.output, capture.report()); audit = time.monotonic()
                    time.sleep(0.03)
                else:
                    raise ObstacleError('OBSERVATION_WALL_LIMIT_REACHED')
            except BaseException as exc:
                error = str(exc) if isinstance(exc, ObstacleError) else type(exc).__name__
                raise
            finally:
                report = capture.report(error) if capture else Metrics(identity(context)).report(error=error or 'NO_OBSERVATION')
                try:
                    if capture: capture.close()
                finally:
                    if observer: observer.close()
                    atomic_write_json(args.output, report)
        print(json.dumps({'output': str(args.output), 'quality': report['quality']}, indent=2)); return 0
    except (Exception, KeyboardInterrupt) as exc:
        print(json.dumps({'error': str(exc) if isinstance(exc, ObstacleError) else type(exc).__name__,
                          'navigation_started': False}), file=sys.stderr)
        return 130 if isinstance(exc, KeyboardInterrupt) else 1
    finally:
        if old_term is not None: signal.signal(signal.SIGTERM, old_term)


if __name__ == '__main__':
    raise SystemExit(main())
