"""V4 sequential orchestration. Importing this module never initializes ROS."""
import argparse
import re
import math
import signal
import sys
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

if __package__:
    from .recovery_policy import decide
    from .reporting import atomic_write_json
else:
    from recovery_policy import decide
    from reporting import atomic_write_json

ROOT = Path(__file__).resolve().parents[1]
# Direct script and package entries share the schema's task limits.
if not __package__:
    sys.path.insert(0, str(ROOT))
from planner.validation import MIN_TASKS, MAX_TASKS, strict_json_loads
from planner.plan_identity import PLAN_ID_SCHEME, compute_plan_id, require_plan_id, validate_trusted_plan, plan_scheme
from planner.provenance import load_source, validate_source_record
EXIT_CODES = {'SUCCEEDED': 0, 'FAILED': 1, 'REJECTED': 1, 'CANCELLED': 1,
              'TIMED_OUT': 3, 'UNKNOWN': 4, 'INTERRUPTED': 130}


def now_utc():
    return datetime.now(timezone.utc).isoformat()


def validate_arguments(goals, timeout, retries):
    if not MIN_TASKS <= len(goals) <= MAX_TASKS:
        raise ValueError('任务数量必须在1到20之间')
    if not math.isfinite(timeout) or timeout <= 0 or type(retries) is not int or retries < 0:
        raise ValueError('timeout必须为正有限数值，retries必须为非负整数')
    for pair in goals:
        if len(pair) != 2 or any(type(v) not in (int, float) or not math.isfinite(v) for v in pair):
            raise ValueError('坐标必须成对且为有限数值')


def run_tasks(goals, navigator_factory, report_path, timeout=120.0, retries=1,
              execution_id=None, plan_id=None, writer=atomic_write_json,
              sleep=time.sleep, finalize=lambda: None, *, trusted_plan=None, source=None, before_goal=None):
    """Injected seams isolate offline tests from ROS; preserve V4 retry semantics."""
    validate_arguments(goals, timeout, retries)
    if execution_id is not None and (not isinstance(execution_id, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,127}', execution_id)):
        raise ValueError('非法run_id')
    if trusted_plan is not None:
        trusted_plan, registered = validate_trusted_plan(trusted_plan)
        expected_goals = list(zip(map(float, registered[::2]), map(float, registered[1::2])))
        if list(goals) != expected_goals:
            raise ValueError('目标坐标与可信计划不匹配')
        computed_id = compute_plan_id(trusted_plan)
        if plan_id is not None:
            require_plan_id(plan_id)
            if plan_id != computed_id:
                raise ValueError('执行器plan_id不匹配')
        plan_id = computed_id
        if source is not None:
            source = validate_source_record(source, plan_id, plan_id_scheme=plan_scheme(trusted_plan))
    elif plan_id is not None or source is not None:
        raise ValueError('plan_id或来源不能代替完整计划校验')
    report_path = Path(report_path)
    # Refuse to overwrite an old experiment, even if --report is supplied.
    if report_path.exists():
        raise ValueError('报告路径已存在，拒绝覆盖')
    report = {
        'report_version': '2.0', 'run_id': execution_id or uuid.uuid4().hex,
        'plan_id_scheme': plan_scheme(trusted_plan) if trusted_plan is not None else None,
        'execution_mode': 'validated-plan' if trusted_plan is not None else 'coordinates-only',
        'plan': trusted_plan, 'source': source,
        'plan_id': plan_id, 'status': 'PENDING', 'created_at': now_utc(),
        'max_retries': retries,
        'tasks': [{'id': i, 'x': x, 'y': y, 'status': 'PENDING', 'attempts': []}
                  for i, (x, y) in enumerate(goals, 1)],
    }
    node = None
    current = None

    def stop_active():
        if node is None:
            return True
        try:
            return node.abort_active()
        except (Exception, KeyboardInterrupt, SystemExit):
            return False

    try:
        # A writable audit trail is required before any goal can be sent.
        writer(report_path, report)
        node = navigator_factory()
        report['status'] = 'RUNNING'
        writer(report_path, report)
        for current in report['tasks']:
            current.update(status='RUNNING', started_at=now_utc())
            task_start = time.monotonic()
            writer(report_path, report)
            for attempt_num in range(1, retries + 2):
                attempt = {'number': attempt_num, 'started_at': now_utc(), 'status': 'RUNNING'}
                current['attempts'].append(attempt)
                writer(report_path, report)
                started = time.monotonic()
                if before_goal is not None:
                    before_goal(node)
                success = node.navigate(current['x'], current['y'], timeout_sec=timeout)
                status, reason = node.last_status, node.last_reason
                # Never let an inconsistent client response advance the chain.
                if (status == 'SUCCEEDED') != bool(success) or status not in EXIT_CODES:
                    status, reason, success = 'UNKNOWN', 'inconsistent_navigation_result', False
                decision = decide(status, reason, attempt_num, retries)
                attempt.update(status=status, reason=reason, ended_at=now_utc(),
                               duration_seconds=round(time.monotonic() - started, 2),
                               recovery_action=decision.action, recovery_reason=decision.reason)
                writer(report_path, report)
                if decision.action != 'RETRY':
                    break
                sleep(1)
            current.update(status=status, reason=reason, ended_at=now_utc(),
                           duration_seconds=round(time.monotonic() - task_start, 2))
            writer(report_path, report)
            if not success:
                report['status'] = status
                break
        else:
            report['status'] = 'SUCCEEDED'
    except (Exception, KeyboardInterrupt, SystemExit) as exc:
        confirmed = stop_active()
        status = ('INTERRUPTED' if isinstance(exc, (KeyboardInterrupt, SystemExit)) else 'FAILED') if confirmed else 'UNKNOWN'
        # Preserve UNKNOWN from the Action client even if a later cleanup succeeds.
        if node is not None and getattr(node, 'last_status', None) == 'UNKNOWN':
            status = 'UNKNOWN'
        report.update(status=status, error_type=type(exc).__name__)
        if current is not None and current['status'] == 'RUNNING':
            current.update(status=status, reason='execution_interrupted' if isinstance(exc, (KeyboardInterrupt, SystemExit))
                           else 'execution_exception', ended_at=now_utc())
            if current['attempts'] and current['attempts'][-1]['status'] == 'RUNNING':
                current['attempts'][-1].update(status=status, reason=current['reason'], ended_at=now_utc(),
                                               recovery_action='HALT', recovery_reason='execution_stopped')
    finally:
        # Also try to resolve a late send response when navigate returned UNKNOWN.
        if not stop_active():
            report['status'] = 'UNKNOWN'
            if current is not None:
                current.update(status='UNKNOWN', reason='termination_unconfirmed')
        for task in report['tasks']:
            if task['status'] == 'PENDING':
                task.update(status='SKIPPED', reason='task_chain_stopped')
        cleanup_failed = False
        if node is not None:
            try:
                node.destroy_node()
            except (Exception, KeyboardInterrupt, SystemExit):
                cleanup_failed = True
        try:
            finalize()
        except (Exception, KeyboardInterrupt, SystemExit):
            cleanup_failed = True
        if cleanup_failed:
            report['cleanup_error'] = True
            if report['status'] == 'SUCCEEDED':
                report['status'] = 'FAILED'
        report['ended_at'] = now_utc()
        try:
            writer(report_path, report)
        except (Exception, KeyboardInterrupt, SystemExit):
            if report['status'] not in ('UNKNOWN', 'INTERRUPTED'):
                report['status'] = 'FAILED'
            report['report_write_failed'] = True
            print('最终报告写入失败；执行未被认定成功', file=sys.stderr)
    return EXIT_CODES.get(report['status'], 1), report


def main(argv=None):
    # ROS flags are forwarded unchanged and stripped without importing ROS in preview.
    raw = list(sys.argv[1:] if argv is None else argv)
    ros_index = raw.index('--ros-args') if '--ros-args' in raw else len(raw)
    parser = argparse.ArgumentParser()
    parser.add_argument('coordinates', nargs='*', type=float)
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--retries', type=int, default=1)
    parser.add_argument('--execute', action='store_true', help='显式发送导航目标')
    parser.add_argument('--report', type=Path)
    parser.add_argument('--execution-id')
    parser.add_argument('--plan-id')
    parser.add_argument('--plan', type=Path)
    parser.add_argument('--provenance', type=Path)
    parser.add_argument('--source-id')
    parser.add_argument('--allow-legacy-provenance', action='store_true')
    args = parser.parse_args(raw[:ros_index])
    if len(args.coordinates) % 2:
        parser.error('坐标必须成对')
    goals = list(zip(args.coordinates[::2], args.coordinates[1::2]))
    trusted_plan = source = None
    try:
        if args.plan is not None:
            trusted_plan, coordinates = validate_trusted_plan(strict_json_loads(args.plan.read_text(encoding='utf-8')))
            registered_goals = list(zip(map(float, coordinates[::2]), map(float, coordinates[1::2])))
            if goals and goals != registered_goals:
                raise ValueError('CLI目标与计划不匹配')
            goals = registered_goals
            computed_id = compute_plan_id(trusted_plan)
            if args.plan_id is not None:
                require_plan_id(args.plan_id)
                if args.plan_id != computed_id:
                    raise ValueError('plan_id不匹配')
            args.plan_id = computed_id
            provenance_path = args.provenance
            if provenance_path is None and (args.plan.parent / 'provenance.json').is_file():
                provenance_path = args.plan.parent / 'provenance.json'
            if provenance_path:
                source = load_source(provenance_path, trusted_plan, allow_legacy=args.allow_legacy_provenance)
            if args.source_id is not None:
                require_plan_id(args.source_id)
                if source is None or source['provenance_id'] != args.source_id:
                    raise ValueError('来源摘要不匹配')
        elif args.plan_id is not None or args.provenance is not None or args.source_id is not None:
            raise ValueError('必须提供完整计划')
        validate_arguments(goals, args.timeout, args.retries)
        if args.execution_id is not None and not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,127}', args.execution_id):
            raise ValueError('非法run_id')
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    if not args.execute:
        print(f'预检查通过：{len(goals)}个任务；未发送导航目标')
        if trusted_plan is not None:
            print(f'plan_id ({plan_scheme(trusted_plan)}): {args.plan_id}')
        return 0
    # Only explicit execution loads ROS and creates a node.
    import rclpy
    if __package__:
        from .task_executor_v2 import RobotNavigator
    else:
        from task_executor_v2 import RobotNavigator

    def create():
        rclpy.init(args=[sys.argv[0], *raw[ros_index:]])
        node = RobotNavigator()
        try:
            if trusted_plan is not None and trusted_plan['schema_version'] == '2.0':
                from planner.planner_v1 import load_scene
                from scenes.runtime_guard import assert_runtime_map
                assert_runtime_map(node, load_scene(trusted_plan['scene_id']), ROOT)
        except BaseException:
            node.destroy_node()
            raise
        return node

    def before_goal(node):
        if trusted_plan is not None and trusted_plan['schema_version'] == '2.0':
            from planner.planner_v1 import load_scene
            from scenes.runtime_guard import assert_runtime_map
            assert_runtime_map(node, load_scene(trusted_plan['scene_id']), ROOT)

    def shutdown():
        if rclpy.ok():
            rclpy.shutdown()

    execution_id = args.execution_id or uuid.uuid4().hex
    path = args.report or ROOT / 'logs' / f'task_v4_{execution_id}.json'
    def interrupted(signum, frame):
        raise KeyboardInterrupt

    old_sigterm = signal.signal(signal.SIGTERM, interrupted)
    try:
        try:
            code, _ = run_tasks(goals, create, path, args.timeout, args.retries,
                                execution_id, args.plan_id, finalize=shutdown,
                                trusted_plan=trusted_plan, source=source, before_goal=before_goal)
        except (ValueError, OSError) as exc:
            print(f'执行器拒绝启动: {type(exc).__name__}', file=sys.stderr)
            return 1
    finally:
        signal.signal(signal.SIGTERM, old_sigterm)
    print(f'执行报告：{path}')
    return code


if __name__ == '__main__':
    raise SystemExit(main())
