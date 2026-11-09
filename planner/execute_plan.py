import argparse
import signal
import subprocess
import sys
import uuid
from pathlib import Path

if __package__:
    from .planner_v1 import load_scene
    from .validation import strict_json_loads, validate_plan_structure
else:
    from planner_v1 import load_scene
    from validation import strict_json_loads, validate_plan_structure

ROOT = Path(__file__).resolve().parents[1]
if not __package__:
    sys.path.insert(0, str(ROOT))
from planner.plan_identity import PLAN_ID_SCHEME, compute_plan_id, require_plan_id, validate_trusted_plan, plan_scheme
from planner.provenance import load_source, validate_source_record


def read_and_validate_plan(path):
    plan = strict_json_loads(path.read_text(encoding='utf-8'))
    return validate_trusted_plan(plan)


def verify_execution_report(path, execution_id, plan_id, coordinates, *, expected_source=None):
    report = strict_json_loads(path.read_text(encoding='utf-8'))
    require_plan_id(plan_id)
    if not isinstance(report, dict) or report.get('report_version') != '2.0':
        raise ValueError('无效执行报告')
    if report.get('run_id') != execution_id or report.get('plan_id') != plan_id:
        raise ValueError('执行报告来源不匹配')
    if (report.get('plan_id_scheme') != plan_scheme(report.get('plan'))
            or compute_plan_id(report.get('plan')) != plan_id):
        raise ValueError('报告计划内容不匹配')
    source = report.get('source')
    if source is not None:
        validate_source_record(source, plan_id, plan_id_scheme=plan_scheme(report['plan']))
    if source != expected_source:
        raise ValueError('报告来源不匹配')
    if report.get('status') != 'SUCCEEDED' or not report.get('ended_at'):
        raise ValueError('任务未成功完成')
    tasks = report.get('tasks')
    if not isinstance(tasks, list) or len(tasks) * 2 != len(coordinates):
        raise ValueError('执行报告任务数量不匹配')
    for index, task in enumerate(tasks, 1):
        if not isinstance(task, dict) or type(task.get('id')) is not int or task['id'] != index:
            raise ValueError('执行报告步骤不匹配')
        if task.get('status') != 'SUCCEEDED' or task.get('reason') != 'goal_reached':
            raise ValueError('执行报告含未成功任务')
        if any(type(task.get(k)) not in (int, float) or task[k] != float(coordinates[2*(index-1)+j])
               for j, k in enumerate(('x', 'y'))):
            raise ValueError('执行报告坐标不匹配')
        attempts = task.get('attempts')
        if not isinstance(attempts, list) or not attempts:
            raise ValueError('缺少执行尝试记录')
        last = attempts[-1]
        if not isinstance(last, dict) or last.get('status') != 'SUCCEEDED' or last.get('recovery_action') != 'CONTINUE':
            raise ValueError('执行报告缺少成功决策')
    return report


class ExecutorStateUnknown(RuntimeError):
    pass


def run_executor(command):
    """Forward interruption to V4 and allow its bounded cancellation to finish."""
    child = subprocess.Popen(command)
    try:
        return child.wait()
    except (KeyboardInterrupt, Exception):
        try:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
            child.wait(timeout=40)
        except (subprocess.TimeoutExpired, KeyboardInterrupt, OSError):
            # Killing the client would not establish that the robot stopped.
            raise ExecutorStateUnknown('无法确认执行器已经终止') from None
        raise


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--plan', default=str(ROOT / 'plans' / 'plan_v1.json'))
    parser.add_argument('--provenance', type=Path)
    parser.add_argument('--allow-legacy-provenance', action='store_true')
    parser.add_argument('--plan-id', help='可选预期语义ID；仍重新校验完整计划')
    parser.add_argument('--execute', action='store_true', help='实际向Nav2发送导航任务')
    args = parser.parse_args(argv)
    try:
        path = Path(args.plan)
        # Validate and hash one immutable in-memory snapshot of the input.
        text = path.read_text(encoding='utf-8')
        class Snapshot:
            def read_text(self, **kwargs):
                return text
        plan, coordinates = read_and_validate_plan(Snapshot())
        plan_id = compute_plan_id(plan)
        if args.plan_id is not None:
            require_plan_id(args.plan_id)
            if args.plan_id != plan_id:
                raise ValueError('plan_id与计划不匹配')
        provenance_path = args.provenance
        if provenance_path is None and (path.parent / 'provenance.json').is_file():
            provenance_path = path.parent / 'provenance.json'
        source = load_source(provenance_path, plan, allow_legacy=args.allow_legacy_provenance) if provenance_path else None
    except (ValueError, KeyError, OSError, TypeError) as exc:
        parser.error(str(exc))
    print(f"场景：{plan['scene_id']}", flush=True)
    print(f"计划校验通过：{len(plan['steps'])}个任务", flush=True)
    print(f'plan_id ({plan_scheme(plan)}): {plan_id}', flush=True)
    if not args.execute:
        print('当前为预检查模式，未发送导航目标')
        return 0
    execution_id = uuid.uuid4().hex
    report_path = ROOT / 'logs' / f'task_v4_{execution_id}.json'
    command = [sys.executable, str(ROOT / 'day4' / 'task_executor_v4.py'),
               *coordinates, '--execute', '--timeout', '60', '--retries', '1',
               '--plan', str(path.resolve()), '--report', str(report_path), '--execution-id', execution_id, '--plan-id', plan_id,
               '--ros-args', '-p', 'use_sim_time:=true']
    if provenance_path:
        # All options precede --ros-args so V4 parses them as application flags.
        position = command.index('--ros-args')
        flags = ['--provenance', str(provenance_path.resolve()), '--source-id', source['provenance_id']]
        if args.allow_legacy_provenance:
            flags.append('--allow-legacy-provenance')
        command[position:position] = flags
    def interrupted(signum, frame):
        raise KeyboardInterrupt

    old_sigterm = signal.signal(signal.SIGTERM, interrupted)
    try:
        returncode = run_executor(command)
        if returncode != 0:
            return returncode if returncode > 0 else 128 - returncode
        verify_execution_report(report_path, execution_id, plan_id, coordinates, expected_source=source)
    except KeyboardInterrupt:
        return 130
    except ExecutorStateUnknown:
        print('UNKNOWN：无法确认执行器终止；禁止继续任务', file=sys.stderr)
        return 4
    except (ValueError, KeyError, TypeError, OSError, RuntimeError):
        print('执行失败或无法验证最终报告', file=sys.stderr)
        return 1
    finally:
        signal.signal(signal.SIGTERM, old_sigterm)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
