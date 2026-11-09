"""Read-only artifact association. Never changes historical identifiers or runs ROS."""
import argparse
import json
from pathlib import Path

from .execute_plan import read_and_validate_plan
from .plan_identity import (PLAN_ID_SCHEME, LEGACY_FILE_SCHEME, compute_plan_id,
                            legacy_file_id, require_plan_id, plan_scheme)
from .provenance import load_source
from .validation import strict_json_loads


def audit_artifacts(response_path, plan_path, report_path=None):
    plan, coordinates = read_and_validate_plan(plan_path)
    source = load_source(response_path, plan, allow_legacy=True)
    plan_id = compute_plan_id(plan)
    result = {'plan_id_scheme': plan_scheme(plan), 'plan_id': plan_id,
              'source': source, 'legacy_plan_file': {
                  'scheme': LEGACY_FILE_SCHEME,
                  'plan_id': legacy_file_id(plan_path.read_text(encoding='utf-8'))},
              'historical_records_modified': False, 'navigation_replayed': False}
    if report_path is None:
        return result
    report = strict_json_loads(report_path.read_text(encoding='utf-8'))
    if not isinstance(report, dict) or report.get('report_version') != '1.0':
        raise ValueError('此兼容审计只关联1.0历史报告')
    require_plan_id(report.get('plan_id'))
    if report['plan_id'] != result['legacy_plan_file']['plan_id']:
        raise ValueError('历史报告与计划原文件哈希不匹配')
    tasks = report.get('tasks')
    if report.get('status') != 'SUCCEEDED' or not isinstance(tasks, list) or len(tasks) * 2 != len(coordinates):
        raise ValueError('历史任务状态或数量不匹配')
    for index, task in enumerate(tasks):
        if (not isinstance(task, dict) or type(task.get('id')) is not int or task['id'] != index + 1
                or task.get('status') != 'SUCCEEDED'):
            raise ValueError('历史步骤不匹配')
        for axis, offset in [('x', 0), ('y', 1)]:
            if type(task.get(axis)) not in (int, float) or task[axis] != float(coordinates[index*2+offset]):
                raise ValueError('历史目标坐标不匹配')
    result['historical_execution'] = {
        'report_version': '1.0', 'run_id': report.get('run_id'),
        'original_plan_id': report['plan_id'], 'original_plan_id_scheme': LEGACY_FILE_SCHEME,
        'associated_semantic_plan_id': plan_id, 'status': report['status'],
        'association_basis': 'original file hash and ordered coordinates; model source was not recorded in this historical report'}
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description='只读关联历史来源、计划和可选执行报告')
    parser.add_argument('--response', required=True, type=Path)
    parser.add_argument('--plan', required=True, type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args(argv)
    try:
        result = audit_artifacts(args.response, args.plan, args.report)
    except (ValueError, OSError, KeyError, TypeError):
        print('{"error":"ARTIFACT_ASSOCIATION_FAILED"}')
        return 1
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
