import copy
import json
import subprocess
import sys
from pathlib import Path
from unittest.mock import Mock

import pytest
from planner import execute_plan, planner_v1
from planner.validation import MAX_TASKS, MIN_TASKS, strict_json_loads

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def plan():
    return planner_v1.build_plan(planner_v1.load_scene('turtlebot3_demo'), ['A区', 'B区'])


def validate(tmp_path, plan):
    path = tmp_path / 'plan.json'
    path.write_text(json.dumps(plan))
    return execute_plan.read_and_validate_plan(path)


@pytest.mark.parametrize('count', [1, 2, 20])
def test_valid_counts(tmp_path, count):
    plan = planner_v1.build_plan(planner_v1.load_scene('turtlebot3_demo'), ['A区'] * count)
    assert len(validate(tmp_path, plan)[1]) == count * 2


@pytest.mark.parametrize('count', [0, 21])
def test_builder_limits(count):
    with pytest.raises(ValueError):
        planner_v1.build_plan(planner_v1.load_scene('turtlebot3_demo'), ['A区'] * count)


@pytest.mark.parametrize('field,value', [
    ('location', 'C区'), ('location', []), ('action', 'SHELL'), ('action', True),
    ('x', 99), ('x', float('nan')), ('x', float('inf')), ('y', float('-inf')),
    ('x', True), ('x', '0.419'), ('step_id', True), ('step_id', 1.0),
    ('step_id', 2), ('command', 'anything')])
def test_invalid_step(tmp_path, plan, field, value):
    plan['steps'][0][field] = value
    with pytest.raises(ValueError):
        validate(tmp_path, plan)


@pytest.mark.parametrize('value', [[], None, 'text', 1, True])
def test_wrong_top_level(tmp_path, value):
    with pytest.raises(ValueError):
        validate(tmp_path, value)


@pytest.mark.parametrize('field,value', [
    ('extra', 1), ('scene_id', '../escape'), ('scene_id', 'x/y'),
    ('frame_id', 'odom'), ('schema_version', '2.0'), ('status', 'RUNNING'),
    ('created_at', 'invalid'), ('steps', []), ('steps', [None]), ('steps', {}),
])
def test_invalid_plan(tmp_path, plan, field, value):
    plan[field] = value
    with pytest.raises(ValueError):
        validate(tmp_path, plan)


def test_too_many_steps(tmp_path, plan):
    plan['steps'] *= 11
    with pytest.raises(ValueError):
        validate(tmp_path, plan)


@pytest.mark.parametrize('field', ['steps', 'scene_id', 'created_at'])
def test_missing_fields(tmp_path, plan, field):
    del plan[field]
    with pytest.raises(ValueError):
        validate(tmp_path, plan)


@pytest.mark.parametrize('text', ['{', '{"a":1,"a":2}', 'NaN', 'Infinity'])
def test_json_fail_closed(text):
    with pytest.raises(ValueError):
        strict_json_loads(text)


@pytest.mark.parametrize('scene_id', ['../escape', '/tmp/scene', 'x/y', '', None, 'x.json', 'x\n'])
def test_scene_id_boundary(scene_id):
    with pytest.raises(ValueError):
        planner_v1.load_scene(scene_id)


def test_symlink_scene_escape(tmp_path, monkeypatch):
    directory = tmp_path / 'configs/scenes'; directory.mkdir(parents=True)
    outside = tmp_path / 'outside.json'; outside.write_text('{}')
    (directory / 'escape.json').symlink_to(outside)
    monkeypatch.setattr(planner_v1, 'ROOT', tmp_path)
    with pytest.raises(ValueError, match='越界'):
        planner_v1.load_scene('escape')


@pytest.mark.parametrize('scene', [[], {}, {'scene_id':'demo','frame_id':'map','waypoints':{'A区':[]}},
    {'scene_id':'demo','frame_id':'map','waypoints':{'A区':{'x':True,'y':0}}}])
def test_bad_registry(scene):
    with pytest.raises(ValueError):
        planner_v1.build_plan(scene, ['A区'])


def test_registry_coordinates_are_authoritative(tmp_path, plan):
    plan['steps'][0]['x'] += 0.0005
    # No tolerance substitution: the identity must describe the exact target.
    with pytest.raises(ValueError):
        validate(tmp_path, plan)


def test_preview_does_not_launch(monkeypatch):
    run = Mock(side_effect=AssertionError('must not launch'))
    monkeypatch.setattr(execute_plan, 'run_executor', run)
    assert execute_plan.main(['--plan', str(ROOT/'plans/plan_nl_v1.json')]) == 0
    run.assert_not_called()


@pytest.mark.parametrize('entry', [
    ['planner/execute_plan.py'], ['-m', 'planner.execute_plan'],
    ['planner/planner_v1.py', '--help'], ['-m', 'planner.planner_v1', '--help'],
    ['planner/nl_parser_v1.py', '--help'], ['-m', 'planner.nl_parser_v1', '--help'],
    ['day4/task_executor_v2.py', '0', '0'], ['-m', 'day4.task_executor_v2', '0', '0'],
    ['day4/task_executor_v4.py', '0', '0'], ['-m', 'day4.task_executor_v4', '0', '0'],
])
def test_cli_without_ros_or_writes(entry):
    result = subprocess.run([sys.executable, '-B', *entry], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
