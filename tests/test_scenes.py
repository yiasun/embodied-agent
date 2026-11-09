"""Offline multi-scene resources and identity; no Gazebo or model requests."""
import copy
import hashlib
import json
import os
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace as NS, ModuleType
from unittest.mock import Mock

import pytest

from planner import agent_planner, planner_v1, execute_plan
from planner.plan_identity import (PLAN_ID_SCHEME,ASSET_PLAN_ID_SCHEME,compute_plan_id,
                                   plan_scheme,validate_trusted_plan)
from planner.provenance import load_source
from scenes.assets import sha256,validate_scene_assets
from scenes.build_assets import build,generated_assets
from scenes.geometry import raster,read_pgm,rectangles_from_sdf,static_navigation_check
from scenes.manage import launch_arguments
from scenes.runtime_guard import check_map_message,expected_ros_map,assert_runtime_map
from day4 import task_executor_v4

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture
def scene():
    return planner_v1.load_scene('inspection_world')


@pytest.fixture
def sandbox():
    return planner_v1.load_scene('turtlebot3_demo')


@pytest.fixture
def local_root(tmp_path,monkeypatch):
    shutil.copytree(ROOT/'assets',tmp_path/'assets')
    shutil.copytree(ROOT/'configs',tmp_path/'configs')
    monkeypatch.setattr(planner_v1,'ROOT',tmp_path)
    return tmp_path


def read_config(root):
    return json.loads((root/'configs/scenes/inspection_world.json').read_text())


def repin(root,scene,files):
    """Test-only manifest rewrite to reach deeper checks after hash verification."""
    manifest_path=root/scene['assets']['manifest']
    manifest=json.loads(manifest_path.read_text())
    for role in files:
        record=manifest['files'][role]
        record['sha256']=sha256((root/record['path']).read_bytes())
    manifest_path.write_text(json.dumps(manifest))
    scene['assets']['manifest_sha256']=sha256(manifest_path.read_bytes())
    index=root/'configs/scene_resources/index.json'
    data=json.loads(index.read_text());data['managed_scenes']['inspection_world']=scene['assets']
    index.write_text(json.dumps(data))
    (root/'configs/scenes/inspection_world.json').write_text(json.dumps(scene,ensure_ascii=False))


def map_message(bundle):
    config,w,h,values=expected_ros_map(bundle)
    return NS(header=NS(frame_id='map'),info=NS(width=w,height=h,resolution=config['resolution'],
        origin=NS(position=NS(x=config['origin'][0],y=config['origin'][1],z=0),
                  orientation=NS(x=0,y=0,z=0,w=1))),data=values)


def test_all_resources_and_hashes_exist(scene):
    bundle=validate_scene_assets(scene,ROOT)
    assert set(bundle['paths'])=={'geometry','nav2_template','world','map','map_image','nav2_params'}
    assert all(p.is_file() for p in bundle['paths'].values())
    assert bundle['metrics']['waypoint_count']==4
    assert bundle['metrics']['reachable_safe_cells']==39623
    assert bundle['metrics']['clearance_m']==0.45


def test_map_resolution_origin_extent_and_exact_sdf_projection(scene):
    bundle=validate_scene_assets(scene,ROOT)
    grid=bundle['source']['map']
    assert grid=={'resolution':0.05,'origin':[9.0,19.0,0.0],'width':360,'height':280}
    _,_,pixels=read_pgm(bundle['paths']['map_image'].read_bytes())
    assert pixels==raster(rectangles_from_sdf(bundle['paths']['world'].read_bytes()),grid)
    assert set(pixels)=={0,255}
    assert len(rectangles_from_sdf(bundle['paths']['world'].read_bytes()))==16


def test_collision_and_visual_geometry_match_and_no_fuel(scene):
    bundle=validate_scene_assets(scene,ROOT)
    world=ET.fromstring(bundle['paths']['world'].read_bytes()).find('world')
    assert world.get('name')=='default'
    assert not world.findall('.//include') and not world.findall('.//mesh')
    for link in world.findall('model/link'):
        if link.find('collision') is not None:
            assert link.findtext('collision/geometry/box/size')==link.findtext('visual/geometry/box/size')
    plugins={p.get('name') for p in world.findall('plugin')}
    assert 'gz::sim::systems::Physics' in plugins and 'gz::sim::systems::Sensors' in plugins


@pytest.mark.parametrize('location',['配电间','仓储间','机房','巡检站'])
def test_waypoints_free_with_clearance_and_connected(scene,location):
    bundle=validate_scene_assets(scene,ROOT)
    _,_,pixels=read_pgm(bundle['paths']['map_image'].read_bytes())
    metrics=static_navigation_check(pixels,bundle['source']['map'],bundle['source']['spawn'],
                                    {location:scene['waypoints'][location]},0.45)
    assert metrics['waypoint_count']==1


@pytest.mark.parametrize('point',[{'x':11.5,'y':21.5},{'x':10.1,'y':23.0},
                                 {'x':9.01,'y':19.01},{'x':0.419,'y':-1.822}])
def test_obstacle_edge_and_sandbox_coordinates_not_valid_static_goals(scene,point):
    bundle=validate_scene_assets(scene,ROOT)
    _,_,pixels=read_pgm(bundle['paths']['map_image'].read_bytes())
    with pytest.raises(ValueError):
        static_navigation_check(pixels,bundle['source']['map'],bundle['source']['spawn'],{'bad':point},0.45)


@pytest.mark.parametrize('kind',['world','map','map_image','geometry','nav2_params','nav2_template'])
def test_asset_mutation_rejected_even_with_same_mtime(local_root,kind):
    scene=read_config(local_root)
    manifest=json.loads((local_root/scene['assets']['manifest']).read_text())
    path=local_root/manifest['files'][kind]['path']
    stat=path.stat()
    path.write_bytes(path.read_bytes()+b' ')
    os.utime(path,ns=(stat.st_atime_ns,stat.st_mtime_ns))
    with pytest.raises(ValueError,match='hash mismatch'):
        validate_scene_assets(scene,local_root)


@pytest.mark.parametrize('kind',['world','map_image'])
def test_rehashed_but_geometrically_wrong_assets_rejected(local_root,kind):
    scene=read_config(local_root)
    manifest=json.loads((local_root/scene['assets']['manifest']).read_text())
    path=local_root/manifest['files'][kind]['path']
    if kind=='world':
        text=path.read_text().replace('name="south_wall"','name="changed_wall"')
        path.write_text(text)
    else:
        data=bytearray(path.read_bytes());data[-500]=255-data[-500];path.write_bytes(data)
    repin(local_root,scene,[kind])
    with pytest.raises(ValueError,match='differ from authored geometry'):
        validate_scene_assets(scene,local_root)


def test_missing_asset_rejected(local_root):
    scene=read_config(local_root)
    (local_root/'assets/scenes/inspection_world/v1/inspection_world.pgm').unlink()
    with pytest.raises(ValueError,match='missing'):validate_scene_assets(scene,local_root)


def test_changed_registry_or_asset_pin_rejected(local_root):
    scene=read_config(local_root)
    scene['waypoints']['配电间']['x']=0.419
    with pytest.raises(ValueError,match='registry differs'):validate_scene_assets(scene,local_root)
    scene=read_config(local_root);scene['assets']['revision']='2.0.0'
    with pytest.raises(ValueError,match='pin'):validate_scene_assets(scene,local_root)


def test_asset_metadata_cannot_be_removed_to_downgrade(local_root):
    scene=read_config(local_root);del scene['assets']
    (local_root/'configs/scenes/inspection_world.json').write_text(json.dumps(scene))
    with pytest.raises(ValueError,match='pin'):planner_v1.load_scene('inspection_world')


def test_symlink_asset_escape_rejected(local_root,tmp_path):
    scene=read_config(local_root)
    path=local_root/'assets/scenes/inspection_world/v1/inspection_world.sdf'
    outside=local_root/'outside.sdf';outside.write_bytes(path.read_bytes())
    path.unlink();path.symlink_to(outside)
    with pytest.raises(ValueError,match='escapes'):validate_scene_assets(scene,local_root)


def test_regeneration_reproducible(local_root):
    config=read_config(local_root);directory=local_root/'assets/scenes/inspection_world/v1'
    before={p.name:p.read_bytes() for p in directory.iterdir() if p.is_file()}
    build(local_root)
    assert before=={p.name:p.read_bytes() for p in directory.iterdir() if p.is_file()}
    validate_scene_assets(config,local_root)


def test_all_assets_consistent_but_waypoint_blocked_rejected(local_root):
    directory=local_root/'assets/scenes/inspection_world/v1'
    source=json.loads((directory/'geometry.json').read_text())
    source['waypoints']['配电间']['x']=11.5;source['waypoints']['配电间']['y']=21.5
    (directory/'geometry.json').write_text(json.dumps(source,ensure_ascii=False))
    build(local_root)
    scene=read_config(local_root)
    repin(local_root,scene,['geometry','world','map','map_image','nav2_params'])
    with pytest.raises(ValueError,match='clearance'):validate_scene_assets(scene,local_root)


def test_geometrically_consistent_but_disconnected_rooms_rejected(local_root):
    directory=local_root/'assets/scenes/inspection_world/v1'
    source=json.loads((directory/'geometry.json').read_text())
    source['boxes'].append({'name':'sealed_corridor','bounds':[10.2,25,25.8,25.2]})
    (directory/'geometry.json').write_text(json.dumps(source,ensure_ascii=False))
    build(local_root)
    scene=read_config(local_root)
    repin(local_root,scene,['geometry','world','map','map_image','nav2_params'])
    with pytest.raises(ValueError,match='not connected'):validate_scene_assets(scene,local_root)


@pytest.mark.parametrize('scene_id',['wrong_scene','warehouse','depot','../inspection_world'])
def test_unknown_unregistered_scene_rejected(scene_id):
    with pytest.raises(ValueError):planner_v1.load_scene(scene_id)


@pytest.mark.parametrize('target_scene,name',[('inspection_world','A区'),('inspection_world','B区'),
                                            ('turtlebot3_demo','配电间')])
def test_scene_location_isolation_before_provider(target_scene,name):
    provider=Mock()
    with pytest.raises(ValueError):
        agent_planner.plan_instruction('去'+name,planner_v1.load_scene(target_scene),provider=provider)
    provider.generate_intent.assert_not_called()


def test_new_asset_plan_identity_v2_and_legacy_unchanged(scene,sandbox):
    old=json.loads((ROOT/'plans/real_api/qwen_plan.json').read_text())
    assert compute_plan_id(old)=='e205afb6f4eca79a73bb9a9d0513c64d156df61a1ab5b9e053fab869f9e738e3'
    assert plan_scheme(old)==PLAN_ID_SCHEME
    new=planner_v1.build_plan(scene,list(scene['waypoints']))
    assert new['schema_version']=='2.0' and plan_scheme(new)==ASSET_PLAN_ID_SCHEME
    assert new['scene_assets']['manifest_sha256']==scene['assets']['manifest_sha256']
    changed=copy.deepcopy(new);changed['scene_assets']['manifest_sha256']='a'*64
    assert compute_plan_id(changed)!=compute_plan_id(new)
    with pytest.raises(ValueError):validate_trusted_plan(changed)


def test_v2_cannot_be_downgraded_or_cross_scene(scene):
    plan=planner_v1.build_plan(scene,['配电间'])
    del plan['scene_assets'];plan['schema_version']='1.0'
    with pytest.raises(ValueError,match='资产版本'):validate_trusted_plan(plan)
    plan=planner_v1.build_plan(scene,['配电间']);plan['scene_id']='turtlebot3_demo'
    with pytest.raises(ValueError):validate_trusted_plan(plan)


def test_new_plan_save_source_execution_and_report_identity(tmp_path,scene):
    result=agent_planner.plan_instruction('先去配电间，再去仓储间，接着去机房，最后去巡检站',scene,env={})
    agent_planner.save_result(result,tmp_path/'bundle')
    plan,coords=execute_plan.read_and_validate_plan(tmp_path/'bundle/plan.json')
    source=load_source(tmp_path/'bundle/provenance.json',plan)
    node=NS(navigate=Mock(return_value=True),last_status='SUCCEEDED',last_reason='goal_reached',
            abort_active=Mock(return_value=True),destroy_node=Mock())
    code,report=task_executor_v4.run_tasks([(s['x'],s['y']) for s in plan['steps']],lambda:node,
        tmp_path/'report.json',trusted_plan=plan,source=source,execution_id='offline-mock')
    assert code==0 and report['plan_id']==result.provenance['plan_id']
    assert report['plan_id_scheme']==source['plan_id_scheme']==ASSET_PLAN_ID_SCHEME
    execute_plan.verify_execution_report(tmp_path/'report.json','offline-mock',report['plan_id'],coords,expected_source=source)


def test_launch_arguments_pin_matching_world_map_nav2(scene):
    args=launch_arguments('inspection_world')
    bundle=validate_scene_assets(scene,ROOT)
    assert args['world']==str(bundle['paths']['world']) and args['map']==str(bundle['paths']['map'])
    assert args['params_file']==str(bundle['paths']['nav2_params'])
    assert (args['x_pose'],args['y_pose'])==('11.5','26.0')
    assert args['use_sim_time']=='true'


@pytest.mark.parametrize('mutation',['dimensions','origin','resolution','frame','cell','rotation'])
def test_active_wrong_map_rejected(scene,mutation):
    bundle=validate_scene_assets(scene,ROOT);message=map_message(bundle)
    if mutation=='dimensions':message.info.width=384
    if mutation=='origin':message.info.origin.position.x=-10
    if mutation=='resolution':message.info.resolution=0.03
    if mutation=='frame':message.header.frame_id='odom'
    if mutation=='cell':message.data[100]=100-message.data[100]
    if mutation=='rotation':message.info.origin.orientation.z=0.1
    with pytest.raises(ValueError):check_map_message(message,bundle)


def test_correct_live_map_readonly_gate_with_mock_reader(scene):
    bundle=validate_scene_assets(scene,ROOT)
    reader=Mock(return_value=map_message(bundle))
    assert assert_runtime_map(object(),scene,ROOT,reader=reader) is True
    reader.assert_called_once()


def test_guard_failure_in_explicit_execution_cannot_send_goals(tmp_path,scene,monkeypatch):
    import sys
    import scenes.runtime_guard as guard
    result=agent_planner.plan_instruction('去配电间',scene,env={})
    agent_planner.save_result(result,tmp_path/'bundle')
    ros=ModuleType('rclpy');ros.init=Mock();ros.ok=Mock(return_value=True);ros.shutdown=Mock()
    adapter=ModuleType('day4.task_executor_v2')
    node=NS(navigate=Mock(),destroy_node=Mock());adapter.RobotNavigator=Mock(return_value=node)
    monkeypatch.setitem(sys.modules,'rclpy',ros);monkeypatch.setitem(sys.modules,'day4.task_executor_v2',adapter)
    monkeypatch.setattr(guard,'assert_runtime_map',Mock(side_effect=ValueError('wrong map')))
    code=task_executor_v4.main(['--plan',str(tmp_path/'bundle/plan.json'),'--execute','--report',str(tmp_path/'report.json')])
    assert code!=0;node.navigate.assert_not_called();node.destroy_node.assert_called_once()
    ros.shutdown.assert_called_once()


def test_scene_cannot_change_between_goal_attempts(tmp_path,scene):
    plan=planner_v1.build_plan(scene,['配电间','仓储间'])
    node=NS(navigate=Mock(return_value=True),last_status='SUCCEEDED',last_reason='goal_reached',
            abort_active=Mock(return_value=True),destroy_node=Mock())
    gate=Mock(side_effect=[None,ValueError('map changed')])
    code,report=task_executor_v4.run_tasks([(s['x'],s['y']) for s in plan['steps']],lambda:node,
        tmp_path/'report.json',trusted_plan=plan,before_goal=gate)
    assert code!=0 and node.navigate.call_count==1
    assert report['tasks'][1]['status']=='FAILED'


def test_asset_plan_id_independent_of_project_path(local_root):
    moved=planner_v1.load_scene('inspection_world')
    moved_plan=planner_v1.build_plan(moved,list(moved['waypoints']))
    from planner.plan_identity import compute_plan_id
    assert compute_plan_id(moved_plan)=='c14287a68250a91862a14b1618ca73d63efa2e7a011484cc8c18849433ec2e7e'


def test_cannot_claim_v1_source_for_v2_plan(tmp_path,scene):
    result=agent_planner.plan_instruction('去配电间',scene,env={})
    result.provenance['plan_id_scheme']=PLAN_ID_SCHEME
    with pytest.raises(ValueError):agent_planner.save_result(result,tmp_path/'bundle')
    assert not (tmp_path/'bundle').exists()


def test_in_memory_registry_downgrade_rejected_before_provider(scene):
    forged=copy.deepcopy(scene);del forged['assets']
    provider=Mock()
    with pytest.raises(ValueError):agent_planner.plan_instruction('去配电间',forged,provider=provider)
    provider.generate_intent.assert_not_called()
    with pytest.raises(ValueError):planner_v1.build_plan(forged,['配电间'])


def test_generator_cli_default_is_readonly():
    import subprocess,sys
    directory=ROOT/'assets/scenes/inspection_world/v1'
    before={p.name:p.read_bytes() for p in directory.iterdir() if p.is_file()}
    result=subprocess.run([sys.executable,'-B','tools/build_inspection_scene.py'],cwd=ROOT,
                          capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    assert 'no files changed' in result.stdout
    assert before=={p.name:p.read_bytes() for p in directory.iterdir() if p.is_file()}


def test_yaml_must_reference_the_actual_pinned_image(local_root):
    scene=read_config(local_root)
    manifest_path=local_root/scene['assets']['manifest']
    manifest=json.loads(manifest_path.read_text())
    original=local_root/manifest['files']['map_image']['path']
    other=original.with_name('other.pgm');other.write_bytes(original.read_bytes())
    manifest['files']['map_image']['path']=str(other.relative_to(local_root))
    manifest_path.write_text(json.dumps(manifest))
    scene['assets']['manifest_sha256']=sha256(manifest_path.read_bytes())
    path=local_root/'configs/scene_resources/index.json'
    index=json.loads(path.read_text());index['managed_scenes']['inspection_world']=scene['assets']
    path.write_text(json.dumps(index))
    with pytest.raises(ValueError,match='pinned image'):validate_scene_assets(scene,local_root)
