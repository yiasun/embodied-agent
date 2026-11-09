"""Map- and path-derived obstacle selection, independent of ROS/Gazebo bindings."""
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from planner.plan_identity import compute_plan_id,validate_trusted_plan,ASSET_PLAN_ID_SCHEME
from planner.provenance import validate_source_record
from planner.validation import strict_json_loads
from scenes.assets import validate_scene_assets
from scenes.geometry import raster,read_pgm,static_navigation_check


class ObstacleError(ValueError):
    pass


def load_context(plan_path,report_path,root):
    plan,coordinates=validate_trusted_plan(strict_json_loads(Path(plan_path).read_text()))
    if plan['scene_id']!='inspection_world' or plan['schema_version']!='2.0':
        raise ObstacleError('INSPECTION_V2_PLAN_REQUIRED')
    from planner.planner_v1 import load_scene
    scene=load_scene('inspection_world');bundle=validate_scene_assets(scene,root)
    report=strict_json_loads(Path(report_path).read_text())
    if (report.get('report_version')!='2.0' or report.get('plan_id_scheme')!=ASSET_PLAN_ID_SCHEME
            or compute_plan_id(report.get('plan'))!=compute_plan_id(plan)
            or report.get('plan_id')!=compute_plan_id(plan)):
        raise ObstacleError('REPORT_PLAN_MISMATCH')
    source=validate_source_record(report.get('source'),report['plan_id'],plan_id_scheme=ASSET_PLAN_ID_SCHEME)
    if not isinstance(report.get('run_id'),str) or not report['run_id']:
        raise ObstacleError('INVALID_RUN_ID')
    return {'plan':plan,'plan_id':report['plan_id'],'run_id':report['run_id'],
            'provenance_id':source['provenance_id'],'report':report,'bundle':bundle,
            'report_path':str(Path(report_path).resolve())}


def bound_report(context):
    """Revalidate identity between tasks; this alone never permits injection."""
    report=strict_json_loads(Path(context['report_path']).read_text())
    if (report.get('run_id')!=context['run_id'] or report.get('plan_id')!=context['plan_id']
            or report.get('source',{}).get('provenance_id')!=context['provenance_id']):
        raise ObstacleError('NAVIGATION_IDENTITY_CHANGED')
    if compute_plan_id(report.get('plan'))!=context['plan_id']:
        raise ObstacleError('ACTIVE_REPORT_PLAN_CHANGED')
    validate_source_record(report.get('source'),context['plan_id'],plan_id_scheme=ASSET_PLAN_ID_SCHEME)
    return report


def active_task(context):
    report=strict_json_loads(Path(context['report_path']).read_text())
    if (report.get('status')!='RUNNING' or report.get('run_id')!=context['run_id']
            or report.get('plan_id')!=context['plan_id'] or report.get('source',{}).get('provenance_id')!=context['provenance_id']):
        raise ObstacleError('NAVIGATION_NOT_ACTIVE')
    if compute_plan_id(report.get('plan'))!=context['plan_id']:
        raise ObstacleError('ACTIVE_REPORT_PLAN_CHANGED')
    validate_source_record(report.get('source'),context['plan_id'],plan_id_scheme=ASSET_PLAN_ID_SCHEME)
    tasks=[task for task in report.get('tasks',[]) if task.get('status')=='RUNNING']
    if len(tasks)!=1:raise ObstacleError('ACTIVE_TASK_AMBIGUOUS')
    task=tasks[0];index=task.get('id')
    if type(index) is not int or not 1<=index<=len(context['plan']['steps']):raise ObstacleError('INVALID_ACTIVE_TASK')
    expected=context['plan']['steps'][index-1]
    if task.get('x')!=expected['x'] or task.get('y')!=expected['y']:raise ObstacleError('ACTIVE_TASK_TARGET_MISMATCH')
    return task


def site_profile(bundle,mode):
    boxes={b['name']:b['bounds'] for b in bundle['source']['boxes']}
    south,north,divider=boxes['south_partition_mid'],boxes['north_partition_mid'],boxes['south_room_divider']
    # The two partition inner faces form the real corridor cross-section.
    low,high=south[3],north[1]
    band=[south[0]+0.5,min(south[2]-0.5,divider[0]-0.5)]
    if not low<high or not band[0]<band[1]:raise ObstacleError('NO_TEST_CORRIDOR')
    if mode not in ('temporary','persistent'):raise ObstacleError('INVALID_TEST_MODE')
    size=[0.5,0.5,1.2] if mode=='temporary' else [0.4,high-low-0.1,1.2]
    return {'mode':mode,'x_band':band,'corridor_y':[low,high],'size':size,
            'derived_from':['south_partition_mid','north_partition_mid','south_room_divider'],
            'selection':'fresh Nav2 /plan required; no live coordinate has been selected'}


def bounds_for(pose,size):
    return [round(v,6) for v in (pose[0]-size[0]/2,pose[1]-size[1]/2,pose[0]+size[0]/2,pose[1]+size[1]/2)]


def point_box_distance(x,y,bounds):
    return math.hypot(max(bounds[0]-x,0,x-bounds[2]),max(bounds[1]-y,0,y-bounds[3]))


def check_placement(bundle,mode,pose,path,robot):
    profile=site_profile(bundle,mode);size=profile['size'];low,high=profile['corridor_y']
    if any(not math.isfinite(v) for v in [*pose,*robot]) or len(pose)!=3 or len(robot)!=2:
        raise ObstacleError('INVALID_POSITION')
    if not profile['x_band'][0]<=pose[0]<=profile['x_band'][1] or abs(pose[1]-(low+high)/2)>1e-7:
        raise ObstacleError('OUTSIDE_APPROVED_TEST_SITE')
    if abs(pose[2]-size[2]/2)>1e-7:raise ObstacleError('INVALID_OBSTACLE_HEIGHT')
    bounds=bounds_for(pose,size);grid=bundle['source']['map']
    _,_,pixels=read_pgm(bundle['paths']['map_image'].read_bytes())
    occupied=raster([bounds],grid)
    if any(a==0 and b==0 for a,b in zip(occupied,pixels)):
        raise ObstacleError('OBSTACLE_INTERSECTS_STATIC_GEOMETRY')
    if not path or any(len(p)!=2 or any(not math.isfinite(v) for v in p) for p in path):
        raise ObstacleError('INVALID_LIVE_PATH')
    if not any(point_box_distance(x,y,bounds)<=0.05 for x,y in path):
        raise ObstacleError('OBSTACLE_NOT_ON_ACTUAL_PATH')
    params=json.loads(bundle['paths']['nav2_params'].read_text())
    controller=params['controller_server']['ros__parameters']['FollowPath']
    if controller.get('motion_model')!='DiffDrive':raise ObstacleError('UNSUPPORTED_SPEED_MODEL')
    speed=max(abs(controller['vx_max']),abs(controller['vx_min']))
    # 2s service deadline + <=0.5s observation age, plus 1m residual margin.
    separation=bundle['source']['robot_radius_m']+1.0+speed*2.5
    if point_box_distance(*robot,bounds)<separation:
        raise ObstacleError('ROBOT_TOO_CLOSE_TO_INJECTION')
    overlay=bytes(0 if a==0 or b==0 else 255 for a,b in zip(pixels,occupied))
    # Prove this is a genuine cut for C, not just an arbitrarily named blocker.
    if mode=='persistent':
        end={'x':path[-1][0],'y':path[-1][1]}
        static_navigation_check(pixels,grid,{'x':robot[0],'y':robot[1]},{'target':end},0.3)
        try:static_navigation_check(overlay,grid,{'x':robot[0],'y':robot[1]},{'target':end},0.3)
        except ValueError:pass
        else:raise ObstacleError('BLOCKER_DOES_NOT_CUT_CURRENT_ROUTE')
    return {'position':list(pose),'size':size,'bounds':bounds,'required_robot_separation':separation,
            'path_sha256':hashlib.sha256(json.dumps(path,separators=(',',':')).encode()).hexdigest()}


def select_position(bundle,mode,path,robot):
    profile=site_profile(bundle,mode);resolution=bundle['source']['map']['resolution']
    center=sum(profile['x_band'])/2
    candidates=sorted({round(x/resolution)*resolution for x,y in path
                       if profile['x_band'][0]<=x<=profile['x_band'][1]},key=lambda x:abs(x-center))
    last=None
    for x in candidates:
        pose=[round(x,4),sum(profile['corridor_y'])/2,profile['size'][2]/2]
        try:return check_placement(bundle,mode,pose,path,robot)
        except ObstacleError as exc:last=exc
    raise last or ObstacleError('NO_SAFE_ACTUAL_PATH_CANDIDATE')


def obstacle_sdf(name,selection,token):
    root=ET.Element('sdf',version='1.9');model=ET.SubElement(root,'model',name=name)
    ET.SubElement(model,'static').text='true'
    ET.SubElement(model,'pose').text=' '.join(map(str,[*selection['position'],0,0,0]))
    link=ET.SubElement(model,'link',name='ea_link_'+token)
    for tag in ['collision','visual']:
        node=ET.SubElement(link,tag,name='ea_'+tag+'_'+token)
        geometry=ET.SubElement(node,'geometry');box=ET.SubElement(geometry,'box')
        ET.SubElement(box,'size').text=' '.join(map(str,selection['size']))
        if tag=='visual':
            ET.SubElement(node,'transparency').text='0'
            material=ET.SubElement(node,'material')
            ET.SubElement(material,'ambient').text='0.9 0.08 0.08 1'
            ET.SubElement(material,'diffuse').text='0.9 0.08 0.08 1'
    return ET.tostring(root,encoding='unicode')


def verify_owned_model(world_sdf,record):
    world=ET.fromstring(world_sdf).find('world')
    models=[m for m in world.findall('model') if m.get('name')==record['entity_name']]
    if len(models)!=1:raise ObstacleError('OWNED_MODEL_NOT_UNIQUE')
    model=models[0];token=record['lease_id'];selection=record['selection']
    if 'motion' in record:
        from .trajectory import verify_motion_pose
        if model.findtext('static') not in ('false','0'):raise ObstacleError('OWNED_MOTION_MODEL_NOT_DYNAMIC')
        verify_motion_pose(model,record)
    else:
        if model.findtext('static') not in ('true','1'):raise ObstacleError('OWNED_MODEL_NOT_STATIC')
        pose=list(map(float,model.findtext('pose','').split()))
        if len(pose)!=6 or any(not math.isfinite(v) for v in pose) or any(abs(a-b)>1e-6 for a,b in zip(pose,[*selection['position'],0,0,0])):
            raise ObstacleError('OWNED_MODEL_POSE_MISMATCH')
    links=model.findall('link')
    if len(links)!=1 or links[0].get('name')!='ea_link_'+token:raise ObstacleError('OWNERSHIP_TOKEN_MISMATCH')
    local_pose=list(map(float,links[0].findtext('pose','0 0 0 0 0 0').split()))
    if len(local_pose)!=6 or any(not math.isfinite(v) or abs(v)>1e-6 for v in local_pose):raise ObstacleError('OWNED_LINK_POSE_MISMATCH')
    for tag in ['collision','visual']:
        nodes=links[0].findall(tag)
        if len(nodes)!=1 or nodes[0].get('name')!='ea_'+tag+'_'+token:raise ObstacleError('OWNERSHIP_TOKEN_MISMATCH')
        local_pose=list(map(float,nodes[0].findtext('pose','0 0 0 0 0 0').split()))
        if len(local_pose)!=6 or any(not math.isfinite(v) or abs(v)>1e-6 for v in local_pose):raise ObstacleError('OWNED_GEOMETRY_POSE_MISMATCH')
        size=list(map(float,nodes[0].findtext('geometry/box/size','').split()))
        if len(size)!=3 or any(not math.isfinite(v) for v in size) or any(abs(a-b)>1e-6 for a,b in zip(size,selection['size'])):
            raise ObstacleError('OWNED_MODEL_GEOMETRY_MISMATCH')
    if float(links[0].findtext('visual/transparency','0'))!=0:raise ObstacleError('OWNED_MODEL_INVISIBLE')
    return True


def find_active_report(plan_path,logs_dir,root):
    plan,_=validate_trusted_plan(strict_json_loads(Path(plan_path).read_text()))
    wanted=compute_plan_id(plan);matches=[]
    for path in Path(logs_dir).glob('task_v4_*.json'):
        try:
            data=strict_json_loads(path.read_text())
            if data.get('status')!='RUNNING' or data.get('plan_id')!=wanted:continue
            context=load_context(plan_path,path,root);active_task(context)
            matches.append(path.resolve())
        except (ValueError,KeyError,TypeError,OSError):continue
    if len(matches)!=1:raise ObstacleError('EXPECTED_EXACTLY_ONE_MATCHING_ACTIVE_RUN')
    return matches[0]
