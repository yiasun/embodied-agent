"""Leased obstacle lifecycle. Create once; verify ownership before every removal."""
import copy
import json
import math
import re
import time
import xml.etree.ElementTree as ET
from datetime import datetime,timezone
from pathlib import Path

from day4.reporting import atomic_write_json
from .safety import (ObstacleError,active_task,check_placement,obstacle_sdf,
                     select_position,verify_owned_model,point_box_distance)


def utc():return datetime.now(timezone.utc).isoformat()


def _model_pose(model):
    pose=list(map(float,model.findtext('pose','0 0 0 0 0 0').split()))
    if len(pose)!=6 or any(not math.isfinite(v) for v in pose):raise ObstacleError('INVALID_WORLD_POSE')
    return pose


def verify_live_base(context,backend):
    expected=ET.fromstring(context['bundle']['paths']['world'].read_bytes()).find('world')
    actual=ET.fromstring(backend.world_sdf()).find('world')
    if actual is None or actual.get('name')!=context['bundle']['source']['world_name']:
        raise ObstacleError('WRONG_LIVE_WORLD')
    original={m.get('name'):m for m in expected.findall('model')}
    models={m.get('name'):m for m in actual.findall('model')}
    entities=backend.entities()
    robot_name=backend.robot_name
    if set(models)!=set(original)|{robot_name} or set(entities)!=set(original)|{robot_name}:
        raise ObstacleError('LIVE_SCENE_EXTRA_OR_MISSING_ENTITIES')
    for name,reference in original.items():
        model=models[name]
        if model.findtext('static') not in ('true','1') or any(abs(a-b)>1e-6 for a,b in zip(_model_pose(model),_model_pose(reference))):
            raise ObstacleError('LIVE_STATIC_WORLD_MISMATCH')
        actual_links=model.findall('link');expected_links=reference.findall('link')
        if len(actual_links)!=len(expected_links) or any(any(abs(a-b)>1e-6 for a,b in zip(_model_pose(x),_model_pose(y))) for x,y in zip(actual_links,expected_links)):
            raise ObstacleError('LIVE_STATIC_LINK_POSE_MISMATCH')
        for tag in ['collision','visual']:
            expected_nodes=reference.findall('link/'+tag);actual_nodes=model.findall('link/'+tag)
            if len(expected_nodes)!=len(actual_nodes):raise ObstacleError('LIVE_STATIC_GEOMETRY_MISMATCH')
            for a,b in zip(actual_nodes,expected_nodes):
                if any(abs(x-y)>1e-6 for x,y in zip(_model_pose(a),_model_pose(b))):
                    raise ObstacleError('LIVE_STATIC_GEOMETRY_POSE_MISMATCH')
                sa=list(map(float,a.findtext('geometry/box/size','').split()))
                sb=list(map(float,b.findtext('geometry/box/size','').split()))
                if len(sa)!=3 or any(abs(x-y)>1e-6 for x,y in zip(sa,sb)):
                    raise ObstacleError('LIVE_STATIC_GEOMETRY_MISMATCH')
    poses,received=backend.poses()
    if time.monotonic()-received>0.5:raise ObstacleError('STALE_GAZEBO_POSE')
    for name,model in original.items():
        pose=poses.get(entities[name]['id'])
        if (pose is None or any(abs(a-b)>1e-5 for a,b in zip(pose['position'],_model_pose(model)[:3]))
                or any(abs(v)>1e-5 for v in pose['orientation'][:3]) or abs(abs(pose['orientation'][3])-1)>1e-5):
            raise ObstacleError('LIVE_STATIC_POSE_MISMATCH')
    robot=poses.get(entities[robot_name]['id'])
    if robot is None:raise ObstacleError('ROBOT_POSE_NOT_OBSERVED')
    return entities,robot['position'][:2]


def validate_record(record):
    if not isinstance(record,dict):raise ObstacleError('INVALID_OWNERSHIP_JOURNAL')
    token=record.get('lease_id')
    if (record.get('lease_version')!='1.0' or not isinstance(token,str)
            or not re.fullmatch('[0-9a-f]{32}',token)
            or record.get('entity_name')!='ea_stage61_'+token
            or record.get('scene_id')!='inspection_world'):
        raise ObstacleError('INVALID_OWNERSHIP_JOURNAL')
    if type(record.get('create_sent',False)) is not bool:
        raise ObstacleError('INVALID_CREATION_STATE')
    if record.get('create_sent') and (not isinstance(record.get('selection'),dict)
            or not isinstance(record.get('world_session_id'),str)
            or not re.fullmatch('[0-9a-f]{64}',record['world_session_id'])):
        raise ObstacleError('INCOMPLETE_OWNERSHIP_JOURNAL')
    if record.get('entity_id') is not None and (type(record['entity_id']) is not int or record['entity_id']<=0):
        raise ObstacleError('INVALID_ENTITY_ID')
    if 'motion' in record and record.get('create_sent'):
        from .trajectory import validate_motion_record
        validate_motion_record(record)


def owned_entity(backend,record):
    entities=backend.entities();entity=entities.get(record['entity_name'])
    if entity is None:return None
    if record.get('entity_id') is not None and entity['id']!=record['entity_id']:
        raise ObstacleError('ENTITY_ID_REUSED_OR_CHANGED')
    verify_owned_model(backend.world_sdf(),record)
    again=backend.entities().get(record['entity_name'])
    if again is None or again['id']!=entity['id']:raise ObstacleError('ENTITY_CHANGED_DURING_VERIFICATION')
    return entity['id']


def absent_from_world(backend,name):
    world=ET.fromstring(backend.world_sdf()).find('world')
    if world is None:raise ObstacleError('INVALID_WORLD_SDF_DURING_CLEANUP')
    return not any(model.get('name')==name for model in world.findall('model'))


def cleanup(backend,record,*,sleep=time.sleep,timeout=5.0):
    validate_record(record)
    record['cleanup_requested_at']=utc()
    if not record.get('create_sent'):
        record['cleanup']='NOT_NEEDED';return True
    try:
        bindings=backend.discover()
        if bindings['session_id']!=record.get('world_session_id'):
            raise ObstacleError('WORLD_SESSION_CHANGED_NO_DELETE')
        entity_id=owned_entity(backend,record)
        if entity_id is None:
            if not absent_from_world(backend,record['entity_name']):
                raise ObstacleError('SCENE_AND_PHYSICS_READBACK_DISAGREE')
            # An unacknowledged create may still be queued. Absence alone does
            # not establish that it can never appear later.
            if record.get('entity_id') is None and record.get('create_ack') is not False:
                raise ObstacleError('CREATE_OUTCOME_UNKNOWN_CANNOT_CONFIRM_ABSENCE')
            record.update(cleanup='CLEAN',cleanup_confirmed_at=utc());return True
        record['entity_id']=entity_id
        # No broad/name-only removal. This id has just been bound to our exact
        # name, token, pose, collision, visual and server session.
        backend.remove(entity_id)
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            entity=backend.entities().get(record['entity_name'])
            if entity is None and absent_from_world(backend,record['entity_name']):
                record.update(cleanup='CLEAN',cleanup_confirmed_at=utc());return True
            if entity is not None and entity['id']!=entity_id:raise ObstacleError('ENTITY_CHANGED_AFTER_REMOVE')
            sleep(0.1)
        raise ObstacleError('REMOVAL_NOT_CONFIRMED')
    except BaseException as exc:
        code=str(exc) if isinstance(exc,ObstacleError) else type(exc).__name__
        record.update(cleanup='UNKNOWN',cleanup_error=code)
        return False


def run_lease(context,backend,observer_factory,record,path,*,hold_seconds=8,
              writer=atomic_write_json,sleep=time.sleep,verification_timeout=5.0,motion_controller=None,
              wait_task_seconds=0):
    validate_record(record)
    if not math.isfinite(hold_seconds) or not 0<hold_seconds<=180:raise ObstacleError('INVALID_HOLD_DURATION')
    if not math.isfinite(wait_task_seconds) or not 0<=wait_task_seconds<=300:raise ObstacleError('INVALID_TASK_WAIT_TIMEOUT')
    record=copy.deepcopy(record);record.update(create_sent=False,create_ack=None,entity_id=None,
        cleanup='NOT_NEEDED',status='PREPARING',created_at=utc(),hold_seconds=hold_seconds)
    observer=None;error=None;audit_failure=False;create_attempted=False
    try:
        writer(path,record)  # WAL must be writable before any mutation.
        if 'motion' in record:
            # Subscribe and spin before any discovery/world RPC or task transition.
            observer=observer_factory(context['bundle'])
            if hasattr(observer,'bind_context'):observer.bind_context(context)
            if hasattr(observer,'start_preparation'):observer.start_preparation()
            if wait_task_seconds>0 and hasattr(observer,'wait_for_task'):
                record['status']='WAITING_FOR_TASK'
                def checkpoint():
                    record['path_observation']=observer.diagnostics();writer(path,record)
                observer.wait_for_task(record['target_step'],wait_task_seconds,checkpoint)
                record['status']='PREPARING'
        bindings=backend.discover()
        record.update(world_session_id=bindings['session_id'],gazebo_services=bindings)
        entities,robot=verify_live_base(context,backend)
        if record['entity_name'] in entities:raise ObstacleError('DUPLICATE_ENTITY')
        task=active_task(context)
        if record.get('target_step') is not None and task['id']!=record['target_step']:
            raise ObstacleError('WRONG_ACTIVE_TASK_FOR_TEST')
        if observer is None:observer=observer_factory(context['bundle'])
        actual_path=observer.sample(task)
        # Re-read current world poses after potentially waiting for a ROS path.
        poses,received=backend.poses()
        robot=poses.get(entities[backend.robot_name]['id'])
        if robot is None or time.monotonic()-received>0.5:raise ObstacleError('STALE_ROBOT_POSE')
        robot=robot['position'][:2]
        if math.hypot(actual_path[0][0]-robot[0],actual_path[0][1]-robot[1])>1.0:
            raise ObstacleError('PATH_START_DOES_NOT_MATCH_ROBOT')
        if 'motion' in record:
            from .trajectory import select_trajectory
            if motion_controller is None:raise ObstacleError('MOTION_CONTROLLER_REQUIRED')
            selection=select_trajectory(context['bundle'],actual_path,robot,record['motion'])
        else:
            selection=select_position(context['bundle'],record['mode'],actual_path,robot)
        if active_task(context)['id']!=task['id']:raise ObstacleError('ACTIVE_TASK_CHANGED')
        if backend.discover()['session_id']!=record['world_session_id']:
            raise ObstacleError('WORLD_SESSION_CHANGED_BEFORE_CREATE')
        if any(name.startswith('ea_stage61_') for name in backend.entities()):
            raise ObstacleError('EXISTING_TEST_OBSTACLE_NO_DUPLICATE_CREATE')
        actual_path=observer.sample(task)
        safety_bounds=selection.get('swept_bounds',selection['bounds'])
        if not any(point_box_distance(x,y,safety_bounds)<=0.05 for x,y in actual_path):
            raise ObstacleError('PATH_CHANGED_BEFORE_CREATE')
        poses,received=backend.poses()
        final_robot=poses.get(entities[backend.robot_name]['id'])
        if final_robot is None or time.monotonic()-received>0.5:raise ObstacleError('STALE_ROBOT_POSE')
        robot=final_robot['position'][:2]
        if len(robot)!=2 or any(not math.isfinite(v) for v in robot):raise ObstacleError('INVALID_FINAL_ROBOT_POSE')
        if any(not math.isfinite(v) for p in actual_path for v in p):raise ObstacleError('INVALID_FINAL_PATH')
        if point_box_distance(*robot,safety_bounds)<selection['required_robot_separation']:
            raise ObstacleError('ROBOT_APPROACHED_BEFORE_CREATE')
        if active_task(context)['id']!=task['id']:raise ObstacleError('ACTIVE_TASK_CHANGED')
        import hashlib
        selection['path_sha256']=hashlib.sha256(json.dumps(actual_path,separators=(',',':')).encode()).hexdigest()
        selection['path_evidence']={'point_count':len(actual_path),'endpoint':actual_path[-1],
            'intersecting_samples':[p for p in actual_path if point_box_distance(*p,selection['bounds'])<=0.05][:5]}
        record.update(selection=selection,active_task_id=task['id'],robot_position_before_create=robot,
                      create_requested_at=utc(),create_sent=True,status='CREATING')
        writer(path,record)
        if time.monotonic()-received>0.5:raise ObstacleError('POSE_EXPIRED_DURING_JOURNAL_WRITE')
        if 'motion' in record and hasattr(observer,'assert_selected_fresh'):
            observer.assert_selected_fresh(task)
            if time.monotonic()-received>0.5:raise ObstacleError('POSE_EXPIRED_DURING_PATH_RECHECK')
        create_attempted=True
        try:
            if 'motion' in record:
                from .trajectory import motion_sdf
                sdf=motion_sdf(record)
            else:sdf=obstacle_sdf(record['entity_name'],selection,record['lease_id'])
            record['create_ack']=bool(backend.create(record['entity_name'],sdf))
        except Exception as exc:
            record['create_ack']=None
            raise ObstacleError('CREATE_OUTCOME_UNKNOWN') from None
        if record['create_ack'] is False:raise ObstacleError('CREATE_REJECTED')
        deadline=time.monotonic()+verification_timeout
        while time.monotonic()<deadline:
            entity_id=owned_entity(backend,record)
            if entity_id is not None:
                record.update(entity_id=entity_id,create_confirmed_at=utc(),status='ACTIVE')
                writer(path,record);break
            sleep(0.1)
        else:raise ObstacleError('CREATE_READBACK_NOT_CONFIRMED')
        if 'motion' in record:
            if hasattr(observer,'stop_preparation'):observer.stop_preparation()
            motion_controller(context,backend,observer,record,path,writer)
        else:
            until=time.monotonic()+hold_seconds
            while time.monotonic()<until:
                state=json.loads(Path(context['report_path']).read_text())
                if state.get('run_id')!=context['run_id']:raise ObstacleError('RUN_REPORT_CHANGED')
                if state.get('status')!='RUNNING':
                    record['release_reason']='NAVIGATION_ENDED';break
                sleep(min(0.2,max(0,until-time.monotonic())))
        record['status']='COMPLETED'
    except BaseException as exc:
        error=exc
        if not create_attempted:record['create_sent']=False
        record['status']='INTERRUPTED' if isinstance(exc,(KeyboardInterrupt,SystemExit)) else 'FAILED'
        record['error']=str(exc) if isinstance(exc,ObstacleError) else type(exc).__name__
    finally:
        clean=cleanup(backend,record,sleep=sleep,timeout=verification_timeout)
        if observer is not None:
            if hasattr(observer,'diagnostics'):
                try:record['path_observation']=observer.diagnostics()
                except BaseException:record['path_diagnostics_failed']=True;audit_failure=True
            try:observer.close()
            except BaseException:record['observer_close_error']=True;audit_failure=True
        record['ended_at']=utc()
        try:writer(path,record)
        except BaseException:audit_failure=True;record['journal_write_failed']=True
    code=4 if not clean else 130 if isinstance(error,(KeyboardInterrupt,SystemExit)) else 1 if error or audit_failure else 0
    return code,record
