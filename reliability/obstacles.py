"""Default Dry Run. No mutation without --apply; never launches navigation."""
import argparse
import fcntl
import json
import os
import signal
import sys
import uuid
from contextlib import contextmanager
from pathlib import Path

from day4.reporting import atomic_write_json
from planner.validation import strict_json_loads
from .lifecycle import cleanup,run_lease,validate_record
from .safety import ObstacleError,load_context,site_profile,find_active_report

ROOT=Path(__file__).resolve().parents[1]


@contextmanager
def exclusive_test_lock():
    directory=ROOT/'logs/reliability';directory.mkdir(parents=True,exist_ok=True)
    with (directory/'.obstacle.lock').open('a') as stream:
        try:fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise ObstacleError('ANOTHER_OBSTACLE_TEST_OWNS_LOCK') from None
        try:yield
        finally:fcntl.flock(stream,fcntl.LOCK_UN)


def new_record(context,mode):
    token=uuid.uuid4().hex
    return {'lease_version':'1.0','lease_id':token,'entity_name':'ea_stage61_'+token,
            'scene_id':'inspection_world','scene_assets':context['plan']['scene_assets'],
            'plan_id':context['plan_id'],'run_id':context['run_id'],'provenance_id':context['provenance_id'],
            'run_report':context['report_path'],'mode':mode,'navigation_started_by_tool':False}


def main(argv=None,*,backend_factory=None,observer_factory=None):
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    locate=commands.add_parser('find-run',help='Read-only lookup; rejects missing or ambiguous active runs')
    locate.add_argument('--plan',required=True,type=Path)
    inject=commands.add_parser('inject')
    inject.add_argument('--plan',required=True,type=Path)
    inject.add_argument('--run-report',required=True,type=Path)
    inject.add_argument('--mode',choices=['temporary','persistent'],default='temporary')
    inject.add_argument('--target-step',type=int,help='Require this task to be RUNNING; never starts or waits for navigation')
    inject.add_argument('--hold-seconds',type=float,default=8)
    inject.add_argument('--journal',type=Path)
    inject.add_argument('--apply',action='store_true')
    remove=commands.add_parser('cleanup')
    remove.add_argument('--journal',required=True,type=Path)
    remove.add_argument('--apply',action='store_true')
    probe=commands.add_parser('probe')
    probe.add_argument('--plan',type=Path)
    probe.add_argument('--run-report',type=Path)
    probe.add_argument('--live',action='store_true',help='Read-only discovery/scene inspection; no creation or removal')
    args=parser.parse_args(argv)
    if backend_factory is None:
        from .gazebo import GazeboBackend
        backend_factory=GazeboBackend
    if observer_factory is None:
        from .observer import NavigationObserver
        observer_factory=NavigationObserver
    old_term=None
    try:
        if args.command=='find-run':
            print(find_active_report(args.plan,ROOT/'logs',ROOT))
            return 0
        if args.command=='probe':
            if not args.live:
                print(json.dumps({'dry_run':True,'action':'No Gazebo node constructed; use probe --live for read-only discovery'}))
                return 0
            if bool(args.plan)!=bool(args.run_report):raise ObstacleError('PROBE_PLAN_AND_REPORT_REQUIRED_TOGETHER')
            backend=backend_factory()
            bindings=backend.discover();entities=backend.entities()
            matches=False
            if args.plan:
                from .lifecycle import verify_live_base
                context=load_context(args.plan,args.run_report,ROOT)
                verify_live_base(context,backend);matches=True
            print(json.dumps({'read_only':True,'bindings':bindings,
                'live_static_scene_matches_formal_assets':matches,
                'test_obstacles':[e for name,e in entities.items() if name.startswith('ea_stage61_')]},indent=2))
            return 0
        if args.command=='cleanup':
            record=strict_json_loads(args.journal.read_text());validate_record(record)
            if not args.apply:
                print(json.dumps({'dry_run':True,'entity_name':record['entity_name'],
                    'entity_id':record.get('entity_id'),'last_cleanup':record.get('cleanup'),
                    'action':'Would verify server session, exact entity id and ownership geometry before removal'},indent=2))
                return 0
            def interrupted(signum,frame):raise KeyboardInterrupt
            old_term=signal.signal(signal.SIGTERM,interrupted)
            with exclusive_test_lock():
                try:
                    backend=backend_factory();clean=cleanup(backend,record)
                except BaseException as exc:
                    clean=False;record.update(cleanup='UNKNOWN',cleanup_error=type(exc).__name__)
                atomic_write_json(args.journal,record)
            print(json.dumps({'cleanup':record['cleanup'],'entity_id':record.get('entity_id'),
                              'error':record.get('cleanup_error')},indent=2))
            return 0 if clean else 4
        context=load_context(args.plan,args.run_report,ROOT)
        import math
        if args.target_step is not None and not 1<=args.target_step<=len(context['plan']['steps']):
            raise ObstacleError('INVALID_TARGET_STEP')
        if not math.isfinite(args.hold_seconds) or not 0<args.hold_seconds<=180:
            raise ObstacleError('HOLD_MUST_BE_0_TO_180_SECONDS')
        if not args.apply:
            print(json.dumps({'dry_run':True,'plan_id':context['plan_id'],'run_id':context['run_id'],
                'provenance_id':context['provenance_id'],'mode':args.mode,'hold_seconds':args.hold_seconds,'target_step':args.target_step,
                'site_profile':site_profile(context['bundle'],args.mode),
                'action':'No live path selected, no services called, no files written'},ensure_ascii=False,indent=2))
            return 0
        def interrupted(signum,frame):raise KeyboardInterrupt
        old_term=signal.signal(signal.SIGTERM,interrupted)
        with exclusive_test_lock():
            record=new_record(context,args.mode)
            record['target_step']=args.target_step
            path=args.journal or ROOT/'logs/reliability'/('lease_'+record['lease_id']+'.json')
            path.parent.mkdir(parents=True,exist_ok=True)
            descriptor=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
            os.close(descriptor)
            atomic_write_json(path,{**record,'create_sent':False,'status':'PREPARING','cleanup':'NOT_NEEDED'})
            backend=backend_factory()
            code,result=run_lease(context,backend,observer_factory,record,path,hold_seconds=args.hold_seconds)
        print(json.dumps({'journal':str(path),'status':result['status'],'cleanup':result['cleanup'],
            'entity_id':result.get('entity_id'),'error':result.get('error'),
            'cleanup_error':result.get('cleanup_error')},ensure_ascii=False,indent=2))
        return code
    except (Exception,KeyboardInterrupt) as exc:
        code=str(exc) if isinstance(exc,ObstacleError) else 'INTERRUPTED' if isinstance(exc,KeyboardInterrupt) else type(exc).__name__
        print(json.dumps({'error':code,'navigation_started':False}),file=sys.stderr)
        return 130 if isinstance(exc,KeyboardInterrupt) else 1
    finally:
        if old_term is not None:signal.signal(signal.SIGTERM,old_term)


if __name__=='__main__':
    raise SystemExit(main())
