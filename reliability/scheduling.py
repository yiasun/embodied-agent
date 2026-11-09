"""Bounded noncritical I/O. These workers never send heartbeats or robot commands."""
import copy
import threading
import time
from collections import deque
from pathlib import Path

from .safety import ObstacleError


class Timings:
    def __init__(self,clock=time.monotonic):
        self.clock=clock;self.lock=threading.Lock();self.stats={};self.events=deque(maxlen=512)

    def observe(self,name,seconds,outcome='OK'):
        with self.lock:
            row=self.stats.setdefault(name,{'count':0,'total_s':0.0,'max_s':None,'last_s':None,'ok':0,'failed':0})
            row.update(count=row['count']+1,total_s=row['total_s']+seconds,
                       max_s=max(row['max_s'] or 0,seconds),last_s=seconds)
            row['ok' if outcome=='OK' else 'failed']+=1
            self.events.append({'operation':name,'duration_wall_s':seconds,'outcome':outcome,'completed_mono':self.clock()})

    def call(self,name,function,*args,**kwargs):
        start=self.clock();outcome='OK'
        try:return function(*args,**kwargs)
        except BaseException as exc:
            outcome=type(exc).__name__;raise
        finally:self.observe(name,max(0,self.clock()-start),outcome)

    def snapshot(self):
        with self.lock:return {'time_basis':'monotonic_wall','statistics':copy.deepcopy(self.stats),'events':list(self.events)}


class AsyncTask:
    """One outstanding job. An expired job cannot be accepted or retried in flight."""
    def __init__(self,name,clock=time.monotonic):
        self.name=name;self.clock=clock;self.lock=threading.Lock();self.job=None;self.thread=None;self.closed=False

    def submit(self,function):
        with self.lock:
            if self.closed or self.job is not None:raise ObstacleError('IO_TASK_BUSY_OR_CLOSED')
            job={'started':self.clock(),'done':False,'result':None,'error':None}
            self.job=job
        def run():
            result=None;error=None
            try:result=function()
            except BaseException as exc:error=str(exc) if isinstance(exc,ObstacleError) else type(exc).__name__
            with self.lock:job.update(done=True,result=result,error=error,ended=self.clock())
        self.thread=threading.Thread(target=run,name='ea_'+self.name,daemon=True)
        try:self.thread.start()
        except BaseException:
            with self.lock:self.job=None;self.thread=None
            raise

    def take(self,limit):
        with self.lock:
            if self.job is None:return None
            job=self.job
            duration=(job.get('ended',self.clock())-job['started'])
            if duration>limit:raise ObstacleError(self.name.upper()+'_TIMED_OUT')
            if not job['done']:return None
            self.job=None
            if job['error']:raise ObstacleError(self.name.upper()+'_FAILED_'+job['error'])
            return dict(job)

    def close(self):
        with self.lock:self.closed=True
        if self.thread is not None:self.thread.join(timeout=0.05)
        with self.lock:
            return {'unfinished':bool(self.thread and self.thread.is_alive()),
                    'pending_age_wall_s':max(0,self.clock()-self.job['started']) if self.job else None}


def freeze_report(report):
    """Freeze append-only sample/path/event containers, without copying every path point.

    Capture appends new dictionaries and replaces execution snapshots; existing
    sample dictionaries and path point arrays are not modified after publication.
    """
    result=dict(report)
    for name in ('samples','paths','bt_events'):
        if name in result:result[name]=list(result[name])
    if 'quality' in result:
        result['quality']=dict(result['quality'])
        if 'issues' in result['quality']:result['quality']['issues']=list(result['quality']['issues'])
    return result


class RuntimeIO:
    SESSION_PERIOD=0.2
    DISCOVERY_LIMIT=1.0
    SESSION_TTL=2.5
    PERSIST_LIMIT=2.0

    def __init__(self,probe,expected_session,verified_at,timings,clock=time.monotonic):
        self.probe=probe;self.expected=expected_session;self.clock=clock;self.timings=timings
        self.verified_at=verified_at;self.last_probe_finish=clock()
        self.discovery=AsyncTask('discovery',clock);self.persistence=AsyncTask('persistence',clock)
        self.cancel=threading.Event();self.generation=0;self.skipped_progress=0;self.shutdown=None

    def poll(self,*,launch=True):
        completed=self.discovery.take(self.DISCOVERY_LIMIT)
        if completed:
            if completed['result']['session_id']!=self.expected:raise ObstacleError('WORLD_SESSION_CHANGED_DURING_MOTION')
            # Use query start, not end, as conservative age of the observation.
            self.verified_at=completed['started'];self.last_probe_finish=completed['ended']
        self.persistence.take(self.PERSIST_LIMIT)
        if self.clock()-self.verified_at>self.SESSION_TTL:raise ObstacleError('WORLD_SESSION_PROOF_STALE')
        if launch and self.discovery.job is None and self.clock()-self.last_probe_finish>=self.SESSION_PERIOD:
            self.discovery.submit(lambda:self.timings.call('gazebo_discovery',self.probe))

    def queue_progress(self,path,record,report,writer,checkpoint):
        if self.persistence.job is not None:
            self.skipped_progress+=1;return False # bounded: never queue unbounded snapshots
        self.generation+=1
        journal=copy.deepcopy(record);metrics=freeze_report(report)
        mark={'progress_only':True,'generation':self.generation,'snapshot_mono':self.clock()}
        journal['progress']=dict(mark);metrics['progress']=dict(mark)
        progress_path=Path(str(path)+'.progress.json')
        def persist():
            if self.cancel.is_set():return
            self.timings.call('json_writer',writer,progress_path,journal)
            if checkpoint and not self.cancel.is_set():
                self.timings.call('checkpoint',checkpoint,metrics,progress=True)
        self.persistence.submit(persist);return True

    def close(self):
        self.cancel.set()
        final_error=None
        try:self.poll(launch=False)
        except Exception as exc:final_error=str(exc) if isinstance(exc,ObstacleError) else type(exc).__name__
        self.shutdown={'discovery':self.discovery.close(),'persistence':self.persistence.close(),'final_error':final_error}
        if final_error is None:
            try:self.poll(launch=False)
            except Exception as exc:
                self.shutdown['final_error']=str(exc) if isinstance(exc,ObstacleError) else type(exc).__name__
        if hasattr(self.probe,'close'):
            try:self.probe.close()
            except Exception as exc:
                self.shutdown['probe_close_error']=type(exc).__name__
                if self.shutdown['final_error'] is None:self.shutdown['final_error']='SESSION_PROBE_SHUTDOWN_FAILED'
        return self.shutdown

    def diagnostics(self):
        return {'session_ttl_wall_s':self.SESSION_TTL,'discovery_deadline_wall_s':self.DISCOVERY_LIMIT,
                'persistence_deadline_wall_s':self.PERSIST_LIMIT,
                'session_proof_age_wall_s':max(0,self.clock()-self.verified_at),
                'shutdown':self.shutdown,'progress_generations':self.generation,'progress_snapshots_skipped':self.skipped_progress,
                'background_heartbeat_sender':False}
