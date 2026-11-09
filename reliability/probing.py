"""Read-only metadata process: isolates native SDK GIL/transport locks from heartbeat."""
import multiprocessing
import os
import re
import threading
import time

from .safety import ObstacleError


def _serve(connection,world,robot):
    # Gazebo native diagnostics belong to the child; never dump arbitrary stderr/env.
    with open(os.devnull,'w') as sink:
        os.dup2(sink.fileno(),1);os.dup2(sink.fileno(),2)
    try:
        from .gazebo import GazeboBackend
        backend=GazeboBackend(world,robot)
        connection.send({'ready':True})
        while True:
            command=connection.recv()
            if command=='close':return
            if command!='probe':raise ObstacleError('INVALID_SESSION_PROBE_COMMAND')
            try:connection.send({'session_id':backend.discover()['session_id']})
            except Exception as exc:
                connection.send({'error':str(exc) if isinstance(exc,ObstacleError) else type(exc).__name__})
    except (EOFError,BrokenPipeError):pass
    except Exception as exc:
        try:connection.send({'error':type(exc).__name__})
        except (OSError,EOFError):pass
    finally:connection.close()


class SessionProbe:
    def __init__(self,world,robot,*,context=None,target=_serve,timeout=0.9):
        self.context=context or multiprocessing.get_context('spawn')
        self.connection,child=self.context.Pipe();self.timeout=timeout;self.closed=False;self.ready=False
        self.lock=threading.Lock()
        self.process=self.context.Process(target=target,args=(child,world,robot),daemon=True,name='ea_readonly_session_probe')
        try:self.process.start()
        except BaseException:
            self.connection.close();child.close();raise
        child.close()

    def _query(self,timeout):
        with self.lock:
            if self.closed:raise ObstacleError('SESSION_PROBE_CLOSED')
            deadline=time.monotonic()+timeout
            if not self.ready:
                if not self.connection.poll(max(0,deadline-time.monotonic())):raise ObstacleError('SESSION_PROBE_START_TIMED_OUT')
                if self.connection.recv()!={'ready':True}:raise ObstacleError('SESSION_PROBE_START_FAILED')
                self.ready=True
            self.connection.send('probe')
            if not self.connection.poll(max(0,deadline-time.monotonic())):raise ObstacleError('SESSION_PROBE_QUERY_TIMED_OUT')
            result=self.connection.recv()
            if time.monotonic()>deadline:raise ObstacleError('SESSION_PROBE_QUERY_TIMED_OUT')
            if not isinstance(result,dict):raise ObstacleError('INVALID_SESSION_PROBE_RESPONSE')
            if 'error' in result:raise ObstacleError('SESSION_PROBE_FAILED')
            session=result.get('session_id')
            if not isinstance(session,str) or not re.fullmatch('[0-9a-f]{64}',session):
                raise ObstacleError('INVALID_SESSION_PROBE_RESPONSE')
            return {'session_id':session}

    def startup_check(self):return self._query(3.0) # before the first heartbeat
    def __call__(self):return self._query(self.timeout)

    def close(self):
        if self.closed:return
        self.closed=True
        # Only our own read-only helper. No signal is ever sent to Gazebo/Nav2/V4.
        if self.process.is_alive():self.process.terminate()
        self.process.join(timeout=0.1)
        if self.process.is_alive():self.process.kill();self.process.join(timeout=0.1)
        self.connection.close()
        if self.process.is_alive():raise ObstacleError('SESSION_PROBE_PROCESS_STOP_UNKNOWN')
