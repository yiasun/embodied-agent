from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from day4.navigation_session import NavigationSession


class Future:
    def __init__(self, result=None, done=True, error=None):
        self.value, self.ready, self.error = result, done, error
    def done(self):
        return self.ready
    def result(self):
        if self.error:
            raise self.error
        return self.value


def setup_session(status=4, result_done=True, send_done=True, accepted=True, server=True,
                  cancel_done=True, cancel_accept=True, events=None):
    result = Future(SimpleNamespace(status=status), result_done)
    cancel = Future(SimpleNamespace(goals_canceling=[1] if cancel_accept else []), cancel_done)
    handle = SimpleNamespace(accepted=accepted, get_result_async=Mock(return_value=result),
                             cancel_goal_async=Mock(return_value=cancel))
    send = Future(handle, send_done)
    client = SimpleNamespace(wait_for_server=Mock(return_value=server), send_goal_async=Mock(return_value=send))
    events = events if events is not None else []
    def wait(future, seconds):
        assert seconds > 0
        if events:
            event = events.pop(0)
            if isinstance(event, BaseException):
                raise event
            if event is not None:
                future.value, future.ready = SimpleNamespace(status=event), True
    session = NavigationSession(client, wait, lambda x,y:(x,y))
    return session, client, handle, send, result, cancel


@pytest.mark.parametrize('status,expected,success', [(4,'SUCCEEDED',True), (5,'CANCELLED',False),
                                                    (6,'FAILED',False), (2,'UNKNOWN',False), (None,'UNKNOWN',False)])
def test_terminal_states(status, expected, success):
    session, *_ = setup_session(status)
    assert session.navigate(0,0) is success
    assert session.last_status == expected


def test_server_timeout():
    session, client, *_ = setup_session(server=False)
    assert not session.navigate(0,0)
    assert session.last_reason == 'server_unavailable'
    client.wait_for_server.assert_called_once_with(timeout_sec=10.0)
    client.send_goal_async.assert_not_called()


def test_rejected():
    session, _, handle, *_ = setup_session(accepted=False)
    assert not session.navigate(0,0)
    assert (session.last_status,session.last_reason) == ('REJECTED','goal_rejected')
    handle.cancel_goal_async.assert_not_called()


def test_send_timeout_latches_unknown_and_never_resends():
    session, client, *_ = setup_session(send_done=False)
    assert not session.navigate(0,0)
    assert (session.last_status,session.last_reason) == ('UNKNOWN','send_response_timeout')
    assert not session.abort_active()
    assert not session.navigate(1,1)
    assert client.send_goal_async.call_count == 1


def test_late_send_response_cancelled_without_retry():
    session, client, handle, send, result, _ = setup_session(send_done=False, result_done=False)
    assert not session.navigate(0,0)
    send.ready = True
    session.wait = lambda future, seconds: setattr(result, 'ready', True)
    result.value = SimpleNamespace(status=5)
    assert session.abort_active()
    handle.cancel_goal_async.assert_called_once()
    assert session.last_status == 'UNKNOWN'
    assert not session.navigate(1,1)
    assert client.send_goal_async.call_count == 1


@pytest.mark.parametrize('cancel_done,cancel_accept', [(True,True),(True,False),(False,False)])
def test_timeout_requires_terminal_result(cancel_done,cancel_accept):
    session, _, handle, _, result, cancel = setup_session(
        result_done=False,cancel_done=cancel_done,cancel_accept=cancel_accept)
    def wait(future, seconds):
        if future is result and handle.cancel_goal_async.called:
            result.ready=True;result.value=SimpleNamespace(status=5)
    session.wait=wait
    assert not session.navigate(0,0,timeout_sec=2)
    assert (session.last_status,session.last_reason) == ('TIMED_OUT','timeout_cancel_confirmed')


@pytest.mark.parametrize('cancel_done,reason', [(True,'cancel_result_timeout'),(False,'cancel_response_timeout')])
def test_cancel_without_terminal_is_unknown(cancel_done,reason):
    session, *_ = setup_session(result_done=False,cancel_done=cancel_done)
    assert not session.navigate(0,0,2)
    assert (session.last_status,session.last_reason) == ('UNKNOWN',reason)


def test_success_racing_cancel():
    session, _, handle, _, result, _=setup_session(result_done=False)
    def wait(future, seconds):
        if future is result and handle.cancel_goal_async.called:
            result.ready=True
    session.wait=wait
    assert session.navigate(0,0,2)
    assert session.last_status=='SUCCEEDED'


@pytest.mark.parametrize('exception', [RuntimeError('failure'),KeyboardInterrupt()])
@pytest.mark.parametrize('confirmed', [True,False])
def test_exception_or_interrupt_cancels_active(exception,confirmed):
    session, _, handle, send, result, _=setup_session(result_done=False)
    raised=False
    def wait(future, seconds):
        nonlocal raised
        if future is result and not raised:
            raised=True
            raise exception
        if future is result and confirmed:
            result.ready=True;result.value=SimpleNamespace(status=5)
    session.wait=wait
    if isinstance(exception,KeyboardInterrupt):
        with pytest.raises(KeyboardInterrupt):session.navigate(0,0)
    else:
        assert not session.navigate(0,0)
    assert handle.cancel_goal_async.called
    assert session.last_status == (('INTERRUPTED' if isinstance(exception,KeyboardInterrupt) else 'FAILED') if confirmed else 'UNKNOWN')


def test_send_exception_is_unknown():
    session,client,*_=setup_session()
    client.send_goal_async.side_effect=RuntimeError('transport failure')
    assert not session.navigate(0,0)
    assert session.last_status=='UNKNOWN'


def test_cancel_exception_is_unknown():
    session,_,handle,*_=setup_session(result_done=False)
    handle.cancel_goal_async.side_effect=RuntimeError('cancel failure')
    assert not session.navigate(0,0)
    assert session.last_status=='UNKNOWN'


@pytest.mark.parametrize('x,y,timeout', [(float('nan'),0,1),(0,float('inf'),1),(0,0,float('nan')),(0,0,0),(True,0,1)])
def test_invalid_arguments_never_send(x,y,timeout):
    session,client,*_=setup_session()
    with pytest.raises(ValueError):session.navigate(x,y,timeout)
    client.send_goal_async.assert_not_called()


@pytest.mark.parametrize('error',[RuntimeError('send future failure'),KeyboardInterrupt()])
def test_exception_during_send_wait_is_unknown(error):
    session,client,_,send,*_=setup_session(send_done=False)
    def wait(future,seconds):raise error
    session.wait=wait
    if isinstance(error,KeyboardInterrupt):
        with pytest.raises(KeyboardInterrupt):session.navigate(0,0)
    else:
        assert not session.navigate(0,0)
    assert session.last_status=='UNKNOWN'
    assert client.send_goal_async.call_count==1


def test_terminal_future_exception_does_not_claim_stopped():
    session,_,handle,_,result,_=setup_session()
    result.error=RuntimeError('result unavailable')
    assert not session.navigate(0,0)
    assert session.last_status=='UNKNOWN'


def test_missing_goal_handle_is_unknown():
    session,_,_,send,*_=setup_session()
    send.value=None
    assert not session.navigate(0,0)
    assert session.last_status=='UNKNOWN'


def test_system_exit_during_navigation_cancels_goal():
    session,_,handle,_,result,_=setup_session(result_done=False)
    raised=False
    def wait(future,seconds):
        nonlocal raised
        if future is result and not raised:
            raised=True;raise SystemExit(0)
        if future is result:
            result.ready=True;result.value=SimpleNamespace(status=5)
    session.wait=wait
    with pytest.raises(SystemExit):session.navigate(0,0)
    handle.cancel_goal_async.assert_called_once()
    assert session.last_status=='INTERRUPTED'
