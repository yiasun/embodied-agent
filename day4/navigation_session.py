"""Bounded Action protocol state machine, independent of ROS imports.

wait(future, seconds), make_goal(x, y), and the Action client are injected.
A timed-out send may have reached the server: never retry it as an ordinary failure.
"""
import math


class NavigationSession:
    def __init__(self, client, wait, make_goal, feedback=None,
                 server_timeout=10.0, send_timeout=10.0,
                 cancel_timeout=5.0, terminal_timeout=10.0):
        for value in (server_timeout, send_timeout, cancel_timeout, terminal_timeout):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Action timeouts must be finite and positive')
        self.client, self.wait, self.make_goal = client, wait, make_goal
        self.feedback = feedback
        self.server_timeout, self.send_timeout = server_timeout, send_timeout
        self.cancel_timeout, self.terminal_timeout = cancel_timeout, terminal_timeout
        self.pending_send = self.handle = self.result_future = None
        self.uncertain = False
        self.last_status, self.last_reason = 'PENDING', 'not_started'

    def _state(self, status, reason):
        self.last_status, self.last_reason = status, reason
        return status == 'SUCCEEDED'

    def _unknown(self, reason):
        self.uncertain = True
        return self._state('UNKNOWN', reason)

    def _terminal(self, response, timed_out=False):
        status = getattr(response, 'status', None)
        # action_msgs/GoalStatus: SUCCEEDED=4, CANCELED=5, ABORTED=6.
        # All other states leave termination unconfirmed.
        if status not in (4, 5, 6):
            return self._unknown('nonterminal_or_missing_result')
        self.handle = self.result_future = self.pending_send = None
        if status == 4:
            return self._state('SUCCEEDED', 'goal_reached')
        if status == 5:
            return self._state('TIMED_OUT' if timed_out else 'CANCELLED',
                               'timeout_cancel_confirmed' if timed_out else 'goal_cancelled')
        return self._state('FAILED', 'nav2_status_6')

    def abort_active(self):
        """Best effort, bounded cancellation; True only when no goal can remain active.

        Do not cancel unrelated goals. For an unresolved send we must first obtain
        its handle; if unavailable, the caller must retain UNKNOWN and halt.
        """
        try:
            if self.pending_send is not None:
                if not self.pending_send.done():
                    self.wait(self.pending_send, self.send_timeout)
                if not self.pending_send.done():
                    self._unknown('send_response_timeout')
                    return False
                handle = self.pending_send.result()
                self.pending_send = None
                if handle is None:
                    self._unknown('missing_goal_handle')
                    return False
                if not handle.accepted:
                    return True
                self.handle = handle
            if self.handle is None:
                return not self.uncertain
            if self.result_future is None:
                self.result_future = self.handle.get_result_async()
            if self.result_future.done():
                return self._consume_cleanup_result()
            cancel = self.handle.cancel_goal_async()
            self.wait(cancel, self.cancel_timeout)
            # Even when cancellation is refused/times out, a terminal result can
            # establish safety. Cancellation acknowledgement alone cannot.
            self.wait(self.result_future, self.terminal_timeout)
            if not self.result_future.done():
                self._unknown('cancel_response_timeout' if not cancel.done()
                              else 'cancel_result_timeout')
                return False
            return self._consume_cleanup_result()
        except (Exception, KeyboardInterrupt, SystemExit):
            self._unknown('cancel_exception')
            return False

    def _consume_cleanup_result(self):
        response = self.result_future.result()
        if getattr(response, 'status', None) not in (4, 5, 6):
            self._unknown('cancel_nonterminal_result')
            return False
        self.handle = self.result_future = self.pending_send = None
        return True

    def navigate(self, x, y, timeout_sec=120.0):
        if self.uncertain or self.handle is not None or self.pending_send is not None:
            return self._unknown('previous_goal_state_unknown')
        self._state('FAILED', 'navigation_failed')
        if any(type(v) not in (float, int) or not math.isfinite(v) for v in (x, y, timeout_sec)) or timeout_sec <= 0:
            raise ValueError('Coordinates/timeouts must be finite; timeout must be positive')
        try:
            if not self.client.wait_for_server(timeout_sec=self.server_timeout):
                return self._state('FAILED', 'server_unavailable')
            goal = self.make_goal(x, y)
            # If send_goal_async raises, transmission may still have occurred.
            self.uncertain = True
            self.pending_send = self.client.send_goal_async(goal, feedback_callback=self.feedback)
            self.wait(self.pending_send, self.send_timeout)
            if not self.pending_send.done():
                return self._unknown('send_response_timeout')
            self.handle = self.pending_send.result()
            self.pending_send = None
            if self.handle is None:
                return self._unknown('missing_goal_handle')
            if not self.handle.accepted:
                self.handle = None
                self.uncertain = False
                return self._state('REJECTED', 'goal_rejected')
            self.uncertain = False
            self.result_future = self.handle.get_result_async()
            self.wait(self.result_future, timeout_sec)
            if self.result_future.done():
                return self._terminal(self.result_future.result())
            cancel = self.handle.cancel_goal_async()
            self.wait(cancel, self.cancel_timeout)
            self.wait(self.result_future, self.terminal_timeout)
            if not self.result_future.done():
                return self._unknown('cancel_response_timeout' if not cancel.done()
                                     else 'cancel_result_timeout')
            return self._terminal(self.result_future.result(), timed_out=True)
        except (KeyboardInterrupt, SystemExit):
            confirmed = self.abort_active()
            self._state('INTERRUPTED' if confirmed else 'UNKNOWN',
                        'interrupt_cancel_confirmed' if confirmed else 'interrupt_state_unknown')
            raise
        except Exception as exc:
            confirmed = self.abort_active()
            return self._state('FAILED' if confirmed else 'UNKNOWN',
                               f'exception_{type(exc).__name__}' if confirmed else 'exception_state_unknown')
