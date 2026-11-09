import io
import socket
from unittest.mock import Mock
from urllib.error import HTTPError, URLError

import pytest

from planner.providers import PlanningError
from planner.providers import transport


@pytest.mark.parametrize('error,expected',[(TimeoutError('synthetic-key'),'REQUEST_TIMEOUT'),
    (socket.timeout('synthetic-key'),'REQUEST_TIMEOUT'),
    (URLError(TimeoutError('synthetic-key')),'REQUEST_TIMEOUT'),
    (URLError('synthetic-key'),'NETWORK_ERROR'),(OSError('synthetic-key'),'NETWORK_ERROR')])
def test_transport_errors_sanitized_once(monkeypatch,error,expected):
    opener=Mock();opener.open.side_effect=error
    monkeypatch.setattr(transport,'build_opener',Mock(return_value=opener))
    with pytest.raises(PlanningError,match=expected) as caught:
        transport.UrllibTransport().post('https://example.invalid/v1',headers={},body=b'{}',timeout=2)
    assert 'synthetic-key' not in str(caught.value)
    opener.open.assert_called_once()


def test_actual_request_construction_with_fake_opener(monkeypatch):
    stream=Mock();stream.status=200;stream.headers={'X-Request-Id':'test'};stream.read.return_value=b'{}'
    opener=Mock();opener.open.return_value.__enter__=Mock(return_value=stream)
    opener.open.return_value.__exit__=Mock(return_value=False)
    build=Mock(return_value=opener)
    monkeypatch.setattr(transport,'build_opener',build)
    answer=transport.UrllibTransport().post('https://example.invalid/v1/chat/completions',
        headers={'Authorization':'Bearer synthetic-key','Content-Type':'application/json'},body=b'{}',timeout=5)
    request=opener.open.call_args.args[0]
    assert request.get_method()=='POST' and request.data==b'{}'
    assert request.get_header('Authorization')=='Bearer synthetic-key'
    assert answer.status==200 and answer.body==b'{}'
    assert any(isinstance(h,transport.NoRedirect) for h in build.call_args.args)
    proxies=[h for h in build.call_args.args if isinstance(h,transport.ProxyHandler)]
    assert len(proxies)==1 and proxies[0].proxies=={}
    opener.open.assert_called_once_with(request,timeout=5)


def test_http_error_body_is_returned_not_logged(monkeypatch):
    error=HTTPError('https://example.invalid',429,'synthetic-key',{},io.BytesIO(b'{"error":{}}'))
    opener=Mock();opener.open.side_effect=error
    monkeypatch.setattr(transport,'build_opener',Mock(return_value=opener))
    answer=transport.UrllibTransport().post('https://example.invalid/v1',headers={},body=b'{}',timeout=5)
    assert answer.status==429 and answer.body==b'{"error":{}}'
    opener.open.assert_called_once()


def test_redirect_is_not_followed():
    assert transport.NoRedirect().redirect_request(None,None,302,'redirect',{},'https://other.invalid') is None


def test_transport_response_size_bound(monkeypatch):
    stream=Mock();stream.read.return_value=b' '*(transport.MAX_RESPONSE_BYTES+1)
    opener=Mock();opener.open.return_value.__enter__=Mock(return_value=stream)
    opener.open.return_value.__exit__=Mock(return_value=False)
    monkeypatch.setattr(transport,'build_opener',Mock(return_value=opener))
    with pytest.raises(PlanningError,match='RESPONSE_TOO_LARGE'):
        transport.UrllibTransport().post('https://example.invalid/v1',headers={},body=b'{}',timeout=5)
