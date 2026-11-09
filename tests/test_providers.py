"""Provider tests use synthetic credentials and in-memory HTTP responses only."""
import json
import traceback
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from planner.planner_v1 import load_scene
from planner.providers import PlanningError, create_provider
from planner.providers.ollama import OllamaProvider
from planner.providers.qwen import QwenProvider
from planner.providers.rule import RuleProvider
from planner.providers.transport import HTTPResponse, MAX_RESPONSE_BYTES

FAKE_KEY = 'synthetic-unit-test-credential'
INTENT = {'steps': [{'action': 'NAVIGATE', 'location': 'A区'}, {'action': 'NAVIGATE', 'location': 'B区'}]}


def response(content=None, **updates):
    data = {'id': 'chat-test-001', 'model': 'reported-model',
            'choices': [{'finish_reason': 'stop', 'message': {
                'role': 'assistant', 'content': json.dumps(INTENT if content is None else content, ensure_ascii=False)}}],
            'usage': {'prompt_tokens': 31, 'completion_tokens': 17, 'total_tokens': 48}}
    data.update(updates)
    return HTTPResponse(200, json.dumps(data, ensure_ascii=False).encode(), {})


@pytest.fixture
def scene():
    return load_scene('turtlebot3_demo')


def provider_for(name='qwen', answer=None, **env_updates):
    env = {'LLM_PROVIDER': name, 'QWEN_API_KEY': FAKE_KEY, 'OLLAMA_MODEL': 'installed-local-model:test'}
    env.update(env_updates)
    transport = Mock()
    transport.post.return_value = response() if answer is None else answer
    return create_provider(env=env, transport=transport), transport


@pytest.mark.parametrize('name,url,model', [
    ('qwen', 'https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions', 'qwen-plus'),
    ('ollama', 'http://localhost:11434/v1/chat/completions', 'installed-local-model:test'),
])
def test_request_and_response_contract(scene, name, url, model):
    provider, transport = provider_for(name)
    result = provider.generate_intent('先去A区，再去B区', scene)
    transport.post.assert_called_once()
    args, kwargs = transport.post.call_args
    assert args == (url,)
    assert kwargs['timeout'] == 30
    headers = kwargs['headers']
    assert headers['Content-Type'] == 'application/json'
    if name == 'qwen':
        assert headers['Authorization'] == 'Bearer ' + FAKE_KEY
    else:
        assert 'Authorization' not in headers
    body = json.loads(kwargs['body'])
    assert body['model'] == model
    assert body['stream'] is False
    assert body['response_format'] == {'type': 'json_object'}
    assert body['max_tokens'] == 2048
    prompt = json.loads(body['messages'][1]['content'])
    assert prompt == {'instruction': '先去A区，再去B区', 'allowed_locations': ['A区', 'B区']}
    assert '0.419' not in kwargs['body'].decode() and '-1.822' not in kwargs['body'].decode()
    assert FAKE_KEY not in kwargs['body'].decode()
    assert json.loads(result.content) == INTENT
    assert result.provider == name and result.model == 'reported-model'
    assert result.requested_model == model and result.request_id == 'chat-test-001'
    assert (result.input_tokens, result.output_tokens, result.total_tokens) == (31, 17, 48)
    assert result.duration_seconds >= 0
    assert result.raw_response['choices']


@pytest.mark.parametrize('name,klass', [('rule', RuleProvider), ('qwen', QwenProvider), ('ollama', OllamaProvider)])
def test_explicit_switch(name, klass):
    provider, _ = provider_for(name)
    assert isinstance(provider, klass)


def test_default_rule_does_not_read_cloud_configuration(scene):
    class OnlySelection(dict):
        def get(self, key, default=None):
            assert key == 'LLM_PROVIDER'
            return super().get(key, default)
    transport = Mock(side_effect=AssertionError('no network'))
    provider = create_provider(env=OnlySelection(), transport=transport)
    result = provider.generate_intent('去A区', scene)
    transport.post.assert_not_called()
    assert result.provider == 'rule' and result.request_id is None
    assert result.input_tokens is result.output_tokens is result.total_tokens is None


@pytest.mark.parametrize('name', ['', 'QWEN', 'deepseek', 'auto', ' qwen'])
def test_unknown_provider_no_fallback(name):
    with pytest.raises(PlanningError, match='UNKNOWN_PROVIDER'):
        create_provider(env={'LLM_PROVIDER': name})


@pytest.mark.parametrize('key', [None, '', '   '])
def test_missing_key_never_calls_transport(key):
    transport = Mock()
    with pytest.raises(PlanningError, match='MISSING_API_KEY'):
        create_provider(env={'LLM_PROVIDER': 'qwen', 'QWEN_API_KEY': key}, transport=transport)
    transport.post.assert_not_called()


@pytest.mark.parametrize('key', ['bad\nheader', ' spaced ', '中文'])
def test_invalid_key_format(key):
    with pytest.raises(PlanningError, match='INVALID_API_KEY_FORMAT'):
        provider_for(QWEN_API_KEY=key)


@pytest.mark.parametrize('value', ['', None])
def test_ollama_model_required(value):
    with pytest.raises(PlanningError, match='MISSING_OLLAMA_MODEL'):
        provider_for('ollama', OLLAMA_MODEL=value)


@pytest.mark.parametrize('model', ['model:cloud', 'model:120b-cloud'])
def test_no_cloud_ollama_tags(model):
    with pytest.raises(PlanningError, match='CLOUD_MODEL_NOT_ALLOWED'):
        provider_for('ollama', OLLAMA_MODEL=model)


def test_ollama_never_reads_qwen_key(scene):
    class LocalEnv(dict):
        def get(self, key, default=None):
            assert not key.startswith('QWEN')
            return super().get(key, default)
    env = LocalEnv(LLM_PROVIDER='ollama', OLLAMA_MODEL='local:test')
    transport = Mock(); transport.post.return_value = response()
    provider = create_provider(env=env, transport=transport)
    provider.generate_intent('去A区', scene)
    assert 'Authorization' not in transport.post.call_args.kwargs['headers']


@pytest.mark.parametrize('name,variable,url,expected', [
    ('qwen','QWEN_BASE_URL','https://dashscope-intl.aliyuncs.com/compatible-mode/v1/',
     'https://dashscope-intl.aliyuncs.com/compatible-mode/v1/chat/completions'),
    ('qwen','QWEN_BASE_URL','https://workspace.cn-beijing.maas.aliyuncs.com/compatible-mode/v1',
     'https://workspace.cn-beijing.maas.aliyuncs.com/compatible-mode/v1/chat/completions'),
    ('ollama','OLLAMA_BASE_URL','http://127.0.0.1:11434/v1/', 'http://127.0.0.1:11434/v1/chat/completions'),
    ('ollama','OLLAMA_BASE_URL','http://172.20.16.1:11434/v1', 'http://172.20.16.1:11434/v1/chat/completions'),
    ('ollama','OLLAMA_BASE_URL','http://[::1]:11434/v1', 'http://[::1]:11434/v1/chat/completions'),
])
def test_configurable_region_and_local_urls(scene, name, variable, url, expected):
    provider, transport = provider_for(name, **{variable: url})
    provider.generate_intent('去A区', scene)
    assert transport.post.call_args.args == (expected,)


@pytest.mark.parametrize('name,variable,url', [
    ('qwen','QWEN_BASE_URL','http://example.com/v1'),
    ('qwen','QWEN_BASE_URL','https://user:secret@example.com/v1'),
    ('qwen','QWEN_BASE_URL','https://example.com/v1?key=secret'),
    ('qwen','QWEN_BASE_URL','https://example.com/v1#secret'),
    ('qwen','QWEN_BASE_URL','https://example.com:bad/v1'),
    ('qwen','QWEN_BASE_URL','https://example.com/\nv1'),
    ('ollama','OLLAMA_BASE_URL','https://ollama.com/v1'),
    ('ollama','OLLAMA_BASE_URL','http://8.8.8.8/v1'),
    ('ollama','OLLAMA_BASE_URL','file:///tmp/v1'),
])
def test_bad_endpoints(name, variable, url):
    with pytest.raises(PlanningError, match='INVALID_BASE_URL'):
        provider_for(name, **{variable: url})


@pytest.mark.parametrize('timeout', ['nan', 'inf', '0', '-1', '301', 'bad'])
def test_invalid_timeout(timeout):
    with pytest.raises(PlanningError, match='INVALID_TIMEOUT'):
        provider_for(LLM_TIMEOUT_SECONDS=timeout)


@pytest.mark.parametrize('usage', [None, {}, {'prompt_tokens': 0}, {'completion_tokens': 5}])
def test_absent_usage_stays_null(scene, usage):
    provider, _ = provider_for(answer=response(usage=usage))
    result = provider.generate_intent('去A区', scene)
    assert result.input_tokens == (usage or {}).get('prompt_tokens')
    assert result.output_tokens == (usage or {}).get('completion_tokens')
    assert result.total_tokens is None


@pytest.mark.parametrize('usage', [{'prompt_tokens': True}, {'completion_tokens': -1}, {'total_tokens': '5'}, []])
def test_malformed_usage_rejected(scene, usage):
    provider, _ = provider_for(answer=response(usage=usage))
    with pytest.raises(PlanningError, match='INVALID_RESPONSE'):
        provider.generate_intent('去A区', scene)


@pytest.mark.parametrize('status,error,expected', [
    (401, {}, 'AUTHENTICATION_ERROR'), (403, {}, 'PERMISSION_DENIED'),
    (429, {}, 'RATE_LIMITED'), (429, {'code': 'insufficient_quota'}, 'QUOTA_EXHAUSTED'),
    (403, {'code': 'Arrearage'}, 'QUOTA_EXHAUSTED'),
    (400, {'code': 'AllocationQuota.FreeTierOnly'}, 'QUOTA_EXHAUSTED'),
    (500, {}, 'SERVER_ERROR'), (503, {}, 'SERVER_ERROR'), (404, {}, 'HTTP_ERROR'),
    (302, {}, 'HTTP_ERROR'),
])
def test_http_failures_do_not_retry_or_fallback(scene, status, error, expected):
    error['message'] = FAKE_KEY
    answer = HTTPResponse(status, json.dumps({'error': error}).encode())
    provider, transport = provider_for(answer=answer)
    with pytest.raises(PlanningError) as caught:
        provider.generate_intent('去A区', scene)
    assert caught.value.code == expected and caught.value.http_status == status
    assert FAKE_KEY not in ''.join(traceback.format_exception(caught.value))
    transport.post.assert_called_once()


@pytest.mark.parametrize('error,expected', [(TimeoutError(FAKE_KEY), 'REQUEST_TIMEOUT'),
    (OSError(FAKE_KEY), 'NETWORK_ERROR'), (RuntimeError(FAKE_KEY), 'NETWORK_ERROR'),
    (PlanningError('untrusted-' + FAKE_KEY), 'NETWORK_ERROR')])
def test_transport_errors_redacted_and_not_retried(scene, error, expected):
    provider, transport = provider_for()
    transport.post.side_effect = error
    with pytest.raises(PlanningError) as caught:
        provider.generate_intent('去A区', scene)
    assert caught.value.code == expected
    assert FAKE_KEY not in ''.join(traceback.format_exception(caught.value))
    transport.post.assert_called_once()


@pytest.mark.parametrize('body', [b'{', b'not json', b'[]', b'null', b'{"choices":[]}',
                                      b'{"choices":null}', b'\xff', b'{"a":1,"a":2}'])
def test_invalid_envelope(scene, body):
    provider, transport = provider_for(answer=HTTPResponse(200, body))
    with pytest.raises(PlanningError) as caught:
        provider.generate_intent('去A区', scene)
    assert caught.value.code in ('INVALID_JSON', 'INVALID_RESPONSE')
    transport.post.assert_called_once()


@pytest.mark.parametrize('choice,expected', [
    ({'finish_reason': 'length', 'message': {'content': '{}'}}, 'INCOMPLETE_RESPONSE'),
    ({'message': {'content': '{}'}}, 'INCOMPLETE_RESPONSE'),
    ({'finish_reason': 'stop', 'message': {'tool_calls': [1], 'content': '{}'}}, 'UNSUPPORTED_RESPONSE'),
    ({'finish_reason': 'stop', 'message': {'function_call': {'name': 'shell'}, 'content': '{}'}}, 'UNSUPPORTED_RESPONSE'),
    ({'finish_reason': 'stop', 'message': {'refusal': 'no', 'content': '{}'}}, 'UNSUPPORTED_RESPONSE'),
    ({'finish_reason': 'stop', 'message': {'content': None}}, 'INVALID_RESPONSE'),
])
def test_unsupported_response(scene, choice, expected):
    provider, _ = provider_for(answer=response(choices=[choice]))
    with pytest.raises(PlanningError, match=expected):
        provider.generate_intent('去A区', scene)


def test_request_id_header_and_reported_model_optional(scene):
    answer=response(id=None, model=None)
    answer=replace(answer, headers={'X-Request-Id': 'header-id'})
    provider, _=provider_for(answer=answer)
    result=provider.generate_intent('去A区',scene)
    assert result.request_id=='header-id' and result.model=='qwen-plus'


def test_raw_response_and_repr_cannot_leak_key(scene):
    answer=response(id=FAKE_KEY, model=FAKE_KEY, echoed={'Authorization': FAKE_KEY})
    provider, _=provider_for(answer=answer)
    result=provider.generate_intent('去A区',scene)
    assert FAKE_KEY not in repr(provider)
    assert FAKE_KEY not in repr(result)
    assert FAKE_KEY not in json.dumps(result.as_dict())
    assert '[REDACTED]' in json.dumps(result.raw_response)


def test_oversized_response(scene):
    provider, transport=provider_for(answer=HTTPResponse(200,b' '*(MAX_RESPONSE_BYTES+1)))
    with pytest.raises(PlanningError,match='RESPONSE_TOO_LARGE'):
        provider.generate_intent('去A区',scene)
    transport.post.assert_called_once()


@pytest.mark.parametrize('answer', [None, HTTPResponse(True,b'{}'), HTTPResponse(200,'not bytes'),
                                    HTTPResponse(200,b'{}',[]), HTTPResponse(999,b'{}')])
def test_invalid_transport_response_is_closed(scene,answer):
    provider,transport=provider_for()
    transport.post.return_value=answer
    with pytest.raises(PlanningError,match='INVALID_RESPONSE'):
        provider.generate_intent('去A区',scene)


@pytest.mark.parametrize('fields',[{'id':42},{'model':[]},{'usage':{'total_tokens':0.5}}])
def test_malformed_metadata_rejected(scene,fields):
    provider,_=provider_for(answer=response(**fields))
    with pytest.raises(PlanningError,match='INVALID_RESPONSE'):
        provider.generate_intent('去A区',scene)


def test_measured_request_duration(scene,monkeypatch):
    from planner.providers import compatible
    clock=Mock(side_effect=[10.0,10.125])
    monkeypatch.setattr(compatible.time,'monotonic',clock)
    provider,_=provider_for()
    assert provider.generate_intent('去A区',scene).duration_seconds==0.125
