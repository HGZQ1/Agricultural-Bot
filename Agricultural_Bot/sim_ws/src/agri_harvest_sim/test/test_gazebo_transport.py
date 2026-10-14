import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

PATH = Path(__file__).parents[1]/'scripts/gazebo_transport.py'
spec = importlib.util.spec_from_file_location('tested_gazebo_transport', PATH)
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)


class Message:
    def __init__(self):
        self.data = ''


def test_persistent_endpoint_reuses_node_and_sets_partition_before_creation(monkeypatch):
    nodes, calls = [], []
    class Node:
        def __init__(self, options):
            assert module.os.environ['GZ_PARTITION'] == 'test_partition'
            assert options.partition == 'test_partition'
            self.options = options
            nodes.append(self)
        def request(self, service, request, request_type, response_type, timeout):
            calls.append(json.loads(request.data))
            assert timeout == 5000
            return True, SimpleNamespace(data='{"success":true}')
    monkeypatch.setattr(module, '_bindings', lambda: (Node, SimpleNamespace, Message))
    monkeypatch.setenv('GZ_PARTITION', 'previous_partition')
    monkeypatch.setattr(module.subprocess, 'run', lambda *_a, **_k: pytest.fail('persistent transport must not spawn CLI'))
    client = module.GazeboClient('/world/field/harvest/command', 'test_partition')
    for index in range(10):
        assert client.request({'operation':'scene', 'index':index})['success']
    assert len(nodes) == 1 and len(calls) == 10


def test_uncertain_physical_request_is_sent_once_without_fallback_or_retry(monkeypatch):
    calls = []
    class Node:
        def __init__(self, options): self.options = options
        def request(self, *_args):
            calls.append(1)
            return False, Message()
    monkeypatch.setattr(module, '_bindings', lambda: (Node, SimpleNamespace, Message))
    monkeypatch.setenv('GZ_PARTITION', 'previous_partition')
    monkeypatch.setattr(module.subprocess, 'run', lambda *_a, **_k: pytest.fail('uncertain result must not fall back'))
    client = module.GazeboClient('/service', 'test_partition', allow_cli_fallback=True)
    with pytest.raises(RuntimeError, match='bounded transport deadline'):
        client.request({'operation':'attach'})
    assert calls == [1]


def test_missing_bindings_require_explicit_startup_fallback(monkeypatch):
    def unavailable(): raise ImportError('no Gazebo Python binding')
    monkeypatch.setattr(module, '_bindings', unavailable)
    monkeypatch.setenv('GZ_PARTITION', 'previous_partition')
    with pytest.raises(RuntimeError, match='Python bindings required'):
        module.GazeboClient('/service', 'test_partition')
    client = module.GazeboClient('/service', 'test_partition', allow_cli_fallback=True)
    assert client.node is None and client.backend == 'explicit_cli_fallback'
