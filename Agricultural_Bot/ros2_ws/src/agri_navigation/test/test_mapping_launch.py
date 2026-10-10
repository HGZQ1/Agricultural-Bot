"""Contract tests for the standalone mapping launch description."""

import importlib.util
from pathlib import Path

from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'mapping.launch.py'


def _load_mapping_launch():
    spec = importlib.util.spec_from_file_location('mapping_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_activation_handler_is_registered_before_configure_event(monkeypatch,
                                                                 tmp_path):
    """Do not miss a fast configuring-to-inactive lifecycle transition."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    module = _load_mapping_launch()
    monkeypatch.setattr(
        module, 'get_package_share_directory', lambda _name: str(PACKAGE_ROOT))

    entities = module.generate_launch_description().entities
    handler_indexes = [
        index for index, entity in enumerate(entities)
        if isinstance(entity, RegisterEventHandler)
        and isinstance(entity.event_handler, OnStateTransition)
    ]
    configure_indexes = [
        index for index, entity in enumerate(entities)
        if isinstance(entity, EmitEvent)
        and isinstance(entity.event, ChangeState)
        and entity.event.transition_id == Transition.TRANSITION_CONFIGURE
    ]

    assert len(handler_indexes) == 1
    assert len(configure_indexes) == 1
    assert handler_indexes[0] < configure_indexes[0]

    handler = entities[handler_indexes[0]].event_handler
    activation_events = [
        entity.event for entity in handler.entities
        if isinstance(entity, EmitEvent)
        and isinstance(entity.event, ChangeState)
    ]
    assert [event.transition_id for event in activation_events] == [
        Transition.TRANSITION_ACTIVATE,
    ]
