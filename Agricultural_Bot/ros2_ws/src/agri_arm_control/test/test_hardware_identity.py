"""Empty plugin metadata must never silently authorize arbitrary hardware."""

import copy
from types import SimpleNamespace as NS

import pytest

from agri_arm_control.hardware import validate_sim_hardware


URDF = '''<robot name="live"><ros2_control name="GazeboSimSystem" type="system">
<hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>
<joint name="arm"><command_interface name="position"/>
<state_interface name="position"/><state_interface name="velocity"/></joint>
</ros2_control></robot>'''


def component(plugin=''):
    interface = lambda name: NS(name=name, is_available=True, is_claimed=True)
    return NS(name='GazeboSimSystem', plugin_name=plugin,
              state=NS(id=3, label='active'),
              command_interfaces=[interface('arm/position')],
              state_interfaces=[interface('arm/position'), interface('arm/velocity')])


def test_jazzy_empty_plugin_requires_exact_live_urdf_interfaces():
    assert validate_sim_hardware([component()], 'gazebo', ['arm'], URDF)


@pytest.mark.parametrize('mode,plugin', [
    ('mock', ''), ('mock', 'gz_ros2_control/GazeboSimSystem'),
    ('gazebo', 'real_hardware/RobotDriver')])
def test_empty_mock_or_foreign_nonempty_plugin_cannot_use_urdf_fallback(mode, plugin):
    with pytest.raises(ValueError):
        validate_sim_hardware([component(plugin)], mode, ['arm'], URDF)


@pytest.mark.parametrize('description', [None, '<broken>',
    URDF.replace('name="GazeboSimSystem"', 'name="OtherSystem"'),
    URDF.replace('gz_ros2_control/GazeboSimSystem', 'real_hardware/RobotDriver'),
    URDF.replace('name="arm"', 'name="other_joint"')])
def test_missing_wrong_or_malformed_runtime_urdf_fails_closed(description):
    with pytest.raises(ValueError):
        validate_sim_hardware([component()], 'gazebo', ['arm'], description)


@pytest.mark.parametrize('changed', ['inactive', 'unavailable', 'missing_velocity',
                                      'extra_interface', 'missing_configured_joint'])
def test_live_component_and_interfaces_must_correspond_to_runtime_description(changed):
    live = component()
    required = ['arm']
    if changed == 'inactive':
        live.state.id, live.state.label = 2, 'inactive'
    elif changed == 'unavailable':
        live.command_interfaces[0].is_available = False
    elif changed == 'missing_velocity':
        live.state_interfaces.pop()
    elif changed == 'extra_interface':
        live.command_interfaces.append(copy.copy(live.command_interfaces[0]))
        live.command_interfaces[-1].name = 'extra/position'
    else:
        required.append('missing')
    with pytest.raises(ValueError):
        validate_sim_hardware([live], 'gazebo', required, URDF)


def test_explicit_generic_mock_still_requires_active_feedback_interfaces():
    assert validate_sim_hardware([component('mock_components/GenericSystem')],
                                 'mock', ['arm'])
