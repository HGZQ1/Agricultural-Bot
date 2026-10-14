"""Fail-closed simulation hardware identity checks, including Jazzy metadata gaps."""

from xml.etree import ElementTree


def validate_sim_hardware(components, mode, required_joints, runtime_description=None):
    """Validate live hardware; an empty Gazebo plugin needs matching runtime URDF.

    The worker additionally requires a fresh SCENE_READY service ACK before it
    publishes backend_ready or accepts motion. URDF corroboration alone is not
    the complete Gazebo readiness gate.
    """
    allowed = {'mock': 'mock_components/GenericSystem',
               'gazebo': 'gz_ros2_control/GazeboSimSystem'}
    if mode not in allowed or not components:
        raise ValueError('simulation hardware components or backend mode missing')
    expected_plugin = allowed[mode]
    names = [component.name for component in components]
    if len(set(names)) != len(names) or any(not name for name in names):
        raise ValueError('hardware component names must be nonempty and unique')
    # A nonempty foreign plugin is never rescued by another description.
    for component in components:
        if component.plugin_name and component.plugin_name != expected_plugin:
            raise ValueError(f'hardware plugin {component.plugin_name!r} is not permitted')
        if component.state.id != 3 or component.state.label != 'active':
            raise ValueError(f'hardware component {component.name!r} is not active')
        if not component.plugin_name and mode != 'gazebo':
            raise ValueError('empty mock hardware plugin identity is not permitted')

    controls = {}
    if any(not component.plugin_name for component in components):
        if not runtime_description:
            raise ValueError('empty Gazebo plugin requires runtime robot_description')
        try:
            root = ElementTree.fromstring(runtime_description)
        except ElementTree.ParseError as exc:
            raise ValueError('invalid runtime robot_description') from exc
        if root.tag != 'robot':
            raise ValueError('runtime robot_description must be a robot URDF')
        for control in root.findall('ros2_control'):
            name = control.get('name', '')
            if not name or name in controls:
                raise ValueError('runtime ros2_control names must be unique')
            controls[name] = control

    all_commands, all_states = set(), set()
    for component in components:
        command_names = [interface.name for interface in component.command_interfaces]
        state_names = [interface.name for interface in component.state_interfaces]
        if (len(command_names) != len(set(command_names)) or
                len(state_names) != len(set(state_names))):
            raise ValueError('duplicate live hardware interfaces')
        if any(not interface.is_available for interface in
               (*component.command_interfaces, *component.state_interfaces)):
            raise ValueError('live simulation hardware interfaces are unavailable')
        all_commands.update(command_names)
        all_states.update(state_names)
        if component.plugin_name:
            continue
        control = controls.get(component.name)
        if control is None or control.get('type') != 'system':
            raise ValueError('live hardware component does not match runtime ros2_control')
        if (control.findtext('hardware/plugin') or '').strip() != expected_plugin:
            raise ValueError('runtime ros2_control plugin is not GazeboSimSystem')
        joints = control.findall('joint')
        joint_names = [joint.get('name', '') for joint in joints]
        if not joints or any(not name for name in joint_names) or len(set(joint_names)) != len(joints):
            raise ValueError('runtime hardware joints must be nonempty and unique')
        declared_commands = {f'{joint.get("name")}/{interface.get("name")}'
                             for joint in joints for interface in joint.findall('command_interface')}
        declared_states = {f'{joint.get("name")}/{interface.get("name")}'
                           for joint in joints for interface in joint.findall('state_interface')}
        if set(command_names) != declared_commands or set(state_names) != declared_states:
            raise ValueError('live hardware joint/interface sets do not match runtime URDF')
    if not {f'{joint}/position' for joint in required_joints}.issubset(all_commands):
        raise ValueError('configured arm/gripper position command interfaces are missing')
    if not {f'{joint}/{interface}' for joint in required_joints
            for interface in ('position', 'velocity')}.issubset(all_states):
        raise ValueError('configured arm/gripper measured position/velocity interfaces are missing')
    return True
