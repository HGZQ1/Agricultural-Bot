"""Keep the Gazebo steering override separate from the canonical limits."""

from pathlib import Path
import xml.etree.ElementTree as ET

import xacro


REPOSITORY = Path(__file__).resolve().parents[4]
CANONICAL = REPOSITORY / (
    'ros2_ws/src/agri_robot_description/urdf/agri_robot.urdf.xacro')
SIMULATION = REPOSITORY / (
    'sim_ws/src/agri_sim_description/urdf/agri_robot.gazebo.urdf.xacro')


def _limits(entry, mappings=None):
    root = ET.fromstring(xacro.process_file(
        str(entry), mappings=mappings or {}).toxml())
    return {
        joint.get('name'): joint.find('limit').attrib
        for joint in root.findall('joint')
        if joint.get('name', '').endswith('_steer_joint')
    }


def test_simulation_override_keeps_canonical_position_and_speed_limits():
    canonical = _limits(CANONICAL)
    simulation = _limits(SIMULATION)
    assert set(canonical) == set(simulation)
    assert len(canonical) == 4
    for name, limit in canonical.items():
        assert float(limit['effort']) == 10.0
        assert float(simulation[name]['effort']) == 40.0
        for key in ('lower', 'upper', 'velocity'):
            assert float(simulation[name][key]) == float(limit[key])


def test_steering_authority_can_be_overridden_for_contact_experiments():
    limits = _limits(SIMULATION, {'steering_effort_limit': '10.0'})
    assert all(float(limit['effort']) == 10.0 for limit in limits.values())
