"""Verify v7's sole AMCL change and actual installed crab-motion predictions."""
import copy
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import pytest
import yaml

from test_navigation_smoothing_launch import launch_case


CONFIG = Path(__file__).resolve().parents[1] / 'config'
V7 = CONFIG / 'nav2_close_harvest_omni_v7.yaml'


def read(path):
    return yaml.safe_load(path.read_text())


def test_v7_only_changes_amcl_motion_model_from_existing_sources():
    expected = copy.deepcopy(read(CONFIG / 'nav2_close_harvest_omni_v6.yaml'))
    expected.update(read(CONFIG / 'amcl.yaml'))
    expected['amcl']['ros__parameters']['robot_model_type'] = 'nav2_amcl::OmniMotionModel'
    assert read(V7) == expected
    assert expected['amcl']['ros__parameters']['scan_topic'] == '/scan'
    classes = {item.attrib['type'] for item in ET.parse(
        Path(get_package_share_directory('nav2_amcl')) / 'plugins.xml').iter('class')}
    assert expected['amcl']['ros__parameters']['robot_model_type'] in classes


def test_v7_real_launch_uses_separate_amcl_argument_and_unchanged_controller(launch_case):
    configure, expand = launch_case
    _, _ = configure(params_file=str(V7), amcl_params_file=str(V7),
                      navigation_map='/tmp/navigation collision.yaml',
                      obstacle_scan='/scan_obstacles', velocity_smoothing='false')
    _, amcl = expand('amcl')
    _, controller = expand('controller_server')
    expected = read(V7)
    assert amcl['robot_model_type'] == 'nav2_amcl::OmniMotionModel'
    assert amcl['scan_topic'] == '/scan'
    assert amcl['alpha1'] == amcl['alpha2'] == amcl['alpha3'] == amcl['alpha4'] == .2
    assert amcl['alpha5'] == .1
    assert controller == expected['controller_server']['ros__parameters']


@pytest.fixture(scope='module')
def native_model_probe(tmp_path_factory):
    compiler = shutil.which('g++')
    assert compiler
    prefix = Path(get_package_prefix('nav2_amcl'))
    output = tmp_path_factory.mktemp('native_amcl') / 'motion_probe'
    source = Path(__file__).with_name('native_amcl_motion_probe.cpp')
    args = [compiler, '-std=c++17', '-O2', str(source), '-o', str(output),
            '-I/usr/include/eigen3', '-I' + str(prefix / 'include')]
    args += ['-I' + str(path) for path in (prefix / 'include').iterdir() if path.is_dir()]
    args += ['-L' + str(prefix / 'lib'), '-Wl,-rpath,' + str(prefix / 'lib'), '-lmotions_lib']
    compiled = subprocess.run(args, capture_output=True, text=True, timeout=45)
    assert compiled.returncode == 0, compiled.stderr

    def run(model, motion, noise):
        result = subprocess.run([str(output), model, motion, noise], capture_output=True,
                                text=True, timeout=10)
        assert result.returncode == 0, result.stderr
        return json.loads(result.stdout)
    return run


@pytest.mark.parametrize('model', ['diff', 'omni'])
def test_noiseless_lateral_prediction_has_correct_displacement_and_final_yaw(
        native_model_probe, model):
    result = native_model_probe(model, 'lateral', 'exact')
    assert result['mean_xyyaw'] == pytest.approx([0., .05, 0.], abs=1e-12)
    assert result['std_xyyaw'] == pytest.approx([0., 0., 0.], abs=1e-12)


def test_installed_diff_model_artificial_turn_noise_is_excessive_for_crab_motion(
        native_model_probe):
    diff = native_model_probe('diff', 'lateral', 'noisy')
    omni = native_model_probe('omni', 'lateral', 'noisy')
    # Same baseline alphas and a real 5 cm lateral delta with zero odometry yaw.
    # Inspect actual installed predictions, not a reimplementation of formulas.
    assert diff['std_xyyaw'][2] > .5
    assert max(diff['std_xyyaw'][:2]) > .5
    assert omni['std_xyyaw'][2] < .04
    assert max(omni['std_xyyaw'][:2]) < .04
    assert omni['mean_xyyaw'] == pytest.approx([0., .05, 0.], abs=.001)
