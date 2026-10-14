"""Regression against the installed generator/critic, without ROS or motion.

The v5 stopping plateau must remain reproducible.  v6 must reach another cell
in prediction, while reducing only legal inflated soft costs and continuing
to reject lethal/unknown cells and off-grid footprints.
"""

import copy
import json
from pathlib import Path
import shutil
import subprocess

from ament_index_python.packages import get_package_prefix
import pytest
import yaml

from test_navigation_smoothing_launch import launch_case, load_parameter_file


CONFIG = Path(__file__).resolve().parents[1] / 'config'
V5 = CONFIG / 'nav2_close_harvest_omni.yaml'
V6 = CONFIG / 'nav2_close_harvest_omni_v6.yaml'


def read(path):
    return yaml.safe_load(path.read_text())


def test_v6_changes_only_prediction_and_nonzero_soft_cost():
    before = read(V5)
    expected = copy.deepcopy(before)
    settings = expected['controller_server']['ros__parameters']['FollowPath']
    settings['trajectory_generator_name'] = 'dwb_plugins::StandardTrajectoryGenerator'
    settings['limit_vel_cmd_in_traj'] = False
    settings['acc_lim_x'] = settings['acc_lim_y'] = .12
    settings.pop('sim_period')
    settings['ObstacleFootprint']['scale'] = .01
    assert read(V6) == expected
    assert settings['critics'][0] == 'ObstacleFootprint'
    assert settings['ObstacleFootprint']['scale'] > 0.
    assert settings['decel_lim_x'] == settings['decel_lim_y'] == -.05
    # The installed native probe takes only acceleration as an argument;
    # fail if this profile's other supplied settings no longer match it.
    for key, value in {
            'min_vel_x': -.12, 'max_vel_x': .12,
            'min_vel_y': -.08, 'max_vel_y': .08, 'max_vel_theta': .05,
            'sim_time': 1.5, 'linear_granularity': .025,
            'angular_granularity': .01, 'vx_samples': 11,
            'vy_samples': 11, 'vtheta_samples': 5}.items():
        assert settings[key] == value


@pytest.fixture(scope='module')
def native_probe(tmp_path_factory):
    compiler = shutil.which('g++')
    assert compiler, 'The installed-library regression needs the system C++ compiler'
    prefix = Path(get_package_prefix('dwb_plugins'))
    output = tmp_path_factory.mktemp('native_dwb') / 'generator_probe'
    source = Path(__file__).with_name('native_dwb_generator_probe.cpp')
    args = [compiler, '-std=c++17', '-O2', str(source), '-o', str(output),
            '-I/usr/include/eigen3', '-I' + str(prefix / 'include')]
    args += ['-I' + str(path) for path in (prefix / 'include').iterdir()
             if path.is_dir()]
    args += ['-L' + str(prefix / 'lib'), '-Wl,-rpath,' + str(prefix / 'lib'),
             '-lstandard_traj_generator', '-ldwb_critics', '-lnav2_costmap_2d_core']
    compiled = subprocess.run(args, capture_output=True, text=True, timeout=45)
    assert compiled.returncode == 0, compiled.stderr

    def run(kind, value):
        result = subprocess.run([str(output), kind, str(value)], capture_output=True,
                                text=True, timeout=10)
        assert result.returncode == 0, result.stderr
        return json.loads(result.stdout)
    return run


def test_installed_standard_generator_escapes_limited_window_grid_plateau(native_probe):
    limited = native_probe('limited', .05)
    standard = native_probe('standard', .12)
    assert limited['max_target_x'] == pytest.approx(.0025)
    assert limited['forward_endpoint_m'] == pytest.approx(.00375)
    assert limited['forward_cell_changes'] == limited['lateral_cell_changes'] == 0
    assert standard['max_target_x'] == pytest.approx(.12)
    assert standard['max_target_y'] == pytest.approx(.08)
    assert standard['forward_endpoint_m'] > .05
    assert standard['lateral_endpoint_m'] > .05
    assert standard['forward_cell_changes'] > 0
    assert standard['lateral_cell_changes'] > 0
    assert standard['returned_forward_command'] == pytest.approx(.12)
    assert standard['returned_lateral_command'] == pytest.approx(.08)
    assert standard['samples'] <= 12 * 12 * 6


@pytest.mark.parametrize('scale, weighted', [(1., 12.65), (.01, .1265)])
def test_installed_footprint_critic_retains_hard_rejection_at_small_nonzero_scale(
        native_probe, scale, weighted):
    result = native_probe('footprint', scale)
    assert result['inflated_raw_score'] == 253
    assert result['weighted_inflated_score'] == pytest.approx(weighted)
    assert result['hard_rejections'] == 3  # lethal, unknown, and off-grid edge


def test_v6_real_launch_expands_safety_chain_and_preserves_footprint(launch_case):
    configure, expand = launch_case
    context, nodes = configure(
        params_file=str(V6), velocity_smoothing='false',
        navigation_map='/tmp/navigation collision.yaml', obstacle_scan='/scan_obstacles')
    assert nodes['velocity_smoother'].condition.evaluate(context) is False
    node, controller = expand('controller_server')
    assert controller['FollowPath']['trajectory_generator_name'] == (
        'dwb_plugins::StandardTrajectoryGenerator')
    assert controller['FollowPath']['ObstacleFootprint']['scale'] == .01
    _, monitor = expand('collision_monitor')
    assert monitor['cmd_vel_in_topic'] == '/cmd_vel_nav_raw'
    assert monitor['cmd_vel_out_topic'] == '/cmd_vel_nav'
    expanded = load_parameter_file(Path(node._Node__expanded_parameter_arguments[0][0]))
    original = read(CONFIG / 'nav2_navigation.yaml')
    for name in ('local_costmap', 'global_costmap'):
        costmap = expanded[name][name]['ros__parameters']
        baseline = original[name][name]['ros__parameters']
        assert costmap['footprint'] == baseline['footprint']
        assert costmap['footprint_padding'] == .035
        assert costmap['inflation_layer'] == baseline['inflation_layer']
        assert costmap['keepout_filter'] == baseline['keepout_filter']
        assert costmap['static_layer']['map_topic'] == '/navigation_map'
        assert costmap['obstacle_layer']['scan']['topic'] == '/scan_obstacles'
