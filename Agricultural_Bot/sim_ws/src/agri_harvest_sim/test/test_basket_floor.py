"""Offline geometry regressions for the basket floor and deposit region.

Xacro and SDF conversion do not start Gazebo, ROS nodes, or any motion.
"""

from itertools import product
from pathlib import Path
import struct
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest


PACKAGE = Path(__file__).parents[1]
ROOT = PACKAGE.parents[2]
DESCRIPTION = ROOT / 'ros2_ws/src/agri_robot_description'
XACRO = DESCRIPTION / 'urdf/agri_robot.urdf.xacro'
MESH = DESCRIPTION / 'meshes/visual/tomato_box_link.STL'
FAILED_CENTRE = np.array([.00768355779502139, .20981804783442426,
                          .17734143827867221])
RADIUS = .028031044266530716
PROXY_NAME = 'basket_solid_floor_collision'


@pytest.fixture(scope='module')
def models():
    output = {}
    for gazebo in (False, True):
        completed = subprocess.run(
            ['xacro', str(XACRO), f'use_gazebo:={str(gazebo).lower()}'],
            check=True, capture_output=True, text=True, timeout=30)
        output[gazebo] = completed.stdout
    return output


@pytest.fixture(scope='module')
def floor(models):
    robot = ET.fromstring(models[True])
    proxy = robot.find(f"link[@name='tomato_box_link']/collision[@name='{PROXY_NAME}']")
    assert proxy is not None
    centre = np.fromstring(proxy.find('origin').get('xyz'), sep=' ')
    size = np.fromstring(proxy.find('geometry/box').get('size'), sep=' ')
    return centre, size


@pytest.fixture(scope='module')
def mesh_triangles():
    data = MESH.read_bytes()
    count, = struct.unpack_from('<I', data, 80)
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)),
                      ('attribute', '<u2')])
    return np.frombuffer(data, dtype=dtype, count=count, offset=84)[
        'vertices'].astype(float)


@pytest.fixture(scope='module')
def mesh_floor(mesh_triangles):
    # This plane is the real large, solid bottom panel in the current CAD mesh;
    # the visually open lattice belongs to the side walls, not this floor.
    horizontal = mesh_triangles[
        np.all(np.abs(mesh_triangles[:, :, 1] - .20559) < 1e-7, axis=1)]
    assert len(horizontal) > 0
    return horizontal


def covered_by_triangles(points, triangles, axes=(0, 2)):
    """Return triangle-union membership of each projected test point."""
    vertices = triangles[:, :, list(axes)]
    a = vertices[:, 0]
    u, v = vertices[:, 1] - a, vertices[:, 2] - a
    cross = lambda left, right: left[..., 0] * right[..., 1] - left[..., 1] * right[..., 0]
    determinant = cross(u, v)
    valid = np.abs(determinant) > 1e-15
    difference = np.asarray(points)[:, None, :] - a[None, :, :]
    denominator = np.where(valid, determinant, 1.)
    beta = cross(difference, v) / denominator
    gamma = cross(u, difference) / denominator
    return np.any(valid & (beta >= -1e-9) & (gamma >= -1e-9) &
                  (beta + gamma <= 1. + 1e-9), axis=1)


def test_real_xacro_adds_only_simulation_proxy_and_preserves_the_original_mesh(models):
    regular = ET.fromstring(models[False]).find("link[@name='tomato_box_link']")
    simulated = ET.fromstring(models[True]).find("link[@name='tomato_box_link']")
    assert regular.find(f"collision[@name='{PROXY_NAME}']") is None
    assert len(regular.findall('collision')) == 1
    assert len(simulated.findall('collision')) == 2
    def normalized(element):
        # Adding a sibling changes xacro's indentation/tail whitespace, which
        # is not part of the original collision or visual geometry.
        return ET.canonicalize(ET.tostring(element, encoding='unicode'), strip_text=True)

    assert normalized(regular.find('collision')) == normalized(simulated.find('collision'))
    assert normalized(regular.find('visual')) == normalized(simulated.find('visual'))


def test_proxy_top_matches_solid_mesh_floor_without_growing_its_footprint(floor, mesh_floor):
    centre, size = floor
    assert centre == pytest.approx([0., .19809, .1713506044])
    assert size == pytest.approx([.488, .015, .312])
    top = centre[1] + size[1] / 2
    assert top == pytest.approx(float(np.median(mesh_floor[:, :, 1])), abs=1e-7)
    x_samples = np.linspace(centre[0] - size[0] / 2, centre[0] + size[0] / 2, 21)
    z_samples = np.linspace(centre[2] - size[2] / 2, centre[2] + size[2] / 2, 21)
    assert np.all(covered_by_triangles(list(product(x_samples, z_samples)), mesh_floor))


def test_native_lateral_bound_matches_real_basket_inner_walls(mesh_triangles, tmp_path):
    # Sample real side-wall lattice bars near the reported resting fruit. The
    # opening at its exact Z has no triangle, so use bars on both sides of that
    # opening rather than treating a mesh hole as a narrower basket wall.
    wall_points_yz = [[.23191498929366317, .10],
                      [.23191498929366317, .12], [.32, .11]]
    measured_half_widths = []
    for sign in (-1., 1.):
        wall = mesh_triangles[
            np.all(np.abs(mesh_triangles[:, :, 0] - sign * .275) < 1e-7, axis=1)]
        assert len(wall) > 0, 'both physical inner side-wall planes must exist'
        assert np.all(covered_by_triangles(wall_points_yz, wall, axes=(1, 2)))
        measured_half_widths.append(abs(float(np.median(wall[:, :, 0]))))

    # Compile the production constant and predicate: the accepted horizontal
    # region must follow the measured inner wall, not an inset that rejects a
    # whole sphere already resting inside it.
    executable = tmp_path / 'basket_inner_wall'
    source = r'''
#include "BasketEvidence.hh"
#include <iomanip>
#include <iostream>
int main() {
  const std::array<double, 3> reported{
    .24434856214129808, .23191498929366317, .11037605517234184};
  auto mirrored = reported;
  mirrored[0] = -mirrored[0];
  constexpr double radius = .026327331812614842;
  std::cout << std::setprecision(17) << agri::kBasketInnerHalfWidthX << " "
            << agri::BasketContainsSphere(reported, radius) << " "
            << agri::BasketContainsSphere(mirrored, radius);
}
'''
    subprocess.run(['c++', '-std=c++17', '-I', str(PACKAGE / 'src'),
                    '-x', 'c++', '-', '-o', str(executable)],
                   input=source, check=True, capture_output=True, text=True, timeout=30)
    response = subprocess.run([str(executable)], check=True,
                              capture_output=True, text=True, timeout=10)
    native_half_width, accepted, mirrored_accepted = response.stdout.split()
    assert float(native_half_width) == pytest.approx(measured_half_widths[0], abs=1e-7)
    assert float(native_half_width) == pytest.approx(measured_half_widths[1], abs=1e-7)
    assert accepted == mirrored_accepted == '1'


def test_reported_sphere_penetrates_floor_but_is_inside_requested_ideal_basket_region(
        floor, mesh_floor, tmp_path):
    centre, size = floor
    top = centre[1] + size[1] / 2
    assert covered_by_triangles([FAILED_CENTRE[[0, 2]]], mesh_floor)[0]
    separation = FAILED_CENTRE[1] - float(np.median(mesh_floor[:, :, 1]))
    assert separation == pytest.approx(.004228053046, abs=1e-10)
    assert RADIUS - separation == pytest.approx(.023802991221, abs=1e-10)
    # Keep the penetration calculation above: membership in the explicitly
    # expanded ideal region does not mean the sphere is physically supported.
    # Call the actual native predicate instead of copying its region bounds.
    executable = tmp_path / 'basket_containment'
    source = r'''
#include "BasketEvidence.hh"
#include <iostream>
int main(int argc, char **argv) {
  constexpr double radius = .028031044266530716;
  const std::array<double, 3> failed{
    .00768355779502139, .20981804783442426, .17734143827867221};
  auto supported = failed;
  supported[1] = std::stod(argv[1]) + radius;
  std::cout << agri::BasketContainsSphere(failed, radius) << " "
            << agri::BasketContainsSphere(supported, radius);
}
'''
    subprocess.run(['c++', '-std=c++17', '-I', str(PACKAGE / 'src'),
                    '-x', 'c++', '-', '-o', str(executable)],
                   input=source, check=True, capture_output=True, text=True, timeout=30)
    response = subprocess.run([str(executable), str(top)], check=True,
                              capture_output=True, text=True, timeout=10)
    assert response.stdout.split() == ['1', '1']


def rpy_rotation(roll, pitch, yaw):
    cr, cp, cy = np.cos([roll, pitch, yaw])
    sr, sp, sy = np.sin([roll, pitch, yaw])
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def test_sdf_fixed_joint_reduction_keeps_floor_at_correct_ros_height(models, floor, tmp_path):
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text(models[True])
    converted = subprocess.run(['gz', 'sdf', '-p', str(urdf)], check=True,
                               capture_output=True, text=True, timeout=30)
    model = ET.fromstring(converted.stdout).find('model')
    matches = [(link, collision) for link in model.findall('link')
               for collision in link.findall('collision')
               if PROXY_NAME in collision.get('name', '')]
    assert len(matches) == 1
    link, proxy = matches[0]
    assert link.get('name') == 'base_footprint'
    pose = np.fromstring(proxy.findtext('pose'), sep=' ')
    dimensions = np.fromstring(proxy.findtext('geometry/box/size'), sep=' ')
    cad_centre, cad_size = floor
    assert pose[:3] == pytest.approx(cad_centre[[2, 0, 1]], abs=1e-12)
    assert dimensions == pytest.approx(cad_size)
    corners = np.array(list(product(*[[-size / 2, size / 2] for size in dimensions])))
    ros_corners = corners @ rpy_rotation(*pose[3:]).T + pose[:3]
    assert ros_corners[:, 2].max() == pytest.approx(.20559, abs=1e-12)
    assert ros_corners[:, 2].min() == pytest.approx(.19059, abs=1e-12)
