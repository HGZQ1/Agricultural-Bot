"""Observation geometry in the chassis task frame, whose +Z points upward.

These helpers describe camera motion; they do not execute a trajectory or
claim that an observation pose is reachable. MoveIt must check each pose and
the path between poses against the current robot and collision scene.
"""

import math

from .geometry import multiply, rotate, unit


def _finite_vector(values, size, name):
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f'{name} must contain {size} finite values') from exc
    if len(result) != size or not all(math.isfinite(value) for value in result):
        raise ValueError(f'{name} must contain {size} finite values')
    return result


def descending_heights(top, bottom, step):
    """Return at most 32 camera heights from top to bottom, including bottom.

    Heights use the standard task-frame Z axis. Positive ``step`` lowers the
    camera while the last, possibly shorter interval ends exactly at bottom.
    An excessively fine step is rejected rather than silently truncating the
    scan or returning a height below bottom.
    """
    top, bottom, step = _finite_vector((top, bottom, step), 3, 'scan heights')
    if top < bottom or step <= 0:
        raise ValueError('scan top must be at least bottom and step must be positive')
    if top == bottom:
        return [top]
    heights = [top]
    # Clamp only arithmetic roundoff at the bottom, not a whole extra interval.
    tolerance = min(1e-12, step * 1e-9)
    for index in range(1, 32):
        height = top - index * step
        if height <= bottom or math.isclose(height, bottom, rel_tol=0., abs_tol=tolerance):
            heights.append(bottom)
            return heights
        if height >= heights[-1]:
            raise ValueError('scan step is too small to lower the finite height')
        heights.append(height)
    raise ValueError('vertical scan would exceed 32 observation levels')


def camera_pose(side, x, y_abs, z):
    """Return camera position and optical quaternion for horizontal side viewing.

    The standard chassis task frame has +X forward, +Y left, and +Z upward.
    Optical +Z looks toward the work side and optical +Y points downward, so
    changing ``z`` translates the camera vertically without pitching its view.
    """
    if side not in {'left', 'right'}:
        raise ValueError('camera work side must be left or right')
    x, y_abs, z = _finite_vector((x, y_abs, z), 3, 'camera position')
    if y_abs < 0:
        raise ValueError('camera lateral distance must be nonnegative')
    root_half = math.sqrt(.5)
    if side == 'left':
        return (x, y_abs, z), (-root_half, 0., 0., root_half)
    return (x, -y_abs, z), (0., root_half, -root_half, 0.)


def tcp_pose_for_camera(camera_position, camera_orientation,
                        camera_to_tcp_xyz, camera_to_tcp_q):
    """Compose a desired task-frame camera pose with its actual fixed TCP TF.

    ``camera_to_tcp_*`` is the transform returned by lookup_transform with
    target=camera_optical_frame and source=grasp_tcp: TCP expressed in camera
    coordinates. The result is TCP expressed in the standard chassis task
    frame, and must then be transformed to the configured planning frame.
    """
    position = _finite_vector(camera_position, 3, 'camera position')
    orientation = unit(_finite_vector(camera_orientation, 4, 'camera quaternion'))
    offset = _finite_vector(camera_to_tcp_xyz, 3, 'camera-to-TCP translation')
    extrinsic = unit(_finite_vector(camera_to_tcp_q, 4, 'camera-to-TCP quaternion'))
    rotated_offset = rotate(orientation, offset)
    tcp_position = tuple(position[index] + rotated_offset[index] for index in range(3))
    if not all(math.isfinite(value) for value in tcp_position):
        raise ValueError('composed TCP position must remain finite')
    return tcp_position, multiply(orientation, extrinsic)
