"""Frame-independent grasp geometry; task coordinates follow REP 103."""

import math


def unit(vector):
    norm = math.sqrt(sum(float(v)**2 for v in vector))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid direction')
    return tuple(float(v) / norm for v in vector)


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def multiply(a, b):
    x, y, z, w = a
    xx, yy, zz, ww = b
    return unit((w*xx+x*ww+y*zz-z*yy, w*yy-x*zz+y*ww+z*xx,
                 w*zz+x*yy-y*xx+z*ww, w*ww-x*xx-y*yy-z*zz))


def rotate(q, vector):
    x, y, z, w = unit(q)
    vx, vy, vz = vector
    return ((1-2*(y*y+z*z))*vx+2*(x*y-z*w)*vy+2*(x*z+y*w)*vz,
            2*(x*y+z*w)*vx+(1-2*(x*x+z*z))*vy+2*(y*z-x*w)*vz,
            2*(x*z-y*w)*vx+2*(y*z+x*w)*vy+(1-2*(x*x+y*y))*vz)


def quaternion_from_axes(x, y, z):
    """Convert a right-handed orthonormal basis to a normalized quaternion."""
    m = [[x[i], y[i], z[i]] for i in range(3)]
    trace = m[0][0]+m[1][1]+m[2][2]
    if trace > 0:
        scale = math.sqrt(trace+1)*2
        return unit(((m[2][1]-m[1][2])/scale, (m[0][2]-m[2][0])/scale,
                     (m[1][0]-m[0][1])/scale, scale/4))
    i = max(range(3), key=lambda j: m[j][j])
    j, k = (i+1) % 3, (i+2) % 3
    scale = math.sqrt(1+m[i][i]-m[j][j]-m[k][k])*2
    q = [0., 0., 0., 0.]
    q[i], q[j], q[k] = scale/4, (m[j][i]+m[i][j])/scale, (m[k][i]+m[i][k])/scale
    q[3] = (m[k][j]-m[j][k])/scale
    return unit(q)


def grasp_candidates(surface, side, pre_offset=0.08, retreat_offset=0.12,
                     centre_offset=0.025):
    """TCP +Z approaches the selected side; radius correction is a proxy."""
    if side not in {'left', 'right'}:
        raise ValueError('unknown work side')
    if not 0 <= centre_offset <= 0.04:
        raise ValueError('surface centre offset exceeds configured geometric bound')
    sign = 1 if side == 'left' else -1
    for pitch in (0., math.radians(15), -math.radians(15)):
        direction = (0., sign*math.cos(pitch), math.sin(pitch))
        x_axis = (1., 0., 0.)
        y_axis = cross(direction, x_axis)
        base = quaternion_from_axes(x_axis, y_axis, direction)
        for roll in (0., math.pi/2, -math.pi/2):
            orientation = multiply(base, (0., 0., math.sin(roll/2), math.cos(roll/2)))
            centre = tuple(surface[i]+centre_offset*direction[i] for i in range(3))
            pre = tuple(centre[i]-pre_offset*direction[i] for i in range(3))
            retreat = tuple(centre[i]-retreat_offset*direction[i] for i in range(3))
            yield {'centre': centre, 'pre': pre, 'retreat': retreat,
                   'orientation': orientation, 'direction': direction}


def transform_pose(position, orientation, transform):
    """Use the actual rigid TF, including CAD frame rotation."""
    q = transform.transform.rotation
    quaternion = (q.x, q.y, q.z, q.w)
    xyz = rotate(quaternion, position)
    t = transform.transform.translation
    return ((xyz[0]+t.x, xyz[1]+t.y, xyz[2]+t.z), multiply(quaternion, orientation))


def basket_poses(drop=(0.17, 0., 0.60), clearance=0.15):
    """Basket height increments standard task-frame +Z, never CAD +Z."""
    if clearance <= 0 or not all(math.isfinite(v) for v in (*drop, clearance)):
        raise ValueError('invalid basket clearance')
    return tuple(drop), (drop[0], drop[1], drop[2]+clearance)
