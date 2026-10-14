"""Frame math and observation matching shared by the simulation gateway."""
import math


def rotate(q, point):
    x, y, z, w = q
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid rotation')
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    px, py, pz = point
    tx, ty, tz = 2*(y*pz-z*py), 2*(z*px-x*pz), 2*(x*py-y*px)
    return [px+w*tx+y*tz-z*ty, py+w*ty+z*tx-x*tz, pz+w*tz+x*ty-y*tx]


def multiply(a, b):
    ax,ay,az,aw = a; bx,by,bz,bw = b
    return [aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz]


def inverse_point(pose, point):
    q = pose['xyzw']; inv = [-q[0], -q[1], -q[2], q[3]]
    return rotate(inv, [point[i]-pose['xyz'][i] for i in range(3)])


def relative_pose(parent, child):
    q = parent['xyzw']; inv = [-q[0], -q[1], -q[2], q[3]]
    return {'xyz': inverse_point(parent, child['xyz']), 'xyzw': multiply(inv, child['xyzw'])}


def match_observation(snapshot, point, plant_id):
    matches = []
    for fruit in snapshot['fruits']:
        if fruit['plant_id'] != plant_id or not fruit['ripe'] or fruit['state'] != 'ON_PLANT':
            continue
        base_point = inverse_point(snapshot['base_pose'], fruit['xyz'])
        gap = math.dist(base_point, point)
        if gap <= fruit['radius'] + 0.03:
            matches.append((gap, fruit))
    matches.sort(key=lambda pair: pair[0])
    if not matches:
        raise ValueError('visual observation has no matching unpicked ripe fruit')
    if len(matches) > 1 and matches[1][0]-matches[0][0] < 0.005:
        raise ValueError('ambiguous fruit association')
    return matches[0][1]
