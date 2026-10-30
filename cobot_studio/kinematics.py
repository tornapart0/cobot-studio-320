"""2022 M5 joint frames from Elephant Robotics' noetic URDF, in millimetres.

Source and frame conventions are recorded in docs/JOINT_MAP.md. No mesh assets.
SDK J1..J6 degrees map directly to local +Z rotations in this ordered chain.
"""
import math
from .core import angles

MODEL_ID = 'mycobot-320-m5-2022'
# Origin XYZ (mm), origin RPY (rad). These are frame offsets, not link lengths.
ORIGINS = (
    ((0, 0, 173.9), (0, 0, 0)),
    ((0, -88.78, 0), (0, -1.5708, 1.5708)),
    ((135, 0, -88.78), (0, 0, 0)),
    ((120, 0, 88.78), (0, 0, 1.57080)),
    ((0, -95, 0), (1.5708, 0, 0)),
    ((0, 65.5, 0), (-1.5708, 0, 0)),
)


def multiply(a, b):
    return tuple(tuple(sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def apply(matrix, vector):
    return tuple(sum(row[k]*vector[k] for k in range(3)) for row in matrix)


def rotation(axis, angle):
    c, s = math.cos(angle), math.sin(angle)
    if axis == 'x':
        return ((1,0,0),(0,c,-s),(0,s,c))
    if axis == 'y':
        return ((c,0,s),(0,1,0),(-s,0,c))
    return ((c,-s,0),(s,c,0),(0,0,1))


def forward(q):
    orientation = ((1,0,0),(0,1,0),(0,0,1))
    position = (0,0,0)
    points, axes = [], []
    for angle, (xyz, rpy) in zip(angles(q), ORIGINS):
        offset = apply(orientation, xyz)
        position = tuple(a+b for a,b in zip(position, offset))
        origin_rotation = multiply(rotation('z', rpy[2]), multiply(rotation('y', rpy[1]), rotation('x', rpy[0])))
        orientation = multiply(orientation, origin_rotation)
        points.append(position)
        axes.append(apply(orientation, (0,0,1)))
        orientation = multiply(orientation, rotation('z', math.radians(angle)))
    return points, axes, orientation


def endpoint(q):
    return forward(q)[0][-1]


def link_paths(q):
    """Rigid orthogonal link centerlines in their moving parent frames.

    Joint frames stay unchanged. Only the physical-link depiction differs from
    the old diagonal chords between origins. Each bend is a local right angle.
    """
    orientation = ((1,0,0),(0,1,0),(0,0,1))
    position = (0,0,0)
    paths = []
    for angle, (xyz, rpy) in zip(angles(q), ORIGINS):
        path = [position]
        for i in range(3):
            if abs(xyz[i]) < 1e-9:
                continue
            step = tuple(xyz[i] if k == i else 0 for k in range(3))
            offset = apply(orientation, step)
            position = tuple(a+b for a,b in zip(position, offset))
            path.append(position)
        paths.append(path)
        origin_rotation = multiply(rotation('z', rpy[2]), multiply(rotation('y', rpy[1]), rotation('x', rpy[0])))
        orientation = multiply(orientation, origin_rotation)
        orientation = multiply(orientation, rotation('z', math.radians(angle)))
    return paths


def rotation_arc(point, axis, radius=20):
    """Positive right-hand rotation arrow in a plane normal to a joint axis."""
    def cross(a,b):
        return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
    reference = (1,0,0) if abs(axis[2]) > 0.9 else (0,0,1)
    u = cross(axis, reference)
    length = math.sqrt(sum(v*v for v in u))
    u = tuple(v/length for v in u)
    v = cross(axis,u)
    return [tuple(p+radius*(math.cos(a)*x+math.sin(a)*y) for p,x,y in zip(point,u,v))
            for a in (math.radians(-140+i*9) for i in range(31))]
