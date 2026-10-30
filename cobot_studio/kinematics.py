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
