"""Validated motion plans and a deliberately limited, non-executing Python editor."""
import ast
import math
from dataclasses import dataclass

# Intersection of published M5 ranges and current SDK limits. Not collision limits.
LIMITS = (165, 135, 145, 145, 165, 175)
MAX_SPEED = 10


def angles(values):
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise ValueError("Provide six joint angles in degrees.")
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        raise ValueError("Angles must be finite numbers.")
    if any(abs(v) > limit for v, limit in zip(values, LIMITS)):
        raise ValueError(f"Joint limits are ±{LIMITS} degrees.")
    return tuple(float(v) for v in values)


@dataclass(frozen=True)
class Move:
    target: tuple
    speed: int = 10

    def __post_init__(self):
        object.__setattr__(self, 'target', angles(self.target))
        if type(self.speed) is not int or not 1 <= self.speed <= MAX_SPEED:
            raise ValueError("Workbench speed must be an integer from 1 to 10.")


@dataclass(frozen=True)
class JointMove:
    joint_id: int
    angle: float
    speed: int = 5

    def __post_init__(self):
        if type(self.joint_id) is not int or not 1 <= self.joint_id <= 6:
            raise ValueError('Joint ID must be 1–6.')
        q = [0]*6
        q[self.joint_id-1] = self.angle
        angles(q)
        Move(tuple(q), self.speed)


def next_pose(current, command):
    if isinstance(command, Move):
        return command.target
    if isinstance(command, JointMove):
        q = list(angles(current))
        q[command.joint_id-1] = command.angle
        return angles(q)
    return angles(current)


@dataclass(frozen=True)
class Wait:
    seconds: float

    def __post_init__(self):
        if type(self.seconds) not in (int, float) or not math.isfinite(self.seconds) or not 0 <= self.seconds <= 30:
            raise ValueError("Wait must be between 0 and 30 seconds.")


def parse_program(source):
    """Accept literal robot.move([...], speed) and robot.wait(seconds) only."""
    if len(source) > 50000:
        raise ValueError("Program is too large.")
    tree = ast.parse(source)
    plan = []
    if not 1 <= len(tree.body) <= 500:
        raise ValueError("Use 1–500 commands.")
    for statement in tree.body:
        call = statement.value if isinstance(statement, ast.Expr) else None
        if not (isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
                and isinstance(call.func.value, ast.Name) and call.func.value.id == 'robot'
                and not call.keywords):
            raise ValueError("Use robot.joint(id, angle, speed), robot.move([six angles], speed) or robot.wait(seconds).")
        try:
            values = [ast.literal_eval(arg) for arg in call.args]
        except (ValueError, TypeError) as exc:
            raise ValueError("Only literal numbers and lists are supported.") from exc
        if call.func.attr == 'move' and len(values) in (1, 2):
            plan.append(Move(*values))
        elif call.func.attr == 'joint' and len(values) in (2, 3):
            plan.append(JointMove(*values))
        elif call.func.attr == 'wait' and len(values) == 1:
            plan.append(Wait(*values))
        else:
            raise ValueError("Unknown command or wrong number of arguments.")
    return plan


def program(plan):
    lines = []
    for command in plan:
        if isinstance(command, Move):
            lines.append(f'robot.move({list(command.target)!r}, {command.speed})')
        elif isinstance(command, JointMove):
            lines.append(f'robot.joint({command.joint_id}, {command.angle!r}, {command.speed})')
        else:
            lines.append(f'robot.wait({command.seconds!r})')
    return '\n'.join(lines)+'\n'


def preview(plan, start):
    """Joint-space interpolation for plotting; no physical or collision model."""
    current = angles(start)
    frames = [current]
    for command in plan:
        if isinstance(command, (Move, JointMove)):
            target = next_pose(current, command)
            count = max(1, math.ceil(max(abs(a-b) for a, b in zip(current, target))/2))
            frames.extend(tuple(a+(b-a)*i/count for a, b in zip(current, target))
                          for i in range(1, count+1))
            current = target
    return frames


def target_plan(start, target):
    """Split a requested target into steps of at most 5° per joint."""
    start, target = angles(start), angles(target)
    count = max(1, math.ceil(max(abs(a-b) for a,b in zip(start, target))/5))
    return [Move(tuple(a+(b-a)*i/count for a,b in zip(start, target)), 5)
            for i in range(1, count+1)]
