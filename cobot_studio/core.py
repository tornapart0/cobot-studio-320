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
    speed: int = 5

    def __post_init__(self):
        object.__setattr__(self, 'target', angles(self.target))
        if type(self.speed) is not int or not 1 <= self.speed <= MAX_SPEED:
            raise ValueError("Workbench speed must be an integer from 1 to 10.")


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
            raise ValueError("Use robot.move([six angles], speed) or robot.wait(seconds).")
        try:
            values = [ast.literal_eval(arg) for arg in call.args]
        except (ValueError, TypeError) as exc:
            raise ValueError("Only literal numbers and lists are supported.") from exc
        if call.func.attr == 'move' and len(values) in (1, 2):
            plan.append(Move(*values))
        elif call.func.attr == 'wait' and len(values) == 1:
            plan.append(Wait(*values))
        else:
            raise ValueError("Unknown command or wrong number of arguments.")
    return plan


def program(plan):
    return '\n'.join(f'robot.move({list(x.target)!r}, {x.speed})' if isinstance(x, Move)
                     else f'robot.wait({x.seconds!r})' for x in plan) + '\n'


def preview(plan, start):
    """Joint-space interpolation for plotting; no physical or collision model."""
    current = angles(start)
    frames = [current]
    for command in plan:
        if isinstance(command, Move):
            count = max(1, math.ceil(max(abs(a-b) for a, b in zip(current, command.target))/2))
            frames.extend(tuple(a+(b-a)*i/count for a, b in zip(current, command.target))
                          for i in range(1, count+1))
            current = command.target
    return frames


def target_plan(start, target):
    """Split a requested target into steps of at most 5° per joint."""
    start, target = angles(start), angles(target)
    count = max(1, math.ceil(max(abs(a-b) for a,b in zip(start, target))/5))
    return [Move(tuple(a+(b-a)*i/count for a,b in zip(start, target)), 5)
            for i in range(1, count+1)]
