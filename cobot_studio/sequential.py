"""Reward-based route-order search: at most one final-angle command per joint.

No collision model or physical learning. Retraining reuses the best order found.
"""
import random
import math
from .core import angles, JointMove
from .kinematics import MODEL_ID

SCHEMA = 'cobot-sequential-v2'


def validate(data):
    if not isinstance(data, dict) or data.get('schema') != SCHEMA or data.get('model') != MODEL_ID:
        raise ValueError('Train a new policy for the 2022 M5 model.')
    angles(data.get('start'))
    angles(data.get('goal'))
    order = data.get('order')
    if not isinstance(order, list) or any(type(i) is not int for i in order) or sorted(order) != list(range(1,7)):
        raise ValueError('Policy order must contain J1–J6 exactly once.')
    if type(data.get('iterations')) is not int or not 0 <= data['iterations'] <= 1000000:
        raise ValueError('Invalid training iteration count.')
    for key in ('initial_reward', 'final_reward'):
        value = data.get(key)
        if type(value) not in (int,float) or not math.isfinite(value):
            raise ValueError('Invalid training reward.')
    return data


def score(order, start, goal):
    """Negative integrated squared joint error over nominal constant-rate moves.

The time axis is relative joint travel, not a physical actuator prediction.
"""
    q = list(start)
    cost = 0.0
    for joint in order:
        i = joint-1
        distance = abs(goal[i]-q[i])
        other_error = sum((goal[k]-q[k])**2 for k in range(6) if k != i)
        cost += distance*(other_error+(goal[i]-q[i])**2/3)
        q[i] = goal[i]
    return -cost


def train_route(start, goal, iterations=100, previous=None, cancel=None):
    start, goal = angles(start), angles(goal)
    if type(iterations) is not int or not 1 <= iterations <= 1000:
        raise ValueError('Use 1–1000 training iterations per batch.')
    completed = 0
    best = list(range(1,7))
    if previous is not None:
        validate(previous)
        if previous['start'] != list(start) or previous['goal'] != list(goal):
            raise ValueError('Previous policy belongs to another route.')
        best = previous['order'][:]
        completed = previous['iterations']
    rng = random.Random(7+completed)
    initial = score(best, start, goal)
    history = [initial]
    for _ in range(iterations):
        if cancel is not None and cancel.is_set():
            raise ValueError('Training cancelled.')
        candidate = rng.sample(range(1,7), 6)
        value = score(candidate, start, goal)
        if value > history[-1]:
            best = candidate
        history.append(max(value, history[-1]))
    policy = {'schema': SCHEMA, 'model': MODEL_ID, 'start': list(start), 'goal': list(goal),
              'order': best, 'iterations': completed+iterations,
              'initial_reward': previous.get('initial_reward', initial) if previous else initial,
              'final_reward': history[-1]}
    return policy, history


def route_plan(policy, speed=5):
    validate(policy)
    return [JointMove(j, policy['goal'][j-1], speed) for j in policy['order']
            if abs(policy['goal'][j-1]-policy['start'][j-1]) > 0.000001]
