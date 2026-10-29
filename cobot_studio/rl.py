"""Reward-driven random-search training on a synthetic joint reaching task.

Six learned feedback gains; not a dynamics simulator or production RL policy.
JSON is used instead of executable model serialization.
"""
import json
import math
import random
from .core import angles, Move


def rollout(gains, start, goal, steps=60):
    q = list(angles(start))
    goal = angles(goal)
    path = [tuple(q)]
    reward = 0.0
    for _ in range(steps):
        action = [max(-2, min(2, k*(g-x))) for k, g, x in zip(gains, goal, q)]
        q = [x+a for x, a in zip(q, action)]
        reward -= sum((g-x)**2 + 0.05*a*a for g, x, a in zip(goal, q, action))
        path.append(tuple(q))
    return reward, path


def train(seed=7, iterations=100, start=None, goal=None):
    rng = random.Random(seed)
    tasks = [(tuple(rng.uniform(-30, 30) for _ in range(6)),
              tuple(rng.uniform(-30, 30) for _ in range(6))) for _ in range(8)]
    route = None
    if start is not None or goal is not None:
        start, goal = angles(start), angles(goal)
        route = (start, goal)
        tasks = [route]
    horizon = route_steps(*route) if route else 60
    def score(gains):
        return sum(rollout(gains, a, b, steps=horizon)[0] for a, b in tasks)/len(tasks)
    best = [0.02]*6
    initial = score(best)
    history = [initial]
    for _ in range(iterations):
        candidate = [max(0.001, min(1.0, k+rng.gauss(0, 0.08))) for k in best]
        value = score(candidate)
        if value > history[-1]:
            best = candidate
        history.append(max(value, history[-1]))
    policy = {'schema': 'cobot-joint-gains-v1', 'gains': best, 'seed': seed,
              'initial_reward': initial, 'final_reward': history[-1]}
    if route:
        policy.update(start=list(start), goal=list(goal))
    return policy, history


def validate_policy(data):
    if not isinstance(data, dict) or data.get('schema') != 'cobot-joint-gains-v1':
        raise ValueError('Unsupported policy schema. Use a policy trained here.')
    gains = data.get('gains')
    if not isinstance(gains, list) or len(gains) != 6 or any(
        type(k) not in (int, float) or not math.isfinite(k) or not 0 < k <= 1 for k in gains
    ):
        raise ValueError('Policy needs six finite gains in (0, 1].')
    if 'start' in data or 'goal' in data:
        angles(data.get('start'))
        angles(data.get('goal'))
    return data


def load_policy(path):
    with open(path) as handle:
        return validate_policy(json.load(handle))


def policy_plan(policy, start, goal):
    validate_policy(policy)
    start, goal = angles(start), angles(goal)
    _, path = rollout(policy['gains'], start, goal, steps=route_steps(start, goal))
    if max(abs(a-b) for a,b in zip(path[-1], goal)) > 0.5:
        raise ValueError('Policy has not reached B within 0.5°. Train again before pushing.')
    return [Move(q, 5) for q in path[1:]] + [Move(goal, 5)]


def route_steps(start, goal):
    return max(60, math.ceil(max(abs(a-b) for a,b in zip(start, goal))/2)+80)
