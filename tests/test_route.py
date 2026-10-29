import threading
import unittest
from cobot_studio.core import Move
from cobot_studio.hardware import Controller
from cobot_studio.rl import train, policy_plan, validate_policy
from test_workbench import FakeArm, FastCancel


class RouteTests(unittest.TestCase):
    def test_training_uses_and_preserves_selected_route(self):
        start = (-160, -130, 140, 130, 160, -170)
        goal = (160, 130, -140, -130, -160, 170)
        policy, _ = train(start=start, goal=goal)
        self.assertEqual(policy['start'], list(start))
        self.assertEqual(policy['goal'], list(goal))
        plan = policy_plan(policy, start, goal)
        self.assertEqual(plan[-1].target, goal)
        previous = start
        for move in plan:
            self.assertLessEqual(max(abs(a-b) for a,b in zip(previous, move.target)), 2.000001)
            previous = move.target

    def test_missing_endpoint_in_saved_policy_is_rejected(self):
        policy = {'schema': 'cobot-joint-gains-v1', 'gains': [0.5]*6, 'start': [0]*6}
        with self.assertRaises(ValueError):
            validate_policy(policy)

    def test_wrong_hardware_start_sends_no_moves(self):
        controller = Controller(FakeArm)
        controller.connect('/dev/cu.fake')
        with self.assertRaises(ValueError):
            controller.run([Move((5,)*6)], threading.Event(), required_start=(20,)*6)
        self.assertEqual(controller.robot.sent, [])
        self.assertTrue(controller.robot.stopped)

    def test_matching_hardware_start_runs_route(self):
        controller = Controller(FakeArm)
        controller.connect('/dev/cu.fake')
        controller.run([Move((2,)*6)], FastCancel(), required_start=(0,)*6)
        self.assertEqual(len(controller.robot.sent), 1)
