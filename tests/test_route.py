import threading
import unittest
from cobot_studio.core import Move, JointMove, next_pose, preview, parse_program, program
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
        self.assertLessEqual(len(plan), 6)
        self.assertEqual(len({m.joint_id for m in plan}), len(plan))
        current = start
        for move in plan:
            self.assertIsInstance(move, JointMove)
            updated = next_pose(current, move)
            self.assertEqual(sum(a != b for a,b in zip(current, updated)), 1)
            self.assertEqual(move.angle, goal[move.joint_id-1])
            current = updated
        self.assertEqual(current, goal)
        self.assertEqual(preview(plan,start)[-1], goal)
        self.assertEqual(parse_program(program(plan)), plan)

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
