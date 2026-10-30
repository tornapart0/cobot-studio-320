import threading
import unittest
from unittest.mock import patch
from cobot_studio.sequential import train_route, route_plan
from cobot_studio.core import JointMove, next_pose
from cobot_studio.hardware import Controller
from cobot_studio.kinematics import forward, endpoint
from test_workbench import FakeArm, FastCancel


class SingleJointArm(FakeArm):
    def send_angle(self, joint_id, angle, speed):
        self.sent.append((joint_id, angle, speed))
        self.q = list(self.q)
        self.q[joint_id-1] = angle


class SequentialTests(unittest.TestCase):
    def test_continue_training_preserves_or_improves_reward(self):
        a, b = (0,)*6, (15, -10, 20, -5, 10, 30)
        first, _ = train_route(a,b,40)
        more, _ = train_route(a,b,60,previous=first)
        self.assertEqual(more['iterations'], 100)
        self.assertGreaterEqual(more['final_reward'], first['final_reward'])
        with self.assertRaises(ValueError):
            train_route((1,)*6,b,10,previous=more)

    def test_six_final_commands_and_speed_modifier(self):
        controller = Controller(SingleJointArm)
        controller.connect('/dev/cu.fake')
        goal = (20, 25, -30, 40, -15, 50)
        policy, _ = train_route((0,)*6,goal)
        result = controller.run(route_plan(policy), FastCancel(), required_start=(0,)*6,
                                speed_modifier=1.6, expected_goal=goal)
        self.assertEqual(len(controller.robot.sent), 6)
        self.assertTrue(all(command[2] == 8 for command in controller.robot.sent))
        self.assertTrue(result['matched'])
        self.assertEqual(result['measured'], list(goal))

    def test_no_extra_commands_when_feedback_stalls(self):
        controller = Controller(SingleJointArm)
        controller.connect('/dev/cu.fake')
        controller.robot.send_angle = lambda *command: controller.robot.sent.append(command)
        with patch('cobot_studio.hardware.time.monotonic', side_effect=[0, 1000]):
            with self.assertRaises(TimeoutError):
                controller.run([JointMove(2, 20, 5)], FastCancel())
        self.assertEqual(len(controller.robot.sent), 1)
        self.assertTrue(controller.robot.stopped)

    def test_unmatched_goal_not_reported_as_success(self):
        controller = Controller(SingleJointArm)
        controller.connect('/dev/cu.fake')
        self.assertFalse(controller.verify((10,)*6)['matched'])
        self.assertEqual(controller.robot.sent, [])

    def test_unchanged_joints_are_omitted(self):
        policy, _ = train_route((0,)*6, (0, 15, 0, 0, -5, 0))
        self.assertEqual({m.joint_id for m in route_plan(policy)}, {2,5})
        policy, _ = train_route((0,)*6, (0,)*6)
        self.assertEqual(route_plan(policy), [])

    def test_joint_frames_and_directionality(self):
        zero_points, zero_axes, _ = forward((0,)*6)
        # J1 is base +Z. A positive J1 rotates the first offset from -Y toward +X.
        points, axes, _ = forward((90,0,0,0,0,0))
        self.assertAlmostEqual(points[1][0], 88.78, places=2)
        self.assertAlmostEqual(points[1][1], 0, places=2)
        self.assertAlmostEqual(zero_axes[0][2], 1)
        # Joint rotation does not move its own origin or any preceding joint.
        for joint in range(6):
            q = [0]*6
            q[joint] = 10
            moved, _, _ = forward(q)
            for i in range(joint+1):
                for a,b in zip(moved[i],zero_points[i]):
                    self.assertAlmostEqual(a,b)
