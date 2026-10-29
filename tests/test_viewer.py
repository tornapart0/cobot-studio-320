import math
import unittest
from cobot_studio.core import target_plan, LIMITS
from cobot_studio.viewer import joint_points


class ViewerWorkflowTests(unittest.TestCase):
    def test_zero_is_fully_upright(self):
        points = joint_points((0,)*6)
        for before, after in zip(points, points[1:]):
            self.assertAlmostEqual(after[0], 0)
            self.assertAlmostEqual(after[1], 0)
            self.assertGreater(after[2], before[2])
        self.assertAlmostEqual(points[-1][2]-points[0][2],
                               sum(math.dist(a,b) for a,b in zip(points, points[1:])))

    def test_large_slider_target_becomes_bounded_hardware_steps(self):
        start = (-160, -130, -140, -140, -160, -170)
        target = (160, 130, 140, 140, 160, 170)
        plan = target_plan(start, target)
        self.assertEqual(plan[-1].target, target)
        previous = start
        for move in plan:
            self.assertLessEqual(max(abs(a-b) for a,b in zip(previous, move.target)), 5.000001)
            previous = move.target

    def test_chain_keeps_links_rigid_across_joint_changes(self):
        neutral = joint_points((0,)*6)
        rotated = joint_points((70, -50, 40, 30, 20, -90))
        self.assertEqual(len(rotated), 7)
        self.assertNotEqual(rotated[-1], neutral[-1])
        for i in range(6):
            self.assertAlmostEqual(math.dist(neutral[i], neutral[i+1]),
                                   math.dist(rotated[i], rotated[i+1]))
