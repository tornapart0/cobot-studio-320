import math
import unittest
from cobot_studio.kinematics import forward, link_paths, rotation_arc


class LinkShapeTests(unittest.TestCase):
    def test_rigid_links_have_right_angle_bends_and_correct_joint_ends(self):
        for q in ((0,)*6, (30, -45, 60, 15, -30, 80)):
            paths = link_paths(q)
            joints, _, _ = forward(q)
            for i, path in enumerate(paths):
                for a,b in zip(path[-1],joints[i]):
                    self.assertAlmostEqual(a,b)
                for a,b,c in zip(path,path[1:],path[2:]):
                    u = [x-y for x,y in zip(b,a)]
                    v = [x-y for x,y in zip(c,b)]
                    self.assertAlmostEqual(sum(x*y for x,y in zip(u,v)), 0, places=6)

    def test_link_segment_lengths_survive_joint_rotation(self):
        a = link_paths((0,)*6)
        b = link_paths((25, -40, 35, 60, -25, 80))
        for original, rotated in zip(a,b):
            self.assertEqual(len(original),len(rotated))
            for i in range(len(original)-1):
                self.assertAlmostEqual(math.dist(original[i],original[i+1]),
                                       math.dist(rotated[i],rotated[i+1]))

    def test_rotation_arrows_stay_normal_to_joint_axes(self):
        joints, axes, _ = forward((20, -30, 10, 50, 40, 80))
        for p, axis in zip(joints,axes):
            for point in rotation_arc(p,axis):
                relative = [x-y for x,y in zip(point,p)]
                self.assertAlmostEqual(sum(x*y for x,y in zip(relative,axis)),0,places=6)
                self.assertAlmostEqual(math.dist(point,p),20)
