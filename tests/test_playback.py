import unittest
from cobot_studio.playback import sample


class PlaybackTests(unittest.TestCase):
    def test_preserves_waypoints_and_stays_within_each_interval(self):
        frames = [tuple([v]*6) for v in (0, 10, 5, 5, -20)]
        for smooth in (0, 0.5, 1):
            for i, q in enumerate(frames):
                self.assertEqual(sample(frames, i, smooth), q)
            for i in range(len(frames)-1):
                for step in range(21):
                    q = sample(frames, i+step/20, smooth)
                    self.assertTrue(all(min(frames[i][j], frames[i+1][j]) <= v <=
                                        max(frames[i][j], frames[i+1][j]) for j,v in enumerate(q)))

    def test_linear_and_smooth_are_distinct(self):
        frames = [tuple([v]*6) for v in (0, 10, 11)]
        self.assertEqual(sample(frames, 0.5, 0), (5,)*6)
        self.assertNotEqual(sample(frames, 0.5, 0), sample(frames, 0.5, 1))
        self.assertEqual(sample([frames[0]], 99, 1), frames[0])
