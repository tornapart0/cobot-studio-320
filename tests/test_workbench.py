import json
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch
from cobot_studio.core import Move, Wait, angles, parse_program, preview, program
from cobot_studio.hardware import Controller
from cobot_studio.rl import train, rollout, load_policy, policy_plan


class FakeArm:
    def __init__(self, *args, **kwargs):
        self.q = [0]*6
        self.sent = []
        self.stopped = False
        self.closed = False

    def get_angles(self):
        return self.q

    def send_angles(self, q, speed):
        self.sent.append((q, speed))
        self.q = q

    def stop(self):
        self.stopped = True

    def close(self):
        self.closed = True


class FastCancel:
    def is_set(self):
        return False

    def wait(self, seconds):
        return False

    def set(self):
        pass


class CoreTests(unittest.TestCase):
    def test_rejects_non_finite_and_limits(self):
        for q in ([float('nan')]*6, [float('inf')]*6, [True]*6,
                  [0]*5, [0, 136, 0, 0, 0, 0]):
            with self.assertRaises(ValueError):
                angles(q)

    def test_editor_never_executes_code(self):
        for source in ('import os', "__import__('os').system('echo BAD')",
                       'robot.move([0]*6)', 'while True: pass',
                       'robot.move([0,0,0,0,0,0], 100)', 'robot.wait(-1)'):
            with self.assertRaises((ValueError, SyntaxError)):
                parse_program(source)

    def test_roundtrip_and_preview(self):
        plan = [Move((5, 0, 0, 0, 0, 0)), Wait(1)]
        self.assertEqual(parse_program(program(plan)), plan)
        frames = preview(plan, (0,)*6)
        self.assertEqual(frames[-1], plan[0].target)
        self.assertTrue(all(max(abs(a-b) for a,b in zip(x,y)) <= 2
                            for x,y in zip(frames, frames[1:])))

    def test_policy_training_and_json(self):
        policy, history = train(iterations=40)
        self.assertGreater(policy['final_reward'], policy['initial_reward'])
        self.assertEqual(train(iterations=40)[0], policy)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'policy.json'
            path.write_text(json.dumps(policy))
            self.assertEqual(load_policy(path), policy)
            path.write_text('{"schema":"cobot-joint-gains-v1","gains":[1,1,1,1,1,NaN]}')
            with self.assertRaises(ValueError):
                load_policy(path)
        plan = policy_plan(policy, (0,)*6, (10,)*6)
        self.assertLess(max(abs(10-x) for x in plan[-1].target), 1)
        self.assertTrue(all(abs(x) <= 2 for x in plan[0].target))

    def test_feedback_and_stop(self):
        controller = Controller(FakeArm)
        controller.connect('/dev/cu.fake')
        device = controller.robot
        controller.run([Move((5,)*6)], FastCancel())
        self.assertEqual(len(device.sent), 1)
        with self.assertRaises(ValueError):
            controller.run([Move((30,)*6)], threading.Event())
        self.assertTrue(device.stopped)
        controller.close()
        self.assertTrue(device.closed)

    def test_cancel_prevents_motion(self):
        controller = Controller(FakeArm)
        controller.connect('/dev/cu.fake')
        cancel = threading.Event()
        cancel.set()
        controller.run([Move((5,)*6)], cancel)
        self.assertEqual(controller.robot.sent, [])
        self.assertTrue(controller.robot.stopped)

    def test_invalid_feedback_closes_connection(self):
        class BadArm(FakeArm):
            def get_angles(self):
                return -1
        device = BadArm()
        controller = Controller(lambda *a, **kw: device)
        with self.assertRaises(ValueError):
            controller.connect('/dev/cu.fake')
        self.assertTrue(device.closed)
        self.assertIsNone(controller.robot)

    def test_missing_feedback_stops_motion(self):
        controller = Controller(FakeArm)
        controller.connect('/dev/cu.fake')
        controller.robot.get_angles = lambda: None
        with self.assertRaises(ValueError):
            controller.run([Move((5,)*6)], threading.Event())
        self.assertTrue(controller.robot.stopped)
        self.assertFalse(controller.robot.sent)

    def test_arrival_timeout_stops(self):
        controller = Controller(FakeArm)
        controller.connect('/dev/cu.fake')
        controller.robot.send_angles = lambda *a: None
        with patch('cobot_studio.hardware.time.monotonic', side_effect=[0, 21]):
            with self.assertRaises(TimeoutError):
                controller.run([Move((5,)*6)], FastCancel())
        self.assertTrue(controller.robot.stopped)


if __name__ == '__main__':
    unittest.main()
