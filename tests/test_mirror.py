import math
import threading
import unittest
from unittest.mock import patch
from cobot_studio.mirror import HandSample, MirrorMapper, hand_features
from cobot_studio.hardware import Controller
from test_workbench import FakeArm, FastCancel


class MirrorTests(unittest.TestCase):
    def test_wave_maps_to_j4_with_deadband_and_rate_bound(self):
        neutral = HandSample(0,(0,)*6,0.99,'Right')
        mapper = MirrorMapper(neutral,(0,)*6)
        last = (0,)*6
        for step in range(21):
            now = step/10
            sample = HandSample(now,(0,0,0,0.5,0,0),0.99,'Right')
            target = mapper.target(sample,now)
            self.assertLessEqual(abs(target[3]-last[3]),0.300001)
            self.assertTrue(all(target[i] == 0 for i in (0,1,2,4,5)))
            last = target
        self.assertGreater(last[3],0)
        self.assertLessEqual(last[3],10)

    def test_tracking_loss_staleness_and_hand_switch_are_rejected(self):
        neutral = HandSample(0,(0,)*6,0.99,'Right')
        for sample in (HandSample(0,(0,)*6,0.99,'Right'),
                       HandSample(1,(0,)*6,0.1,'Right'),
                       HandSample(1,(0,)*6,0.99,'Left'),
                       HandSample(2,(0,)*6,0.99,'Right'),
                       HandSample(1,(float('nan'),)*6,0.99,'Right')):
            with self.assertRaises(ValueError):
                MirrorMapper(neutral,(0,)*6).target(sample,1)

    def test_orientation_wrap_does_not_create_large_swing(self):
        neutral = HandSample(0,(0,0,0,math.pi-0.01,0,0),0.99,'Right')
        mapper = MirrorMapper(neutral,(0,)*6)
        for i in range(20):
            now = i/10
            target = mapper.target(HandSample(now,(0,0,0,-math.pi+0.01,0,0),0.99,'Right'),now)
        self.assertLess(abs(target[3]),0.5)

    def test_inversion_and_travel_envelope(self):
        neutral = HandSample(0,(0,)*6,0.99,'Right')
        mapper = MirrorMapper(neutral,(160,130,140,140,160,170),signs=(-1,)*6)
        for i in range(100):
            now = i/10
            target = mapper.target(HandSample(now,(10,10,10,1,1,1),0.99,'Right'),now)
            self.assertTrue(all(abs(a-b)<=10.00001 for a,b in zip(target,mapper.anchor)))
        self.assertLess(target[3],mapper.anchor[3])

    def test_no_hand_fails_before_any_hardware_command(self):
        arm = Controller(FakeArm)
        arm.connect('/dev/cu.fake')
        class MissingCamera:
            def sample(self):
                raise ValueError('No hand')
        hold = threading.Event()
        hold.set()
        with self.assertRaises(ValueError):
            arm.mirror(MissingCamera(),HandSample(0,(0,)*6,0.99,'Right'),hold,FastCancel(),lambda q: None,(False,False,False,True,False,False))
        self.assertTrue(arm.robot.stopped)
        self.assertEqual(arm.robot.sent,[])
        self.assertFalse(hold.is_set())

    def test_hold_release_stops_and_only_selected_joint_is_sent(self):
        from test_sequential import SingleJointArm
        hold = threading.Event()
        hold.set()
        arm = Controller(SingleJointArm)
        arm.connect('/dev/cu.fake')
        clock = [0]
        def now():
            clock[0] += 0.05
            return clock[0]
        class Camera:
            def sample(self):
                return HandSample(clock[0],(0.2,0.2,0.2,1,1,1),0.99,'Right')
        original = arm.robot.send_angle
        def send(joint,value,speed):
            original(joint,value,speed)
            hold.clear()
        arm.robot.send_angle = send
        with patch('cobot_studio.hardware.time.monotonic',side_effect=now):
            arm.mirror(Camera(),HandSample(0,(0,)*6,0.99,'Right'),hold,FastCancel(),lambda q: None,(False,False,False,True,False,False))
        self.assertEqual(len(arm.robot.sent),1)
        joint,value,speed = arm.robot.sent[0]
        self.assertEqual(joint,4)
        self.assertLessEqual(abs(value),0.5)
        self.assertEqual(speed,3)
        self.assertTrue(arm.robot.stopped)

    def test_disabled_motors_block_commands_and_report_cause(self):
        arm = Controller(FakeArm)
        arm.connect('/dev/cu.fake')
        arm.robot.is_power_on = lambda: 0
        hold = threading.Event()
        hold.set()
        with self.assertRaisesRegex(ValueError,'motor power'):
            arm.mirror(None,HandSample(0,(0,)*6,0.99,'Right'),hold,FastCancel(),lambda q: None,(False,False,False,True,False,False))
        self.assertEqual(arm.robot.sent,[])
        self.assertTrue(arm.robot.stopped)
        self.assertFalse(hold.is_set())

    def test_landmarks_produce_finite_features(self):
        points = [(0.5,0.6,0.0)]*21
        points[0]=(0.5,0.7,0)
        points[5]=(0.4,0.5,0)
        points[17]=(0.6,0.5,0)
        result = hand_features(points,points)
        self.assertEqual(len(result),6)
        self.assertTrue(all(math.isfinite(x) for x in result))
