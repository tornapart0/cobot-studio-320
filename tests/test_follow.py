import math
import threading
import unittest
from unittest.mock import patch
from cobot_studio.follow import FollowMapper, wrap
from cobot_studio.mirror import HandSample
from cobot_studio.core import LIMITS
from cobot_studio.hardware import Controller
from test_sequential import SingleJointArm


def hand(now=0,x=.5,y=.5,size=0,roll=0):
    return HandSample(now,(x,y,math.log(.2)+size,roll,0,0),.99,'Right')


def image_error(mapper,point,direction,q):
    f=mapper.project(point,direction,q,4/3)
    return (f[0]-.5)**2+(f[1]-.5)**2+(f[2]-mapper.neutral.features[2])**2+wrap(f[3]-mapper.neutral.features[3])**2


class FollowTests(unittest.TestCase):
    def test_centered_hand_has_no_drift(self):
        mapper=FollowMapper(hand(),(0,)*6)
        for i in range(30):
            self.assertEqual(mapper.target(hand(i/10),i/10),(0,)*6)

    def test_virtual_camera_converges_for_translation_roll_and_distance(self):
        for features in (dict(x=.6),dict(y=.6),dict(roll=.3),dict(size=.2)):
            mapper=FollowMapper(hand(),(0,-30,50,0,0,0),follow_distance=True)
            observed=hand(**features)
            point,direction=mapper.observation(observed,mapper.anchor)
            before=image_error(mapper,point,direction,mapper.anchor)
            for step in range(160):
                now=step/10
                q=mapper.target(hand(now,**features),now)
            self.assertLess(image_error(mapper,point,direction,q),before*.3)
            self.assertGreater(sum(abs(a-b)>.05 for a,b in zip(q,mapper.anchor)),1)
            self.assertTrue(all(abs(a)<=b for a,b in zip(q,LIMITS)))

    def test_mount_offsets_and_rotation_are_used(self):
        mapper=FollowMapper(hand(),(0,-30,50,0,0,0),mount_rpy=(0,90,90),mount_xyz=(20,0,10),image_rotation=90)
        point,direction=mapper.observation(hand(x=.6),mapper.anchor)
        before=image_error(mapper,point,direction,mapper.anchor)
        for step in range(120):
            now=step/10
            q=mapper.target(hand(now,x=.6),now)
        self.assertLess(image_error(mapper,point,direction,q),before*.4)

    def test_distance_is_optional_and_stale_tracking_rejected(self):
        mapper=FollowMapper(hand(),(0,)*6,follow_distance=False)
        for step in range(20):
            self.assertEqual(mapper.target(hand(step/10,size=.3),step/10),(0,)*6)
        with self.assertRaises(ValueError):
            mapper.target(hand(),3)
        with self.assertRaises(ValueError):
            FollowMapper(hand(),(0,)*6,mount_rpy=(float('nan'),0,0))

    def test_hardware_uses_measured_pose_emits_small_steps_and_stops(self):
        controller=Controller(SingleJointArm)
        controller.connect('/dev/cu.fake')
        mapper=FollowMapper(hand(),(0,)*6)
        point,direction=mapper.observation(hand(x=.65),(0,)*6)
        clock=[0]
        def now():
            clock[0]+=.025
            return clock[0]
        class Camera:
            def sample(self):
                f=mapper.project(point,direction,controller.robot.q,4/3)
                return HandSample(clock[0],(*f,0,0),.99,'Right')
        run=threading.Event()
        run.set()
        class Cancel:
            ticks=0
            def is_set(self): return self.ticks>=100
            def wait(self,delay): self.ticks+=1
        initial=image_error(mapper,point,direction,(0,)*6)
        with patch('cobot_studio.hardware.time.monotonic',side_effect=now):
            controller.mirror(Camera(),hand(),run,Cancel(),lambda q:None,(True,)*6,reference_pose=(0,)*6,mapping_mode='Wrist camera follow',mapping_state=mapper)
        self.assertGreater(len(controller.robot.sent),0)
        self.assertTrue(controller.robot.stopped)
        self.assertFalse(run.is_set())
        self.assertFalse(mapper.hardware)
        self.assertLess(image_error(mapper,point,direction,controller.robot.q),initial)
        previous=[0]*6
        for joint,value,speed in controller.robot.sent:
            self.assertLessEqual(abs(value-previous[joint-1]),.500001)
            self.assertEqual(speed,3)
            previous[joint-1]=value

    def test_tracking_loss_stops_before_any_command(self):
        controller=Controller(SingleJointArm)
        controller.connect('/dev/cu.fake')
        mapper=FollowMapper(hand(),(0,)*6)
        run=threading.Event()
        run.set()
        class Camera:
            def sample(self): raise ValueError('Tracking lost')
        with self.assertRaisesRegex(ValueError,'Tracking lost'):
            controller.mirror(Camera(),hand(),run,threading.Event(),lambda q:None,(True,)*6,mapping_state=mapper)
        self.assertFalse(run.is_set())
        self.assertTrue(controller.robot.stopped)
        self.assertEqual(controller.robot.sent,[])
        self.assertFalse(mapper.hardware)

    def test_unmoving_feedback_times_out(self):
        controller=Controller(SingleJointArm)
        controller.connect('/dev/cu.fake')
        controller.robot.send_angle=lambda *values: controller.robot.sent.append(values)
        mapper=FollowMapper(hand(),(0,)*6)
        run=threading.Event()
        run.set()
        clock=[0]
        def now():
            clock[0]+=.08
            return clock[0]
        class Camera:
            def sample(self): return hand(clock[0],x=.7)
        class Cancel:
            ticks=0
            def is_set(self): return self.ticks>200
            def wait(self,delay): self.ticks+=1
        with patch('cobot_studio.hardware.time.monotonic',side_effect=now):
            with self.assertRaises(TimeoutError):
                controller.mirror(Camera(),hand(),run,Cancel(),lambda q:None,(True,)*6,mapping_state=mapper)
        self.assertEqual(len(controller.robot.sent),1)
        self.assertTrue(controller.robot.stopped)
