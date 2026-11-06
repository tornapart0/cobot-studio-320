import math
import unittest
from cobot_studio.mirror import HandSample, GestureMapper, finger_extensions, finger_count, hand_features
from cobot_studio.core import LIMITS


def sample(now,count,roll=0):
    extensions = tuple(1.0 if i<count else 0.0 for i in range(5))
    return HandSample(now,(0,0,0,roll,0,0),0.99,'Right',extensions)


class GestureTests(unittest.TestCase):
    def test_each_count_controls_exactly_its_joint(self):
        for count in range(6):
            mapper = GestureMapper(sample(0,count),(0,)*6)
            for step in range(25):
                now=step/10
                pose=mapper.target(sample(now,count,0.4),now)
            self.assertGreater(pose[count],0)
            self.assertTrue(all(abs(pose[j])<1e-9 for j in range(6) if j!=count))
            self.assertLessEqual(pose[count],LIMITS[count])

    def test_switch_count_waits_and_does_not_jump(self):
        mapper=GestureMapper(sample(0,0),(0,)*6)
        for step in range(20):
            now=step/10
            old=mapper.target(sample(now,0,0.4),now)
        for now in (2,2.1,2.2):
            changed=mapper.target(sample(now,2,0.9),now)
            self.assertEqual(changed,old)
        changed=mapper.target(sample(2.4,2,0.9),2.4)
        self.assertLessEqual(abs(changed[2]),6.00001)
        for step in range(1,15):
            now=2.4+step/10
            changed=mapper.target(sample(now,2,1.2),now)
        self.assertEqual(changed[0],old[0])
        self.assertGreater(changed[2],0)
        self.assertTrue(all(changed[j]==0 for j in (1,3,4,5)))

    def test_ambiguous_counts_freeze_and_tracking_loss_rejects(self):
        mapper=GestureMapper(sample(0,0),(0,)*6)
        before=mapper.target(sample(0,0),0)
        uncertain=HandSample(0.1,(0,0,0,1,0,0),0.99,'Right',(0.5,)*5)
        self.assertIsNone(finger_count(uncertain))
        self.assertEqual(mapper.target(uncertain,0.1),before)
        with self.assertRaises(ValueError):
            mapper.target(sample(0,0),1)

    def test_twist_direction_and_slew_bound(self):
        mapper=GestureMapper(sample(0,1),(0,)*6,signs=(-1,)*6)
        previous=(0,)*6
        for step in range(20):
            now=step/10
            pose=mapper.target(sample(now,1,0.6),now)
            self.assertLessEqual(abs(pose[1]-previous[1]),6.000001)
            previous=pose
        self.assertLess(pose[1],0)

    def test_finger_detection_is_rotation_invariant(self):
        points=[(0,0,0)]*21
        for base,pip,dip,tip,x in ((5,6,7,8,-0.03),(9,10,11,12,-0.01),(13,14,15,16,0.01),(17,18,19,20,0.03)):
            points[base]=(x,0.02,0)
            points[pip]=(x,0.04,0)
            points[dip]=(x,0.06,0)
            points[tip]=(x,0.08,0)
        for i,p in zip((1,2,3,4),((-0.035,0.005,0),(-0.05,0.02,0),(-0.065,0.035,0),(-0.08,0.05,0))):
            points[i]=p
        extended=finger_extensions(points)
        self.assertTrue(all(x>0.9 for x in extended))
        from cobot_studio.viewer import rotate
        rotated=[tuple(x+y for x,y in zip(rotate(p,'x',65),(1,2,3))) for p in points]
        for a,b in zip(extended,finger_extensions(rotated)):
            self.assertAlmostEqual(a,b)
        for base,pip,dip,tip,x in ((5,6,7,8,-0.03),(9,10,11,12,-0.01),(13,14,15,16,0.01),(17,18,19,20,0.03)):
            points[dip]=(x,0.04,0.02)
            points[tip]=(x,0.02,0.02)
        folded=finger_extensions(points)
        self.assertTrue(all(x<0.1 for x in folded[1:]))

    def test_rotation_axis_and_full_scale(self):
        for axis,index in (('Roll',3),('Tilt',4),('Turn',5)):
            mapper=GestureMapper(sample(0,0),(0,)*6,rotation_axis=axis)
            for step in range(101):
                features=[0.0]*6
                features[index]=math.radians(5)
                now=step/10
                pose=mapper.target(HandSample(now,tuple(features),.99,'Right',(0,)*5),now)
            self.assertAlmostEqual(pose[0],LIMITS[0]*5/90,delta=.26)
            self.assertEqual(pose[1:],(0,)*5)

    def test_full_left_right_and_zero_for_every_joint(self):
        for count in range(6):
            mapper=GestureMapper(sample(0,count),(20,)*6)
            now=0
            for rotation,expected in ((math.pi/2,LIMITS[count]),(0,0),(-math.pi/2,-LIMITS[count])):
                for _ in range(180):
                    now+=.1
                    pose=mapper.target(sample(now,count,rotation),now)
                self.assertAlmostEqual(pose[count],expected,delta=.26)
                self.assertTrue(all(pose[j]==20 for j in range(6) if j!=count))

    def test_image_direction_matches_reference_poses_and_aspect_ratio(self):
        # Synthetic landmark geometry reproduces upright, image-left and
        # image-right poses. Finger tips are irrelevant to the palm axis.
        base=[(.5,.5,0)]*21
        base[0]=(.5,.7,0)
        for i,x in ((5,.4),(9,.47),(13,.53),(17,.6)):
            base[i]=(x,.5,0)
        def rotate(points,theta,aspect):
            result=[]
            for x,y,z in points:
                x,y=(x-.5)*aspect,y-.7
                result.append((.5+(x*math.cos(theta)-y*math.sin(theta))/aspect,
                               .7+x*math.sin(theta)+y*math.cos(theta),z))
            return result
        for aspect in (1,4/3,16/9):
            for rotation,expected in ((0,0),(-math.pi/2,math.pi/2),(math.pi/2,-math.pi/2)):
                points=rotate(base,rotation,aspect)
                self.assertAlmostEqual(hand_features(points,points,aspect)[3],expected)

    def test_hardware_rate_remains_slow_with_full_range_targets(self):
        mapper=GestureMapper(sample(0,0),(0,)*6)
        mapper.max_rate=3
        previous=0
        for step in range(100):
            now=step/10
            pose=mapper.target(sample(now,0,math.pi/2),now)
            self.assertLessEqual(abs(pose[0]-previous),.300001)
            previous=pose[0]
        self.assertGreater(pose[0],10)
