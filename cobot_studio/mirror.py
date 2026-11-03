"""Camera-relative gesture mapping and bounded hand mirroring. No robot or camera I/O."""
import math
from dataclasses import dataclass
from .core import angles, LIMITS

LABELS = ('Palm left/right', 'Palm up/down', 'Near/far (size proxy)',
          'Wave / palm roll', 'Palm tilt', 'Palm turn')
MODES = ('Finger count + twist',)
COUNT_LABELS = ('Fist + twist', '1 finger + twist', '2 fingers + twist', '3 fingers + twist', '4 fingers + twist', '5 fingers + twist')
STALE_SECONDS = 0.35


@dataclass(frozen=True)
class HandSample:
    captured_at: float
    features: tuple
    confidence: float
    handedness: str
    extensions: tuple = ()  # thumb, index, middle, ring, little; values 0–1

    def valid(self, now):
        return (len(self.features) == 6 and all(math.isfinite(x) for x in self.features)
                and math.isfinite(self.captured_at) and 0 <= now-self.captured_at <= STALE_SECONDS
                and math.isfinite(self.confidence) and self.confidence >= 0.8
                and self.handedness in ('Left','Right')
                and (not self.extensions or (len(self.extensions) == 5 and all(math.isfinite(v) and 0 <= v <= 1 for v in self.extensions))))


def hand_features(image_points, world_points):
    """Six relative palm signals, not measured human Cartesian 6-DOF.

    Position is camera-normalized; apparent palm width proxies distance. Roll
    follows palm span; pitch/yaw use the estimated 3D palm normal.
    """
    if len(image_points) != 21 or len(world_points) != 21:
        raise ValueError('Expected 21 hand landmarks.')
    if any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in image_points+world_points):
        raise ValueError('Invalid hand landmarks.')
    wrist, index, pinky = image_points[0], image_points[5], image_points[17]
    width = math.hypot(index[0]-pinky[0], index[1]-pinky[1])
    if width < 0.025:
        raise ValueError('Hand too small or edge-on. Show your palm to the camera.')
    center = [sum(image_points[i][j] for i in (0,5,9,13,17))/5 for j in range(2)]
    a = [world_points[5][j]-world_points[0][j] for j in range(3)]
    b = [world_points[17][j]-world_points[0][j] for j in range(3)]
    normal = (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
    if math.sqrt(sum(x*x for x in normal)) < 1e-6:
        raise ValueError('Unreliable palm orientation.')
    # Periodic angles are differenced with wraparound during mapping.
    roll = math.atan2(pinky[1]-index[1], pinky[0]-index[0])
    pitch = math.atan2(normal[1], math.hypot(normal[0], normal[2]))
    yaw = math.atan2(normal[0], normal[2])
    return (center[0], center[1], math.log(width), roll, pitch, yaw)


class MirrorMapper:
    def __init__(self, neutral, anchor, sensitivity=1.0, signs=(1,1,1,1,1,1)):
        if not isinstance(neutral,HandSample) or len(neutral.features) != 6 or not all(math.isfinite(v) for v in neutral.features):
            raise ValueError('Invalid neutral-hand calibration.')
        self.neutral = neutral
        self.anchor = angles(anchor)
        if len(signs) != 6 or any(v not in (-1,1) for v in signs):
            raise ValueError('Each mapping sign must be +1 or -1.')
        if not math.isfinite(sensitivity) or not 0.2 <= sensitivity <= 2:
            raise ValueError('Sensitivity must be 0.2×–2×.')
        self.signs, self.sensitivity = tuple(signs), sensitivity
        self.filtered = list(self.anchor)
        self.last_time = None

    def target(self, sample, now):
        if not sample.valid(now) or sample.handedness != self.neutral.handedness:
            raise ValueError('Tracking lost, stale or hand changed. Release and recalibrate.')
        differences = [a-b for a,b in zip(sample.features,self.neutral.features)]
        for i in (3,4,5):
            differences[i] = math.atan2(math.sin(differences[i]),math.cos(differences[i]))
        gains = (40, -40, 15, 20, 20, 20)
        dt = 0 if self.last_time is None else max(0,min(0.1, now-self.last_time))
        self.last_time = now
        for i in range(6):
            offset = max(-10, min(10,differences[i]*gains[i]*self.signs[i]*self.sensitivity))
            desired = max(-LIMITS[i],min(LIMITS[i],self.anchor[i]+offset))
            if abs(desired-self.filtered[i]) < 0.25:
                desired = self.filtered[i]
            # Exponential low-pass and independent 3°/s rate bound.
            step = (desired-self.filtered[i])*(1-math.exp(-dt/0.25))
            self.filtered[i] += max(-3*dt,min(3*dt,step))
        return angles(self.filtered)


def finger_extensions(points):
    """Rotation-invariant finger straightness from the 21 estimated 3D points.

    This is geometric gesture recognition, not an anatomical measurement.
    """
    if len(points) != 21 or any(len(p) != 3 or not all(math.isfinite(x) for x in p) for p in points):
        raise ValueError('Expected 21 finite 3D hand landmarks.')
    def angle(a,b,c):
        u = [x-y for x,y in zip(a,b)]
        v = [x-y for x,y in zip(c,b)]
        denominator = math.sqrt(sum(x*x for x in u)*sum(x*x for x in v))
        if denominator < 1e-10:
            raise ValueError('Degenerate finger landmarks.')
        return math.degrees(math.acos(max(-1,min(1,sum(x*y for x,y in zip(u,v))/denominator))))
    def clamp(value):
        return max(0.0,min(1.0,value))
    result = []
    for base,pip,dip,tip in ((1,2,3,4),(5,6,7,8),(9,10,11,12),(13,14,15,16),(17,18,19,20)):
        a = angle(points[base],points[pip],points[dip])
        b = angle(points[pip],points[dip],points[tip])
        result.append(clamp((min(a,b)-100)/65))
    palm_width = math.dist(points[5],points[17])
    if palm_width < 1e-5:
        raise ValueError('Palm landmarks are unreliable.')
    # A straight thumb folded across the palm is not an extended thumb.
    separation = math.dist(points[4],points[5])/palm_width
    result[0] *= clamp((separation-0.35)/0.45)
    return tuple(result)


def finger_count(sample):
    if len(sample.extensions) != 5:
        return None
    if any(0.35 < value < 0.7 for value in sample.extensions):
        return None
    return sum(value >= 0.7 for value in sample.extensions)


def gesture_description(sample, mode=MODES[0]):
    count = finger_count(sample)
    if count is None:
        return 'Unclear finger count · hold a clearer gesture'
    return ('Fist' if count == 0 else f'{count} finger'+('s' if count != 1 else ''))+f' → J{count+1} · twist to move'


class GestureMapper(MirrorMapper):
    """Each gesture has an explicit joint role; count changes never jump poses."""
    def __init__(self, neutral, anchor, sensitivity=1.0, signs=(1,)*6, mode=MODES[0], rotation_axis='Roll'):
        super().__init__(neutral,anchor,sensitivity,signs)
        if mode not in MODES or len(neutral.extensions) != 5 or any(not math.isfinite(x) or not 0 <= x <= 1 for x in neutral.extensions):
            raise ValueError('Restart camera and calibrate with visible finger landmarks.')
        if rotation_axis not in ('Roll','Tilt','Turn'):
            raise ValueError('Choose Roll, Tilt or Turn.')
        self.axis = ('Roll','Tilt','Turn').index(rotation_axis)+3
        self.mode = mode
        self.targets = list(self.anchor)
        self.selected = finger_count(neutral)
        self.candidate = self.selected
        self.candidate_since = neutral.captured_at
        self.roll_origin = neutral.features[self.axis]
        self.joint_origin = self.anchor[self.selected] if self.selected is not None else 0

    def target(self,sample,now):
        if not sample.valid(now) or sample.handedness != self.neutral.handedness or len(sample.extensions) != 5:
            raise ValueError('Finger tracking lost. Show the same hand clearly.')
        dt = 0 if self.last_time is None else max(0,min(0.1,now-self.last_time))
        self.last_time = now
        count = finger_count(sample)
        if count != self.selected or count is None:
            # Freeze every joint while selecting; do not return old joints to zero.
            self.targets = list(self.filtered)
            self.selected = None
            if count != self.candidate or count is None:
                self.candidate = count
                self.candidate_since = now
            elif now-self.candidate_since >= 0.3:
                self.selected = count
                self.roll_origin = sample.features[self.axis]
                self.joint_origin = self.filtered[count]
        if self.selected is not None:
            delta = sample.features[self.axis]-self.roll_origin
            delta = math.atan2(math.sin(delta),math.cos(delta))
            joint = self.selected
            self.targets[joint] = self.joint_origin+math.degrees(delta)*self.signs[joint]*self.sensitivity
        for i in range(6):
            desired = max(self.anchor[i]-10,min(self.anchor[i]+10,self.targets[i]))
            desired = max(-LIMITS[i],min(LIMITS[i],desired))
            if abs(desired-self.filtered[i]) < 0.25:
                desired = self.filtered[i]
            step = (desired-self.filtered[i])*(1-math.exp(-dt/0.25))
            self.filtered[i] += max(-3*dt,min(3*dt,step))
        return angles(self.filtered)
