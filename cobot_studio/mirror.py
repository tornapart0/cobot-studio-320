"""Camera-relative gesture mapping and bounded hand mirroring. No robot or camera I/O."""
import math
from dataclasses import dataclass
from .core import angles, LIMITS

LABELS = ('Palm left/right', 'Palm up/down', 'Near/far (size proxy)',
          'Wave / palm roll', 'Palm tilt', 'Palm turn')
STALE_SECONDS = 0.35


@dataclass(frozen=True)
class HandSample:
    captured_at: float
    features: tuple
    confidence: float
    handedness: str

    def valid(self, now):
        return (len(self.features) == 6 and all(math.isfinite(x) for x in self.features)
                and math.isfinite(self.captured_at) and 0 <= now-self.captured_at <= STALE_SECONDS
                and math.isfinite(self.confidence) and self.confidence >= 0.8
                and self.handedness in ('Left','Right'))


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
