"""Display-only interpolation. Never changes exported or hardware waypoints."""
import math


def sample(frames, position, smoothness):
    if not frames:
        raise ValueError('No simulation frames.')
    position = max(0.0, min(float(position), len(frames)-1))
    i = min(int(position), max(0, len(frames)-2))
    if len(frames) == 1:
        return tuple(frames[0])
    t = position-i
    blend = max(0.0, min(1.0, smoothness))
    result = []
    for joint in range(6):
        a, b = frames[i][joint], frames[i+1][joint]
        delta = b-a
        before = a-frames[i-1][joint] if i else delta
        after = frames[i+2][joint]-b if i+2 < len(frames) else delta
        def tangent(left, right):
            return 2*left*right/(left+right) if left*right > 0 else 0.0
        m0, m1 = tangent(before, delta), tangent(delta, after)
        cubic = (2*t**3-3*t*t+1)*a+(t**3-2*t*t+t)*m0+(-2*t**3+3*t*t)*b+(t**3-t*t)*m1
        linear = a+delta*t
        # Preserve the interval bounds even at numerical extremes.
        result.append(max(min(a,b), min(max(a,b), linear+blend*(cubic-linear))))
    return tuple(result)
