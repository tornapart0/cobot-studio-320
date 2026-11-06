"""Wrist-camera visual following using a damped image-feature Jacobian.

No learned depth or absolute human pose is assumed. A nominal palm distance
sets the scale; the measured image closes the loop on the real robot.
"""
import math
from .core import angles, LIMITS
from .kinematics import forward, apply, multiply, rotation


def transpose(matrix):
    return tuple(zip(*matrix))


def wrap(value):
    return math.atan2(math.sin(value),math.cos(value))


def solve(matrix, vector):
    """Small positive-definite damped system, solved without extra packages."""
    rows=[list(row)+[value] for row,value in zip(matrix,vector)]
    n=len(rows)
    for col in range(n):
        pivot=max(range(col,n),key=lambda i:abs(rows[i][col]))
        rows[col],rows[pivot]=rows[pivot],rows[col]
        if abs(rows[col][col])<1e-12:
            raise ValueError('Following calculation is singular. Change the starting pose.')
        scale=rows[col][col]
        rows[col]=[v/scale for v in rows[col]]
        for i in range(n):
            if i!=col:
                scale=rows[i][col]
                rows[i]=[a-scale*b for a,b in zip(rows[i],rows[col])]
    return [row[-1] for row in rows]


class FollowMapper:
    def __init__(self,neutral,anchor,sensitivity=1.0,mount_rpy=(0,0,0),mount_xyz=(0,0,0),distance=350,follow_distance=True,enabled=(True,)*6,image_rotation=0):
        if not neutral.valid(neutral.captured_at):
            raise ValueError('Show one clear palm before setting the following reference.')
        self.anchor=angles(anchor)
        if not math.isfinite(sensitivity) or not .2<=sensitivity<=2:
            raise ValueError('Sensitivity must be 0.2–2.')
        if len(mount_rpy)!=3 or len(mount_xyz)!=3 or not all(math.isfinite(v) for v in (*mount_rpy,*mount_xyz)):
            raise ValueError('Camera mount needs three finite angles and offsets.')
        if any(abs(v)>180 for v in mount_rpy) or any(abs(v)>200 for v in mount_xyz):
            raise ValueError('Mount angles must be within ±180° and offsets within ±200 mm.')
        if not math.isfinite(distance) or not 100<=distance<=1000:
            raise ValueError('Approximate palm distance must be 100–1000 mm.')
        if len(enabled)!=6 or not any(enabled):
            raise ValueError('Allow at least one joint for following.')
        self.neutral,self.sensitivity=neutral,sensitivity
        r,p,y=map(math.radians,mount_rpy)
        self.mount=multiply(multiply(rotation('z',y),multiply(rotation('y',p),rotation('x',r))),rotation('z',math.radians(-image_rotation)))
        self.offset=tuple(mount_xyz)
        self.distance=distance
        self.follow_distance=follow_distance
        self.enabled=tuple(enabled)
        self.filtered=list(self.anchor)
        self.max_rate=30.0
        self.last_time=None
        self.hardware=False
        self.status='Reference saved'

    def camera_pose(self,q):
        points,_,orientation=forward(q)
        offset=apply(orientation,self.offset)
        return tuple(a+b for a,b in zip(points[-1],offset)),multiply(orientation,self.mount)

    def observation(self,sample,q):
        """Place an approximate palm point and direction in the robot frame."""
        position,orientation=self.camera_pose(q)
        z=self.distance*math.exp(max(-1,min(1,self.neutral.features[2]-sample.features[2])))
        # Approximate 60° vertical FOV; dimensions are gain estimates, not depth measurements.
        fy=1/(2*math.tan(math.radians(30)))
        local=((sample.features[0]-.5)*sample.aspect_ratio*z/fy,(sample.features[1]-.5)*z/fy,z)
        point=tuple(a+b for a,b in zip(position,apply(orientation,local)))
        roll=sample.features[3]
        direction=apply(orientation,(-math.sin(roll),-math.cos(roll),0))
        return point,direction

    def project(self,point,direction,q,aspect):
        position,orientation=self.camera_pose(q)
        inverse=transpose(orientation)
        x,y,z=apply(inverse,tuple(a-b for a,b in zip(point,position)))
        if z<50:
            raise ValueError('Palm is behind or too close to the virtual camera. Reset the reference.')
        ux,uy,_=apply(inverse,direction)
        fy=1/(2*math.tan(math.radians(30)))
        return (.5+fy*x/(aspect*z),.5+fy*y/z,
                self.neutral.features[2]+math.log(self.distance/z),math.atan2(-ux,-uy))

    def target(self,sample,now,pose=None):
        if not sample.valid(now) or sample.handedness!=self.neutral.handedness:
            raise ValueError('Following paused: show the same hand clearly.')
        q=angles(pose if pose is not None else self.filtered)
        dt=0 if self.last_time is None else max(0,min(.1,now-self.last_time))
        self.last_time=now
        # During preview the real camera stays at the anchor; project its input
        # through the virtual moving camera so a stationary hand does not cause drift.
        point,direction=self.observation(sample,q if self.hardware else self.anchor)
        features=self.project(point,direction,q,sample.aspect_ratio)
        desired=(.5,.5,self.neutral.features[2],self.neutral.features[3])
        error=[features[i]-desired[i] for i in range(4)]
        error[3]=wrap(error[3])
        if not self.follow_distance:
            error[2]=0
        thresholds=(.015,.015,.04,.035)
        error=[0 if abs(v)<thresholds[i] else max(-.5,min(.5,v)) for i,v in enumerate(error)]
        jacobian=[[0.0]*6 for _ in range(4)]
        for j in range(6):
            if not self.enabled[j]:
                continue
            delta=.05 if q[j]+.05<=LIMITS[j] else -.05
            shifted=list(q)
            shifted[j]+=delta
            projected=self.project(point,direction,shifted,sample.aspect_ratio)
            for i in range(4):
                difference=projected[i]-features[i]
                jacobian[i][j]=(wrap(difference) if i==3 else difference)/delta
        # Keep the distance feature constrained at zero velocity when disabled.
        normal=[[sum(jacobian[i][j]*jacobian[k][j] for j in range(6))+(0.00002 if i==k else 0) for k in range(4)] for i in range(4)]
        solution=solve(normal,[-1.5*self.sensitivity*v for v in error])
        velocity=[sum(jacobian[i][j]*solution[i] for i in range(4)) for j in range(6)]
        scale=max(1,max(abs(v) for v in velocity)/self.max_rate)
        base=self.filtered if self.hardware else q
        desired=[base[j]+velocity[j]/scale*dt for j in range(6)]
        if self.hardware:
            desired=[max(q[j]-.5,min(q[j]+.5,desired[j])) for j in range(6)]
        result=angles([max(-LIMITS[j],min(LIMITS[j],desired[j])) for j in range(6)])
        self.filtered=list(result)
        self.status=f'Palm error: horizontal {features[0]-.5:+.2f}, vertical {features[1]-.5:+.2f} · '+('following' if any(error) else 'aligned')
        return result
