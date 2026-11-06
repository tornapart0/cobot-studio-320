"""Single-owner USB connection. No hardware connection on application startup."""
import time
from .core import Move, JointMove, Wait, angles, next_pose
from .kinematics import endpoint
import math


class Controller:
    def __init__(self, factory=None):
        self.robot = None
        self.factory = factory

    def connect(self, port, baud=115200):
        if self.robot:
            raise ValueError('Already connected.')
        if not port.startswith('/dev/cu.'):
            raise ValueError('Choose a Mac /dev/cu.* USB serial port.')
        if baud not in (115200, 1000000):
            raise ValueError('Confirm 115200 or 1000000 against your firmware documentation.')
        if self.factory is None:
            from pymycobot.mycobot320 import MyCobot320
            factory = MyCobot320
        else:
            factory = self.factory
        device = factory(port, baud, timeout=0.2)
        try:
            angles(device.get_angles())
        except Exception:
            device.close()
            raise ValueError('No valid joint feedback. Check power, firmware and baud rate.')
        self.robot = device
        return self.read()

    def read(self):
        if not self.robot:
            raise ValueError('Connect the arm first.')
        return angles(self.robot.get_angles())

    def run(self, plan, cancel, report=lambda q: None, required_start=None, speed_modifier=1.0, expected_goal=None):
        """Wait for measured arrival, abort on timeout or cancellation."""
        if not self.robot:
            raise ValueError('Connect the arm first.')
        if type(speed_modifier) not in (int, float) or not math.isfinite(speed_modifier) or not 0.2 <= speed_modifier <= 2:
            raise ValueError('Arm speed modifier must be 0.2×–2×.')
        try:
            # Validate all command types before issuing any physical movement.
            if not all(isinstance(command, (Move, JointMove, Wait)) for command in plan):
                raise ValueError('Unsupported motion command.')
            if required_start is not None:
                expected = angles(required_start)
                measured = self.read()
                if max(abs(a-b) for a,b in zip(measured, expected)) > 2:
                    raise ValueError('Arm is not at start A (within 2°). Move to A separately, then push the route.')
            for command in plan:
                if cancel.is_set():
                    break
                if isinstance(command, Wait):
                    cancel.wait(command.seconds)
                    continue
                current = self.read()
                target = next_pose(current, command)
                speed = max(1, min(10, round(command.speed*speed_modifier)))
                if isinstance(command, JointMove):
                    self.robot.send_angle(command.joint_id, command.angle, speed)
                else:
                    if max(abs(a-b) for a,b in zip(current, target)) > 10:
                        raise ValueError('Hardware vector steps must be within 10°. Use robot.joint for a single-joint final move.')
                    self.robot.send_angles(list(target), speed)
                # One command per joint; feedback polling never resends it.
                deadline = time.monotonic()+max(20, 10+max(abs(a-b) for a,b in zip(current,target))*2/speed)
                settled = 0
                while not cancel.wait(0.1):
                    measured = self.read()
                    report(measured)
                    settled = settled+1 if max(abs(a-b) for a, b in zip(measured, target)) <= 1 else 0
                    if settled >= 3:
                        break
                    if time.monotonic() >= deadline:
                        raise TimeoutError('Arm did not settle at the commanded joint pose before its movement deadline.')
            if cancel.is_set():
                self.robot.stop()
                return None
            if expected_goal is not None:
                return self.verify(expected_goal)
            return None
        except Exception:
            cancel.set()
            self.robot.stop()
            raise

    def mirror(self, camera, neutral, hold, cancel, report, enabled, sensitivity=1.0, signs=(1,)*6, progress=lambda state: None, reference_pose=None, mapping_mode='palm', mapping_state=None):
        """Continuous explicit-start session; one outstanding joint at a time."""
        from .mirror import MirrorMapper, GestureMapper
        if not self.robot or len(enabled) != 6 or not any(enabled):
            raise ValueError('Connect the arm and select at least one mirror joint.')
        try:
            for method,label in (('is_power_on','motor power'), ('is_all_servo_enable','servo enable')):
                query = getattr(self.robot,method,None)
                if query is not None and query() == 0:
                    raise ValueError(f'Robot reports {label} is off. Check the physical stop and vendor motor-enable procedure.')
            measured_start = self.read()
            anchor = angles(reference_pose) if reference_pose is not None else measured_start
            full_range = isinstance(mapping_state,GestureMapper) or mapping_mode != 'palm'
            if not full_range and any(abs(a-b)>10 for a,b in zip(measured_start,anchor)):
                raise ValueError('Arm moved outside the calibrated mirror range. Read joints and calibrate again.')
            mapper = mapping_state or (MirrorMapper(neutral,anchor,sensitivity,signs) if mapping_mode == 'palm'
                      else GestureMapper(neutral,anchor,sensitivity,signs,mapping_mode))
            if mapper.anchor != anchor:
                raise ValueError('Preview and hardware mirror references differ. Recalibrate.')
            mapper.filtered = list(measured_start)
            if full_range:
                mapper.max_rate = 3.0
                # Hold non-selected joints at actual hardware positions, rather
                # than accidentally moving them toward an earlier preview.
                mapper.targets = list(measured_start)
            mapper.last_time = None
            outstanding = None
            command_count = 0
            while hold.is_set() and not cancel.is_set():
                measured = self.read()
                report(measured)
                if not full_range and any(abs(a-b)>12 for a,b in zip(measured,anchor)):
                    raise ValueError('Mirror travel envelope exceeded. Stop and recalibrate.')
                if any(not use and abs(a-b)>2 for use,a,b in zip(enabled,measured,measured_start)):
                    raise ValueError('An inactive joint moved unexpectedly.')
                target = mapper.target(camera.sample(),time.monotonic())
                if outstanding:
                    joint,value,issued = outstanding
                    if abs(measured[joint]-value)>1:
                        if time.monotonic()-issued > 2:
                            raise TimeoutError('Mirror joint feedback is not following its command.')
                        cancel.wait(0.1)
                        continue
                    outstanding = None
                candidates = [(abs(target[i]-measured[i]),i) for i in range(6) if enabled[i]]
                error,joint = max(candidates)
                if error > 0.3:
                    value = measured[joint]+max(-0.5,min(0.5,target[joint]-measured[joint]))
                    camera.sample()  # Recheck tracking freshness immediately before a command.
                    if not hold.is_set() or cancel.is_set():
                        break
                    self.robot.send_angle(joint+1,value,3)
                    outstanding = (joint,value,time.monotonic())
                    command_count += 1
                    progress(f'Mirror LIVE · {command_count} commands · J{joint+1} target {value:.1f}° · measured {measured[joint]:.1f}°')
                elif not outstanding:
                    progress(f'Mirror LIVE · {command_count} commands · waiting for selected-joint movement')
                cancel.wait(0.1)
            return 'Mirror stopped. Start live mirroring again when ready.'
        finally:
            if isinstance(mapping_state,GestureMapper):
                mapping_state.max_rate = 60.0
                mapping_state.last_time = None
            hold.clear()
            self.robot.stop()

    def verify(self, goal):
        goal = angles(goal)
        measured = self.read()
        error = [abs(a-b) for a,b in zip(measured,goal)]
        model_error = math.dist(endpoint(measured), endpoint(goal))
        return {'kind': 'verification', 'matched': max(error) <= 1 and model_error <= 5,
                'measured': list(measured), 'goal': list(goal), 'joint_errors': error,
                'model_error_mm': model_error}

    def close(self):
        if self.robot:
            try:
                self.robot.stop()
            finally:
                self.robot.close()
                self.robot = None
