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
