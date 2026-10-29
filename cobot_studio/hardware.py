"""Single-owner USB connection. No hardware connection on application startup."""
import time
from .core import Move, angles


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

    def run(self, plan, cancel, report=lambda q: None):
        """Wait for measured arrival, abort on timeout or cancellation."""
        if not self.robot:
            raise ValueError('Connect the arm first.')
        try:
            for command in plan:
                if cancel.is_set():
                    break
                if not isinstance(command, Move):
                    cancel.wait(command.seconds)
                    continue
                current = self.read()
                if max(abs(a-b) for a, b in zip(current, command.target)) > 10:
                    raise ValueError('Hardware steps must be within 10° of measured position. Add intermediate moves.')
                self.robot.send_angles(list(command.target), command.speed)
                deadline = time.monotonic()+20
                settled = 0
                while not cancel.wait(0.1):
                    measured = self.read()
                    report(measured)
                    settled = settled+1 if max(abs(a-b) for a, b in zip(measured, command.target)) <= 1 else 0
                    if settled >= 3:
                        break
                    if time.monotonic() >= deadline:
                        raise TimeoutError('Arm did not reach target within 20 seconds.')
            if cancel.is_set():
                self.robot.stop()
        except Exception:
            cancel.set()
            self.robot.stop()
            raise

    def close(self):
        if self.robot:
            try:
                self.robot.stop()
            finally:
                self.robot.close()
                self.robot = None
