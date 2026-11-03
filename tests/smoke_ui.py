"""Optional desktop smoke check; requires a display and Tk."""
import tkinter as tk
from cobot_studio.app import App

root = tk.Tk()
app = App(root)
root.update()
assert app.camera.process is None
assert not app.mirror_enabled
# Check the worker-to-Tk image format without opening a camera.
import base64
packet = {'captured_at': 1, 'features': None,
          'ppm': base64.b64encode(b'P6\n2 2\n255\n'+bytes([0,120,40])*4).decode()}
original_packet = app.camera.packet
app.camera.packet = lambda: (packet,None)
app.mirror_tick()
assert app.camera_image is not None and app.camera_image.width() == 2
app.camera.packet = original_packet
app.preview()
app.set_pose((10, -15, 20, 25, 30, 35))
root.update()
assert app.viewer.pose == (10, -15, 20, 25, 30, 35)
assert app.controller.robot is None
app.add_pose()
assert app.plan()[-1].target == (10, -15, 20, 25, 30, 35)
app.viewer.reset_camera()
app.stop()
assert app.animation is None
assert app.frames
assert app.controller.robot is None
assert not app.armed.get()
from cobot_studio.rl import train
app.set_pose((-20, 0, 0, 0, 0, 0))
app.capture_pose('start')
app.set_pose((40, 10, -10, 0, 0, 0))
app.capture_pose('goal')
app.policy = train(start=app.route_start, goal=app.route_goal)[0]
app.generate()
assert app.frames[0] == app.route_start
assert app.frames[-1] == app.route_goal
assert len(app.plan()) <= 6
assert all(hasattr(command, "joint_id") for command in app.plan())
app.set_pose((45, 10, -10, 0, 0, 0))
app.capture_pose('goal')
assert app.policy is None
root.update()
# Tracking loss freezes preview and disarms hardware without deleting calibration.
import time
from cobot_studio.mirror import HandSample
original_sample = app.camera.sample
app.camera.sample = lambda: HandSample(time.monotonic(),(0,)*6,0.99,'Right',(0,)*5)
app.calibrate_mirror()
neutral = app.mirror_neutral
app.mirror_enabled = True
app.mirror_run.set()
def missing():
    raise ValueError('temporarily out of view')
app.camera.sample = missing
app.mirror_tick()
assert app.mirror_neutral is neutral
assert app.mirror_mapper is not None
assert not app.mirror_enabled and not app.mirror_run.is_set()
app.camera.sample = lambda: HandSample(time.monotonic(),(0,)*6,0.99,'Right',(0,)*5)
app.mirror_tick()
assert app.mirror_neutral is neutral and not app.mirror_enabled
# One Start action begins a continuous session; no held mouse event is needed.
from unittest.mock import patch
original_controller, original_task = app.controller, app.task
calls=[]
class FakeMirrorController:
    robot = object()
    def mirror(self,*args,**kwargs):
        calls.append(kwargs)
        return 'Mirror stopped.'
app.controller = FakeMirrorController()
app.camera.sample = lambda: HandSample(time.monotonic(),(0,)*6,0.99,'Right',(0,)*5)
app.calibrate_mirror()
mapper = app.mirror_mapper
app.armed.set(True)
assert not app.robot_output.get()
app.robot_output.set(True)
actions=[]
app.task = lambda action: actions.append(action)
with patch('cobot_studio.app.messagebox.askyesno',return_value=True):
    app.enable_mirror()
assert app.mirror_run.is_set() and len(actions) == 1
assert not hasattr(app,'hold_button')
actions[0]()
assert calls[0]['mapping_state'] is mapper
assert calls[0]['mapping_mode'] == 'Finger count + twist'
app.robot_output.set(False)
app.change_mirror_output()
assert not app.mirror_run.is_set()
assert app.mirror_mapper is mapper
app.stop()
assert not app.mirror_run.is_set()
app.controller, app.task = original_controller, original_task
app.camera.sample = original_sample
root.destroy()
print('Desktop UI smoke check passed; no serial connection opened.')
