"""Optional desktop smoke check; requires a display and Tk."""
import tkinter as tk
from cobot_studio.app import App

root = tk.Tk()
app = App(root)
root.update()
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
root.destroy()
print('Desktop UI smoke check passed; no serial connection opened.')
