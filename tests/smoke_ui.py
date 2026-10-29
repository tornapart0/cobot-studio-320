"""Optional desktop smoke check; requires a display and Tk."""
import tkinter as tk
from cobot_studio.app import App

root = tk.Tk()
app = App(root)
root.update()
app.preview()
assert app.frames
assert app.controller.robot is None
assert not app.armed.get()
app.policy = {'schema': 'cobot-joint-gains-v1', 'gains': [0.5]*6}
app.generate()
assert len(app.plan()) == 60
root.update()
root.destroy()
print('Desktop UI smoke check passed; no serial connection opened.')
