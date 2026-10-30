"""Tk desktop UI. Background workers never touch Tk widgets."""
import json
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from pathlib import Path
from .core import parse_program, preview, program, angles, Move, LIMITS, target_plan
from .viewer import ArmViewer, NAMES
from .playback import sample
from .hardware import Controller
from .rl import train, load_policy, policy_plan

EXAMPLE = '''# Degrees, six joints; workbench speed 1–10.
robot.move([0, 0, 0, 0, 0, 0], 5)
robot.move([5, 0, 0, 0, 0, 0], 5)
robot.wait(1)
robot.move([0, 0, 0, 0, 0, 0], 5)
'''


class App:
    def __init__(self, root):
        self.root = root
        self.controller = Controller()
        self.events = queue.Queue()
        self.cancel = threading.Event()
        self.busy = False
        self.policy = None
        self.route_start = None
        self.route_goal = None
        self.current = (0,)*6
        self.frames = []
        self.animation = None
        self.playback = False
        self.play_position = 0.0
        root.title('Cobot Studio · myCobot 320 M5')
        root.geometry('1240x850')
        root.minsize(1080, 780)
        self.status = tk.StringVar(value='Simulation · disconnected')
        self.port = tk.StringVar()
        self.baud = tk.StringVar(value='115200')
        self.start_label = tk.StringVar(value='A: not set')
        self.end_label = tk.StringVar(value='B: not set')
        self.sim_speed = tk.DoubleVar(value=1.0)
        self.sim_smoothness = tk.DoubleVar(value=100)
        self.speed_label = tk.StringVar(value='1.00×')
        self.smooth_label = tk.StringVar(value='100%')
        self.armed = tk.BooleanVar(value=False)
        self.pose_vars = [tk.DoubleVar(value=0) for _ in range(6)]
        self.pose_labels = [tk.StringVar(value='0°') for _ in range(6)]
        style = ttk.Style(root)
        # Keep standard controls and default fonts; use green accents on black.
        style.theme_use('clam')
        root.configure(background='#090d0a')
        root.option_add('*TCombobox*Listbox.background', '#111b14')
        root.option_add('*TCombobox*Listbox.foreground', '#b8edc8')
        root.option_add('*TCombobox*Listbox.selectBackground', '#285d39')
        style.configure('.', background='#090d0a', foreground='#c7e8d1',
                        bordercolor='#2b4935', lightcolor='#2b4935', darkcolor='#090d0a',
                        selectbackground='#285d39', selectforeground='#e1f7e7')
        style.configure('TFrame', background='#090d0a')
        style.configure('TLabel', background='#090d0a')
        style.configure('TNotebook', background='#090d0a', borderwidth=0)
        style.configure('TNotebook.Tab', background='#111b14', padding=(12, 7))
        style.map('TNotebook.Tab', background=[('selected', '#234e30'), ('active', '#1b3523')],
                  foreground=[('selected', '#d6f6df')])
        style.configure('TButton', background='#173b23', bordercolor='#365e42')
        style.map('TButton', background=[('pressed', '#2f6e42'), ('active', '#245632')],
                  foreground=[('disabled', '#66816e')])
        style.configure('TScale', background='#090d0a', troughcolor='#1c3424')
        style.configure('TCheckbutton', background='#090d0a', indicatorbackground='#111b14')
        style.map('TCheckbutton', background=[('active', '#090d0a')],
                  indicatorbackground=[('selected', '#58b978')])
        style.configure('TEntry', fieldbackground='#111b14', insertcolor='#b8edc8')
        style.configure('TCombobox', fieldbackground='#111b14', background='#173b23',
                        arrowcolor='#8cdaa5')
        style.map('TCombobox', fieldbackground=[('readonly', '#111b14')],
                  foreground=[('readonly', '#c7e8d1')])
        outer = ttk.Frame(root, padding=16)
        outer.pack(fill='both', expand=True)
        header = ttk.Frame(outer)
        header.pack(fill='x', pady=(0, 12))
        ttk.Label(header, text='Cobot Studio · Beta', style='Title.TLabel').pack(side='left')
        ttk.Label(header, textvariable=self.status).pack(side='right')

        body = ttk.Frame(outer)
        body.pack(fill='both', expand=True)
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=2, minsize=420)
        body.rowconfigure(0, weight=1)
        left = ttk.Frame(body)
        left.grid(row=0, column=0, sticky='nsew', padx=(0, 16))
        self.viewer = ArmViewer(left)
        self.viewer.pack(fill='both', expand=True)
        view_row = ttk.Frame(left)
        view_row.pack(fill='x', pady=8)
        ttk.Button(view_row, text='Reset view', command=self.viewer.reset_camera).pack(side='left')
        ttk.Button(view_row, text='Show measured pose', command=lambda: self.set_pose(self.current)).pack(side='left', padx=6)
        ttk.Button(view_row, text='Replay', command=self.draw).pack(side='right')
        for title, variable, low, high, label in [
            ('Sim speed', self.sim_speed, 0.25, 4, self.speed_label),
            ('Smoothness', self.sim_smoothness, 0, 100, self.smooth_label)]:
            row = ttk.Frame(left)
            row.pack(fill='x', pady=3)
            ttk.Label(row, text=title, width=16).pack(side='left')
            ttk.Scale(row, from_=low, to=high, variable=variable,
                      command=lambda value: self.playback_labels()).pack(side='left', fill='x', expand=True)
            ttk.Label(row, textvariable=label, width=7, anchor='e').pack(side='left', padx=(8, 0))
        ttk.Label(left, text='Joint target', style='Heading.TLabel').pack(anchor='w', pady=(6, 4))
        for i, limit in enumerate(LIMITS):
            row = ttk.Frame(left)
            row.pack(fill='x', pady=3)
            ttk.Label(row, text=f'J{i+1}  {NAMES[i]}', width=16).pack(side='left')
            ttk.Scale(row, from_=-limit, to=limit, variable=self.pose_vars[i],
                      command=lambda value: self.slider_changed()).pack(side='left', fill='x', expand=True)
            ttk.Label(row, textvariable=self.pose_labels[i], width=7, anchor='e').pack(side='left', padx=(8, 0))
        pose_row = ttk.Frame(left)
        pose_row.pack(fill='x', pady=9)
        ttk.Button(pose_row, text='Zero target', command=lambda: self.set_pose((0,)*6)).pack(side='left')
        ttk.Button(pose_row, text='Add to script', command=self.add_pose).pack(side='left', padx=6)
        ttk.Button(pose_row, text='Push target…', command=self.push_target).pack(side='right')
        ttk.Label(left, text='Sliders adjust the preview. Push target sends a reviewed move over USB.',
                  wraplength=540).pack(anchor='w')

        right = ttk.Frame(body)
        right.grid(row=0, column=1, sticky='nsew')
        tabs = ttk.Notebook(right)
        tabs.pack(fill='both', expand=True)
        sim = ttk.Frame(tabs, padding=12)
        scripting = ttk.Frame(tabs, padding=12)
        connection = ttk.Frame(tabs, padding=12)
        tabs.add(sim, text='Train & simulate')
        tabs.add(scripting, text='Script')
        tabs.add(connection, text='Connect')
        ttk.Label(sim, text='1. Set start A and end B', style='Heading.TLabel').pack(anchor='w')
        ttk.Label(sim, text='Adjust the sliders, then capture each pose.', wraplength=350).pack(anchor='w', pady=6)
        poses = ttk.Frame(sim)
        poses.pack(fill='x', pady=6)
        ttk.Button(poses, text='Set start A', command=lambda: self.capture_pose('start')).pack(side='left')
        ttk.Button(poses, text='Set end B', command=lambda: self.capture_pose('goal')).pack(side='left', padx=6)
        ttk.Label(sim, textvariable=self.start_label, wraplength=350).pack(anchor='w', pady=3)
        ttk.Label(sim, textvariable=self.end_label, wraplength=350).pack(anchor='w', pady=3)
        show = ttk.Frame(sim)
        show.pack(fill='x', pady=6)
        ttk.Button(show, text='Show A', command=lambda: self.show_pose('start')).pack(side='left')
        ttk.Button(show, text='Show B', command=lambda: self.show_pose('goal')).pack(side='left', padx=6)
        ttk.Label(sim, text='2. Train A → B', style='Heading.TLabel').pack(anchor='w', pady=(14, 4))
        ttk.Button(sim, text='Train this route', command=self.train).pack(anchor='w', pady=6)
        self.reward = tk.StringVar(value='Set both poses to begin.')
        ttk.Label(sim, textvariable=self.reward, wraplength=350).pack(anchor='w', pady=6)
        ttk.Label(sim, text='3. Preview and push', style='Heading.TLabel').pack(anchor='w', pady=(14, 4))
        ttk.Button(sim, text='Simulate A → B', command=self.simulate_policy).pack(anchor='w', pady=4)
        ttk.Button(sim, text='Move arm to start A…', command=self.move_to_start).pack(anchor='w', pady=4)
        ttk.Button(sim, text='Push trained A → B…', command=self.push_route).pack(anchor='w', pady=4)
        files = ttk.Frame(sim)
        files.pack(fill='x', pady=18)
        ttk.Button(files, text='Load policy', command=self.load_policy).pack(side='left')
        ttk.Button(files, text='Save policy', command=self.save_policy).pack(side='left', padx=6)
        ttk.Label(sim, text='Joint simulation only. No gravity, contact or collision model. Physical testing is required.',
                  wraplength=350).pack(anchor='w', pady=12)

        ttk.Label(scripting, text='Motion script', style='Heading.TLabel').pack(anchor='w')
        ttk.Label(scripting, text='robot.move([six angles], speed)\nrobot.wait(seconds)').pack(anchor='w', pady=8)
        self.editor = tk.Text(scripting, height=14, width=35, undo=True, wrap='none', background='#0d1510', foreground='#c7e8d1', insertbackground='#8cdaa5', selectbackground='#285d39', selectforeground='#e1f7e7')
        self.editor.pack(fill='both', expand=True)
        self.editor.insert('1.0', EXAMPLE)
        row = ttk.Frame(scripting)
        row.pack(fill='x', pady=8)
        for label, command in [('Open', self.open_program), ('Save', self.save_program), ('Simulate', self.preview)]:
            ttk.Button(row, text=label, command=command).pack(side='left', padx=(0, 4))
        ttk.Button(scripting, text='Push script to arm…', command=self.run_arm).pack(anchor='w', pady=4)
        ttk.Button(scripting, text='Export trajectory', command=self.save_trajectory).pack(anchor='w', pady=4)

        ttk.Label(connection, text='myCobot 320 M5 · USB', style='Heading.TLabel').pack(anchor='w')
        ttk.Label(connection, text='Select your arm’s serial port.').pack(anchor='w', pady=(16, 6))
        self.ports = ttk.Combobox(connection, textvariable=self.port, width=30)
        self.ports.pack(fill='x')
        ttk.Button(connection, text='Refresh ports', command=self.scan).pack(anchor='w', pady=8)
        ttk.Label(connection, text='Firmware baud rate').pack(anchor='w', pady=(16, 6))
        ttk.Combobox(connection, textvariable=self.baud, values=('115200', '1000000'),
                     width=12, state='readonly').pack(anchor='w')
        ttk.Button(connection, text='Connect / read joints', command=self.connect).pack(anchor='w', pady=(18, 6))
        ttk.Button(connection, text='Disconnect', command=self.disconnect).pack(anchor='w', pady=6)
        ttk.Label(connection, text='Connecting reads the arm without moving it. Python stays on this Mac.',
                  wraplength=350).pack(anchor='w', pady=18)

        footer = ttk.Frame(outer)
        footer.pack(fill='x', pady=(14, 4))
        ttk.Checkbutton(footer, text='Workspace clear · physical stop ready', variable=self.armed).pack(side='left')
        ttk.Button(footer, text='STOP', command=self.stop).pack(side='right')
        self.log = tk.Text(outer, height=3, state='disabled', wrap='word', background='#0d1510', foreground='#9cc9a9')
        self.log.pack(fill='x', pady=(4, 0))
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.after(100, self.poll)
        self.scan()

    def target_pose(self):
        return angles([round(v.get(), 1) for v in self.pose_vars])

    def set_pose(self, pose):
        for variable, value in zip(self.pose_vars, pose):
            variable.set(value)
        self.slider_changed()

    def slider_changed(self):
        # Sliders never issue hardware commands.
        if self.animation is not None:
            self.root.after_cancel(self.animation)
            self.animation = None
        self.playback = False
        pose = self.target_pose()
        for label, value in zip(self.pose_labels, pose):
            label.set(f'{value:+.1f}°')
        self.viewer.set_pose(pose)

    def add_pose(self):
        self.editor.insert('end', program([Move(self.target_pose(), 5)]))
        self.editor.see('end')
        self.note('Target added to script.')

    def push_target(self):
        self.guard(lambda: self.send_plan(target_plan(self.current, self.target_pose())))

    def capture_pose(self, which):
        def capture():
            if self.busy:
                raise ValueError('Wait for training or movement to finish before changing A or B.')
            setattr(self, 'route_'+which, self.target_pose())
            self.policy = None
            self.reward.set('Poses changed. Train this route before pushing.')
            self.update_pose_labels()
        self.guard(capture)

    def update_pose_labels(self):
        for label, pose, name in [(self.start_label, self.route_start, 'A'),
                                  (self.end_label, self.route_goal, 'B')]:
            label.set(f'{name}: '+(', '.join(f'{v:g}°' for v in pose) if pose is not None else 'not set'))

    def show_pose(self, which):
        def show():
            pose = getattr(self, 'route_'+which)
            if pose is None:
                raise ValueError('Set this pose with the sliders first.')
            self.set_pose(pose)
        self.guard(show)

    def route(self):
        if self.route_start is None or self.route_goal is None:
            raise ValueError('Set both start A and end B with the sliders first.')
        return self.route_start, self.route_goal

    def trained_route(self):
        start, goal = self.route()
        if not self.policy or self.policy.get('start') != list(start) or self.policy.get('goal') != list(goal):
            raise ValueError('Train a policy for these exact A and B poses first.')
        return policy_plan(self.policy, start, goal)

    def simulate_policy(self):
        self.generate()

    def move_to_start(self):
        def move():
            start, _ = self.route()
            self.send_plan(target_plan(self.current, start))
        self.guard(move)

    def push_route(self):
        def push():
            plan = self.trained_route()
            self.send_plan(plan, required_start=self.route_start)
        self.guard(push)

    def playback_labels(self):
        self.speed_label.set(f'{self.sim_speed.get():.2f}×')
        self.smooth_label.set(f'{self.sim_smoothness.get():.0f}%')

    def animate(self):
        self.animation = None
        now = time.monotonic()
        elapsed = max(0, now-self.play_time)
        self.play_time = now
        # 1× retains the original 40 ms per preview interval; rendering is 60 Hz.
        self.play_position = min(len(self.frames)-1,
                                 self.play_position+elapsed*self.sim_speed.get()/0.04)
        self.viewer.set_pose(sample(self.frames, self.play_position, self.sim_smoothness.get()/100))
        if self.play_position >= len(self.frames)-1:
            self.playback = False
            return
        self.animation = self.root.after(16, self.animate)

    def note(self, message):
        self.log.configure(state='normal')
        self.log.insert('end', str(message)+'\n')
        self.log.see('end')
        self.log.configure(state='disabled')

    def task(self, action):
        if self.busy:
            raise ValueError('Wait for the current operation, or press STOP.')
        self.busy = True
        self.cancel.clear()
        def work():
            try:
                result = action()
                self.events.put(('done', result))
            except Exception as exc:
                self.events.put(('error', str(exc)))
        threading.Thread(target=work, daemon=True).start()

    def poll(self):
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind in ('done', 'error'):
                    self.busy = False
                    self.armed.set(False)
                    if kind == 'error':
                        self.note('Error: '+value)
                    elif isinstance(value, tuple) and len(value) == 6:
                        self.current = value
                        self.set_pose(value)
                        self.note('Measured joints: '+str(value))
                    elif isinstance(value, dict):
                        self.policy = value
                        self.reward.set(f"Training reward: {value['initial_reward']:.0f} → {value['final_reward']:.0f} (higher is better)")
                    else:
                        self.note(value or 'Operation complete.')
                    self.status.set('USB connected' if self.controller.robot else 'Simulation · disconnected')
                elif kind == 'pose':
                    self.current = value
                    self.viewer.set_pose(value)
        except queue.Empty:
            pass
        self.root.after(100, self.poll)

    def scan(self):
        import glob
        ports = sorted(glob.glob('/dev/cu.*'))
        self.ports['values'] = ports
        if ports and not self.port.get():
            self.port.set(ports[0])

    def connect(self):
        def start():
            port, baud = self.port.get(), int(self.baud.get())
            self.task(lambda: self.controller.read() if self.controller.robot else self.controller.connect(port, baud))
        self.guard(start)

    def disconnect(self):
        self.guard(lambda: self.task(self.controller.close))

    def plan(self):
        return parse_program(self.editor.get('1.0', 'end'))

    def preview(self):
        def start():
            self.frames = preview(self.plan(), self.current)
            self.draw()
            self.note(f'Validated {len(self.frames)} preview steps from {self.current}.')
        self.guard(start)

    def draw(self):
        if self.animation is not None:
            self.root.after_cancel(self.animation)
        self.animation = None
        if self.busy:
            self.note('Wait for the current operation before replaying.')
            return
        if not self.frames:
            self.note('Simulate a route or script first.')
            return
        self.play_position = 0.0
        self.play_time = time.monotonic()
        self.playback = True
        self.viewer.set_pose(self.frames[0])
        self.animation = self.root.after(16, self.animate)

    def train(self):
        def begin():
            start, goal = self.route()
            self.reward.set('Training A → B…')
            self.task(lambda: train(start=start, goal=goal)[0])
        self.guard(begin)

    def load_policy(self):
        def load():
            path = filedialog.askopenfilename(filetypes=[('JSON policy', '*.json')])
            if path:
                if self.busy:
                    raise ValueError('Wait for the current operation to finish.')
                self.policy = load_policy(path)
                if 'start' in self.policy:
                    self.route_start = angles(self.policy['start'])
                    self.route_goal = angles(self.policy['goal'])
                    self.update_pose_labels()
                    self.reward.set('A → B policy loaded. Simulate before pushing.')
                else:
                    self.reward.set('Legacy policy has no saved A/B. Set poses and train this route.')
        self.guard(load)

    def save_policy(self):
        def save():
            if not self.policy:
                raise ValueError('Train or load a policy first.')
            self.save_text(json.dumps(self.policy, indent=2), '.json')
        self.guard(save)

    def generate(self):
        def generate():
            plan = self.trained_route()
            self.editor.delete('1.0', 'end')
            # The script includes A explicitly so it preserves the route's start.
            self.editor.insert('1.0', program([Move(self.route_start, 5)] + plan))
            self.frames = preview(plan, self.route_start)
            self.draw()
            self.note('Trained A → B route simulated and saved in the script editor.')
        self.guard(generate)

    def open_program(self):
        def load():
            path = filedialog.askopenfilename(filetypes=[('Python plan', '*.py')])
            if path:
                text = Path(path).read_text()
                parse_program(text)
                self.editor.delete('1.0', 'end')
                self.editor.insert('1.0', text)
        self.guard(load)

    def save_text(self, text, extension):
        path = filedialog.asksaveasfilename(defaultextension=extension)
        if path:
            Path(path).write_text(text)

    def save_program(self):
        self.guard(lambda: self.save_text(program(self.plan()), '.py'))

    def save_trajectory(self):
        def save():
            frames = preview(self.plan(), self.current)
            self.save_text(json.dumps({'schema': 'cobot-joint-preview-v1', 'units': 'degrees',
                                       'start': self.current, 'frames': frames}, indent=2), '.json')
        self.guard(save)

    def send_plan(self, plan, required_start=None):
        if self.busy:
            raise ValueError('Wait for the current operation, or press STOP.')
        if not self.controller.robot or not self.armed.get():
            raise ValueError('Connect USB and check the workspace acknowledgment first.')
        preview(plan, self.current)
        if not messagebox.askyesno('Move real arm', 'Send this plan over USB at speed 1–10?\nKeep the physical emergency stop ready.'):
            return
        if self.animation is not None:
            self.root.after_cancel(self.animation)
            self.animation = None
        self.playback = False
        self.task(lambda: self.controller.run(plan, self.cancel, lambda q: self.events.put(('pose', q)), required_start=required_start))

    def run_arm(self):
        self.guard(lambda: self.send_plan(self.plan()))

    def stop(self):
        if self.animation is not None:
            self.root.after_cancel(self.animation)
            self.animation = None
        self.playback = False
        self.cancel.set()
        self.armed.set(False)
        self.note('Cancel requested. Hardware worker sends stop after its current serial call. Use the physical stop if needed.')

    def close(self):
        self.stop()
        if self.busy:
            self.note('Wait for operation to stop before closing.')
            return
        try:
            self.controller.close()
        finally:
            self.root.destroy()

    def guard(self, action):
        try:
            action()
        except Exception as exc:
            self.note('Error: '+str(exc))


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == '__main__':
    main()
