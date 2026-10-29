"""Tk desktop UI. Background workers never touch Tk widgets."""
import json
import queue
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from pathlib import Path
from .core import parse_program, preview, program, angles
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
        self.current = (0,)*6
        self.frames = []
        root.title('Cobot Studio · myCobot 320 M5')
        root.geometry('1100x800')
        root.minsize(900, 680)
        self.status = tk.StringVar(value='Simulation · disconnected')
        self.port = tk.StringVar()
        self.baud = tk.StringVar(value='115200')
        self.goal = tk.StringVar(value='10, -10, 10, 0, 0, 0')
        self.armed = tk.BooleanVar(value=False)
        outer = ttk.Frame(root, padding=18)
        outer.pack(fill='both', expand=True)
        ttk.Label(outer, text='Cobot Studio', font=('Helvetica', 25, 'bold')).pack(anchor='w')
        ttk.Label(outer, text='Write → preview → train → run over USB', font=('Helvetica', 13)).pack(anchor='w', pady=(2, 12))
        connection = ttk.LabelFrame(outer, text='M5 USB connection', padding=10)
        connection.pack(fill='x')
        self.ports = ttk.Combobox(connection, textvariable=self.port, width=32)
        self.ports.pack(side='left')
        ttk.Button(connection, text='Find ports', command=self.scan).pack(side='left', padx=5)
        ttk.Combobox(connection, textvariable=self.baud, values=('115200', '1000000'), width=10, state='readonly').pack(side='left')
        ttk.Button(connection, text='Connect / read', command=self.connect).pack(side='left', padx=5)
        ttk.Button(connection, text='Disconnect', command=self.disconnect).pack(side='left')
        ttk.Label(outer, textvariable=self.status).pack(anchor='w', pady=8)
        notebook = ttk.Notebook(outer)
        notebook.pack(fill='both', expand=True)
        editor_tab = ttk.Frame(notebook, padding=10)
        rl_tab = ttk.Frame(notebook, padding=10)
        notebook.add(editor_tab, text='Python & preview')
        notebook.add(rl_tab, text='RL training')
        self.editor = tk.Text(editor_tab, height=11, font=('Menlo', 12), undo=True, wrap='none')
        self.editor.pack(fill='both', expand=True)
        self.editor.insert('1.0', EXAMPLE)
        row = ttk.Frame(editor_tab)
        row.pack(fill='x', pady=8)
        for label, command in [('Open .py', self.open_program), ('Save .py', self.save_program),
                               ('Preview', self.preview), ('Save trajectory', self.save_trajectory)]:
            ttk.Button(row, text=label, command=command).pack(side='left', padx=(0, 5))
        self.chart = tk.Canvas(editor_tab, height=180, bg='#111c2d', highlightthickness=0)
        self.chart.pack(fill='x')
        ttk.Label(editor_tab, text='Six joint angles across preview steps · not a 3D or collision simulation').pack(anchor='w')
        ttk.Label(rl_tab, text='Train a joint-reaching policy', font=('Helvetica', 18, 'bold')).pack(anchor='w')
        ttk.Label(rl_tab, text='Reward-driven random search learns six feedback gains in a synthetic joint model.\nActions are bounded to 2° per step. No real arm is used during training.').pack(anchor='w', pady=12)
        rl_row = ttk.Frame(rl_tab)
        rl_row.pack(anchor='w')
        for label, command in [('Train policy', self.train), ('Load policy', self.load_policy), ('Save policy', self.save_policy)]:
            ttk.Button(rl_row, text=label, command=command).pack(side='left', padx=(0, 5))
        ttk.Label(rl_tab, text='Target joint angles, comma separated:').pack(anchor='w', pady=(20, 5))
        ttk.Entry(rl_tab, textvariable=self.goal, width=65).pack(anchor='w')
        ttk.Button(rl_tab, text='Generate Python from policy', command=self.generate).pack(anchor='w', pady=10)
        self.reward = tk.StringVar(value='No policy trained or loaded.')
        ttk.Label(rl_tab, textvariable=self.reward).pack(anchor='w')
        ttk.Label(rl_tab, text='Generated plans start at the last measured arm pose (zero while disconnected).\nReview and preview the Python before running. External pickle / PyTorch models are not loaded.').pack(anchor='w', pady=18)
        run_row = ttk.Frame(outer)
        run_row.pack(fill='x', pady=10)
        ttk.Checkbutton(run_row, text='I checked the clear workspace and physical stop', variable=self.armed).pack(side='left')
        ttk.Button(run_row, text='Run on arm', command=self.run_arm).pack(side='right')
        ttk.Button(run_row, text='STOP / cancel', command=self.stop).pack(side='right', padx=10)
        ttk.Label(outer, text='Software stop depends on USB and firmware. Keep the physical emergency stop accessible.').pack(anchor='w')
        self.log = tk.Text(outer, height=5, state='disabled', font=('Menlo', 10))
        self.log.pack(fill='x', pady=(8, 0))
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.after(100, self.poll)
        self.scan()

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
                        self.note('Measured joints: '+str(value))
                    elif isinstance(value, dict):
                        self.policy = value
                        self.reward.set(f"Training reward: {value['initial_reward']:.0f} → {value['final_reward']:.0f} (higher is better)")
                    else:
                        self.note(value or 'Operation complete.')
                    self.status.set('USB connected' if self.controller.robot else 'Simulation · disconnected')
                elif kind == 'pose':
                    self.current = value
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
        self.chart.delete('all')
        w = max(800, self.chart.winfo_width())
        self.chart.create_line(20, 90, w-20, 90, fill='#45556d')
        for j, color in enumerate(('#77d8ee', '#a8e6a1', '#f1ca75', '#b6a3ff', '#ff9cac', '#f2f4f8')):
            points = []
            for i, frame in enumerate(self.frames):
                points.extend((20+i*(w-40)/max(1, len(self.frames)-1), 90-frame[j]*0.43))
            if len(points) >= 4:
                self.chart.create_line(*points, fill=color, width=2)
            self.chart.create_text(40+j*70, 165, text=f'J{j+1}', fill=color)

    def train(self):
        self.guard(lambda: self.task(lambda: train()[0]))

    def load_policy(self):
        def load():
            path = filedialog.askopenfilename(filetypes=[('JSON policy', '*.json')])
            if path:
                self.policy = load_policy(path)
                self.reward.set('Policy loaded. Generate a trajectory to inspect it.')
        self.guard(load)

    def save_policy(self):
        def save():
            if not self.policy:
                raise ValueError('Train or load a policy first.')
            self.save_text(json.dumps(self.policy, indent=2), '.json')
        self.guard(save)

    def generate(self):
        def generate():
            if not self.policy:
                raise ValueError('Train or load a policy first.')
            target = angles([float(x.strip()) for x in self.goal.get().split(',')])
            plan = policy_plan(self.policy, self.current, target)
            self.editor.delete('1.0', 'end')
            self.editor.insert('1.0', program(plan))
            self.preview()
            self.note('Policy trajectory placed in Python editor.')
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

    def run_arm(self):
        def run():
            if not self.controller.robot or not self.armed.get():
                raise ValueError('Connect USB and check the workspace acknowledgment first.')
            plan = self.plan()
            preview(plan, self.current)
            if not messagebox.askyesno('Physical movement', 'Run this plan on the real arm at speed 1–10?\nKeep the physical emergency stop ready.'):
                return
            self.task(lambda: self.controller.run(plan, self.cancel, lambda q: self.events.put(('pose', q))))
        self.guard(run)

    def stop(self):
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
