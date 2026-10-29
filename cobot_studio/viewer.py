"""Original interactive 3D schematic, rendered with a simple perspective camera.

Link sizes/axes are illustrative, not calibrated myCobot kinematics.
"""
import math
import tkinter as tk

COLORS = ('#8cdaa5', '#74c98f', '#9cddb0', '#66bc81', '#b2e7c2', '#7ad69a')
NAMES = ('Base', 'Shoulder', 'Elbow', 'Wrist 1', 'Wrist 2', 'Tool')


def rotate(point, axis, degrees):
    x, y, z = point
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    if axis == 'x':
        return (x, c*y-s*z, s*y+c*z)
    if axis == 'y':
        return (c*x+s*z, y, -s*x+c*z)
    return (c*x-s*y, s*x+c*y, z)


def joint_points(angles):
    """Six revolute joints plus tool endpoint with cumulative local rotations."""
    axes = ('z', 'y', 'y', 'x', 'y', 'x')
    links = ((0, 0, 55), (25, 0, 105), (90, 0, 35), (48, 0, 0), (35, 0, 0), (28, 0, 0))
    position = (0, 0, 18)
    points = [position]
    rotations = []
    for axis, angle, link in zip(axes, angles, links):
        rotations.append((axis, angle))
        offset = link
        for local_axis, local_angle in reversed(rotations):
            offset = rotate(offset, local_axis, local_angle)
        position = tuple(a+b for a, b in zip(position, offset))
        points.append(position)
    return points


class ArmViewer(tk.Canvas):
    def __init__(self, parent):
        super().__init__(parent, background='#090d0a', highlightthickness=0)
        self.pose = (0,)*6
        self.yaw, self.pitch, self.zoom = -35, 18, 1.0
        self.drag = None
        self.bind('<Configure>', lambda event: self.draw())
        self.bind('<ButtonPress-1>', lambda event: setattr(self, 'drag', (event.x, event.y)))
        self.bind('<B1-Motion>', self.orbit)
        self.bind('<MouseWheel>', self.wheel)

    def orbit(self, event):
        if self.drag:
            self.yaw += (event.x-self.drag[0])*0.5
            self.pitch = max(-60, min(75, self.pitch+(event.y-self.drag[1])*0.4))
            self.drag = (event.x, event.y)
            self.draw()

    def wheel(self, event):
        self.zoom = max(0.45, min(2.5, self.zoom*(1.08 if event.delta > 0 else 0.92)))
        self.draw()

    def set_pose(self, pose):
        self.pose = tuple(pose)
        self.draw()

    def reset_camera(self):
        self.yaw, self.pitch, self.zoom = -35, 18, 1.0
        self.draw()

    def camera(self, point):
        x, y, z = rotate(point, 'z', self.yaw)
        pitch = math.radians(self.pitch)
        depth = y*math.cos(pitch)+(z-120)*math.sin(pitch)
        height = (z-120)*math.cos(pitch)-y*math.sin(pitch)
        scale = min(self.winfo_width()/580, self.winfo_height()/420)*self.zoom
        perspective = 900/(900+depth)
        return (self.winfo_width()*0.45+x*scale*perspective,
                self.winfo_height()*0.5-height*scale*perspective, depth)

    def draw(self):
        self.delete('all')
        w, h = self.winfo_width(), self.winfo_height()
        if w < 10 or h < 10:
            return
        self.create_text(20, 22, text='Arm preview', fill='#c7e8d1', anchor='w', font='TkDefaultFont')
        self.create_text(20, 45, text='Drag to orbit · scroll to zoom', fill='#90b69b', anchor='w', font='TkDefaultFont')
        # Filled ground surface shares the same perspective as the arm.
        floor = [self.camera(point) for point in
                 [(-200, -200, 0), (250, -200, 0), (250, 250, 0), (-200, 250, 0)]]
        self.create_polygon([value for point in floor for value in point[:2]],
                            fill='#162b1d', outline='#4c7959', width=2)
        for value in range(-200, 251, 50):
            for a, b in [((value, -200, 0), (value, 250, 0)), ((-200, value, 0), (250, value, 0))]:
                p, q = self.camera(a), self.camera(b)
                self.create_line(p[:2], q[:2], fill='#25432e')
        base = [self.camera(p) for p in [(-35,-35,0),(35,-35,0),(35,35,0),(-35,35,0)]]
        self.create_polygon([v for p in base for v in p[:2]], fill='#172a1e', outline='#568366', width=2)
        points = [self.camera(p) for p in joint_points(self.pose)]
        segments = sorted(range(6), key=lambda i: -(points[i][2]+points[i+1][2])/2)
        for i in segments:
            p, q = points[i], points[i+1]
            self.create_line(p[:2], q[:2], fill='#355f43', width=22, capstyle='round')
            self.create_line(p[:2], q[:2], fill='#8dc69e', width=13, capstyle='round')
        # Every joint has a separate legend row, so overlapping projected joints remain identifiable.
        for i, p in enumerate(points[:6]):
            x, y = p[:2]
            self.create_oval(x-9, y-9, x+9, y+9, fill=COLORS[i], outline='#090d0a', width=2)
            self.create_text(x+15, y-16, text=f'J{i+1}', fill=COLORS[i], anchor='w', font='TkDefaultFont')
            self.create_text(w-18, 78+i*25, anchor='e', text=f'J{i+1}  {NAMES[i]}   {self.pose[i]:+.0f}°',
                             fill=COLORS[i], font='TkDefaultFont')
        # Draw an orientation marker so wrist/tool rotation remains visible.
        tip_world = joint_points(self.pose)[-1]
        axes = ('z', 'y', 'y', 'x', 'y', 'x')
        for offset, color in [((24, 0, 0), '#8cdaa5'), ((0, 24, 0), '#5da274'), ((0, 0, 24), '#b4e6c3')]:
            for axis, angle in reversed(list(zip(axes, self.pose))):
                offset = rotate(offset, axis, angle)
            end = self.camera(tuple(a+b for a,b in zip(tip_world, offset)))
            self.create_line(points[-1][:2], end[:2], fill=color, width=3)
        tip = points[-1]
        self.create_oval(tip[0]-5, tip[1]-5, tip[0]+5, tip[1]+5, fill='#8cdaa5', outline='')
        self.create_text(20, h-20, anchor='w', text='Illustrative geometry · no collision prediction', fill='#90b69b', font='TkDefaultFont')
