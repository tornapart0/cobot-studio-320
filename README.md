# Cobot Sim & Push · myCobot 320 M5 2022

A local Mac desktop application for setting start/end poses, training a joint-move order, previewing it and sending it over USB. Green/black interface, default system fonts, labeled 3D joint viewer, Python motion editor and measured feedback. The viewer opens in a side view with rigid right-angle link bodies and curved positive-rotation arrows; drag to orbit.

**Beta: this iteration awaits physical validation.** The model now follows the manufacturer's 2022 M5 joint frames. Rendering is schematic and there is no collision or dynamics model. No robot was connected for this iteration’s tests. See [the joint-map audit](docs/JOINT_MAP.md) for dimensions, axes, directionality and limits of verification.

## Launch

Use the **Cobot Sim & Push** Dock shortcut on the configured Mac, or open `Launch.command` in this folder. Python 3.11+ with Tk is required (available with the python.org Mac installer).

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
cobot-studio
```

The launcher uses the code in this folder, so updates apply when you next open the app. It never connects or moves the arm automatically. This is source with a launcher, not a standalone bundled Python installation.

## A → B workflow

1. Adjust the six sliders and select **Set start A**. Adjust again and select **Set end B**. **Show A/B** restores either saved pose. The model uses the manufacturer's true fixed offsets; zero is not artificially straightened.
2. Choose iterations per batch and **Train this route**. Training searches for a joint order with lower integrated joint error. **Continue training** starts from the best existing order, keeps the same endpoints and adds iterations. Changing A or B clears old training.
3. **Simulate A → B** animates the individual joint moves and places the same commands in the script editor. There are at most six: each changed joint moves once directly to its final angle; unchanged joints are omitted. Simulation frames are never all sent to the robot.
4. Preview from several camera angles. A one-joint-at-a-time path can hit obstacles or other parts of the robot. The trainer does not detect or avoid these collisions.
5. In **Connect**, select the USB `/dev/cu.*` port and correct firmware baud rate, then **Connect / read joints**. The unit must be mounted, powered and running the manufacturer's serial-control firmware. The app does not flash, calibrate or power on hardware.
6. Set **Arm speed ×** (0.2–2.0). It multiplies each command's speed and clamps the result to the workbench's 1–10% SDK speed range. This differs from **Sim speed**, which affects only playback.
7. After checking the workspace and physical stop, use **Move arm to start A** for a separately confirmed approach. **Push trained A → B** checks fresh feedback within 2° per joint of A, then issues at most six individual `send_angle` calls. It waits for three samples within 1° of each commanded intermediate pose before advancing. Timeout or invalid feedback stops the run; it never retries a joint command automatically.
8. The final report says **Matched B** only if all measured joint errors are within 1° and the model flange-origin error is within 5 mm. **Verify measured B** repeats the read-only check. Verification is saved with a policy if you save after measuring. The model-based error uses encoder readings; it is not independent physical tool-position measurement.

Continue training can improve simulated move order. It cannot remove encoder calibration, mechanical or model errors. A measured mismatch stays visible and must be investigated. The app does not repeatedly move the real arm or silently apply learned calibration offsets until the error disappears. All training is bounded to the requested batch; every physical run is separately confirmed.

## Speed and smoothness

**Sim speed** is 0.25×–4×. **Smoothness** blends linear display motion and bounded interpolation, with up to 60 display updates per second. **Replay** restarts the last simulation. These controls do not change exported joint commands, real timing or real hardware speed. The viewer does not play script waits or model servo acceleration.

**Arm speed ×** is the physical command modifier. An individual script command includes its base speed:

```python
# Start at a separately checked A pose.
robot.joint(1, 15, 5)
robot.joint(3, -20, 5)
robot.wait(1)
```

`robot.joint(joint_id, final_degrees, speed)` maps to the SDK's `send_angle`. It allows larger final-angle changes than the legacy vector-step method. The GUI still accepts `robot.move([six angles], speed)` and `robot.wait(seconds)`. Vector moves are restricted to 10° per step from measured feedback. Speeds are integer 1–10, waits 0–30 seconds. The editor accepts literal calls only; imports, loops, variables and arbitrary execution are unsupported. Plans are workbench `.py` files, not standalone scripts.

For standard Python control:

```python
from threading import Event
from cobot_studio.hardware import Controller
from cobot_studio.core import JointMove

arm = Controller()
try:
    a = arm.connect('/dev/cu.usbserial-YOUR_DEVICE', 115200)
    b = list(a)
    b[0] += 2
    # Run only after physically checking this specific movement.
    report = arm.run([JointMove(1, b[0], 5)], Event(),
                     required_start=a, expected_goal=b, speed_modifier=0.5)
    print(report)
finally:
    arm.close()
```

Python and training run on the Mac. Commands stream over USB; no Python code or policy is installed into the ESP32. Keep the Mac connected. Software STOP depends on USB and firmware; keep the physical emergency stop ready.

## Model and policy files

See [JOINT_MAP.md](docs/JOINT_MAP.md) for manufacturer sources and frame details. Distances are in millimetres and angles in SDK degrees. No vendor meshes or firmware are bundled.

New policies use `cobot-sequential-v2`, saving the 2022 model identifier, A/B angles, joint order, total iterations and reward. Legacy gain policies can be loaded for reference but must be retrained before using the new route workflow. JSON is numerically validated; no pickle or executable policy files are loaded. No PPO/SAC import or physics training is implemented. The optimizer is a small reward-driven random-search baseline over joint orders, not a neural policy.

## Verify

```sh
python -m unittest discover -s tests -v
PYTHONPATH=. python tests/smoke_ui.py
```

Tests use fake hardware and do not certify real operation. Independent project using the official `pymycobot` API; dependencies retain their own licenses. Original interface and rendering code; no formal safety certification or affiliation with Elephant Robotics.
