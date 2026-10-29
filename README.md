# Cobot Studio 320 · Beta

An original local desktop workbench for **Elephant Robotics myCobot 320 M5**, designed for a Mac connected by USB. Write short Python-style motion commands, preview joint trajectories, train a small reinforcement learning policy, and explicitly run reviewed plans on the arm.

**Status: v0.2.0-beta.3 — working prototype, tested with simulated hardware. Real-arm validation is still required.** No robot was connected during development. No pretrained task policy is included.

## Start on your Mac

Install Python 3.11 or newer with Tk support (the python.org macOS installer includes Tk). Open Terminal in this folder:

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
cobot-studio
```

After cloning/downloading, you can also double-click `Launch.command`. If macOS refuses to open the downloaded launcher, use the Terminal commands above. The first launch installs the Python dependencies into a local virtual environment. Nothing is uploaded to a cloud service.

## Interface

The interface uses a simple green and black palette, standard controls and system default fonts.

The left side has an interactive 3D arm schematic and six labeled joint sliders. Drag the view to orbit; scroll to zoom. Joint colors match the on-arm labels and angle legend. The tool orientation marker shows wrist rotation. This is original illustrative geometry, **not calibrated forward kinematics or collision detection**.

The all-zero pose is fully upright in the schematic. This display convention does not redefine the real arm’s joint zero or calibration.

Sliders affect only the preview. **Add to script** records the target, while **Push target** creates 5° intermediate waypoints from the last measured pose and asks before sending. The hardware adapter still checks live feedback before every move.

The right side has three tabs:

- **Train & simulate**: adjust sliders and select **Set start A**, then adjust again and select **Set end B**. **Train this route** learns on these exact poses. **Simulate A → B** animates from A and places the route in the script editor. **Move arm to start A** is a separate, confirmed movement; **Push trained A → B** reads live feedback and requires the real arm to be within 2° per joint of A. Changing a saved endpoint invalidates the previous policy. Show A/B restores either saved pose to the sliders. Saving a policy includes both endpoints; loading it restores them.
- **Script**: edit simple Python commands, animate them in the arm viewer, save/open scripts, and send a reviewed script.
- **Connect**: USB port, firmware baud rate, connect/read, and disconnect.

**Show measured pose** restores the last hardware reading (zero before connection). During simulation the 3D view animates while the sliders retain your target. Simulation is a joint-path preview without physical timing or wait playback. STOP cancels both animation and remaining hardware commands.

## Workflow

1. Start disconnected. Open `examples/small_move.py` or write a plan.
2. Click **Simulate** in the Script tab to validate and animate all six joint angles. The preview begins at zero while disconnected, or the last measured pose while connected.
3. In **Train & simulate**, capture A and B from the sliders, select **Train this route**, then **Simulate A → B**. Save the policy and plan if desired.
4. Mount and power your M5 according to its manual. Connect USB, select its `/dev/cu.*` device, and choose the baud rate matching its firmware. The default here is 115200; do not assume that it matches every revision.
5. **Connect / read** reads joint feedback without issuing a move or enabling power. It fails closed if feedback is invalid. Install the correct vendor serial-control firmware using myStudio if needed; this app does not flash firmware.
6. Review the measured starting pose and the complete trajectory. Clear the workspace and keep the physical emergency stop accessible. Check the acknowledgment and select **Push script to arm**. A confirmation precedes motion.
7. **STOP / cancel** cancels remaining commands. The worker requests a software stop after its current serial operation. A blocked/disconnected USB link can delay or prevent that request: use the physical stop when needed.

Python runs on the Mac and streams joint commands through `pymycobot.MyCobot320`. “Push script to arm” does **not** upload Python or RL weights into the ESP32. Keep the Mac and USB connected throughout execution.

## Simple commands

```python
robot.move([0, 0, 0, 0, 0, 0], 5)
robot.move([5, 0, 0, 0, 0, 0], 5)
robot.wait(1)
```

The editor accepts only literal `robot.move([j1, j2, j3, j4, j5, j6], speed)` and `robot.wait(seconds)` calls. Angles are degrees, speed is an SDK percentage capped to **1–10** here. Waits are 0–30 seconds. Imports, variables, loops and arbitrary Python execution are intentionally unsupported in the GUI. Saved `.py` files are plans for this workbench; they are not standalone scripts.

For normal Python, use the same validated controller directly:

```python
from threading import Event
from cobot_studio.hardware import Controller
from cobot_studio.core import Move

arm = Controller()
try:
    measured = arm.connect('/dev/cu.usbserial-YOUR_DEVICE', 115200)
    # Only after physically checking this specific movement:
    target = list(measured)
    target[0] += 2
    arm.run([Move(target, 5)], Event())
finally:
    arm.close()
```

The hardware adapter reads feedback before each move, rejects steps over 10°, waits for three feedback samples within 1° of target, and stops on invalid feedback or a 20-second arrival timeout. The 1° arrival threshold is software tolerance, not a specification of Cartesian accuracy. Do not treat the speed cap or joint limits as collision protection.

## What the RL feature does

This is a small, reproducible reward-driven random-search baseline. The GUI optimizes six proportional feedback gains for your captured A → B route. The Python training API also supports a generic baseline trained on eight generated tasks when no endpoints are supplied. Observations are joint error, actions are increments capped to ±2° per step, and reward penalizes squared joint error and action magnitude. `rollout()` is a deterministic joint-space integrator. It has **no gravity, torque, link geometry, contact, payload, latency or collision model**.

The trained policy generates a bounded, finite joint trajectory which you can inspect and run through the same USB adapter. On hardware, the generated waypoints are replayed with measured-arrival checks; the simulated timing is not preserved. This is an educational sim-to-hardware workflow, **not validated physical sim-to-real transfer**. Save/load uses plain JSON with schema and numeric validation, not pickle or executable model files.

For manipulation, contact tasks or external PPO/SAC policies, first add a calibrated MuJoCo or PyBullet model using the exact arm revision and licensed model assets, then validate sensors, timing, limits and collision handling. External neural policy import and physics simulation are not implemented in this version.

## Research and design

See [the research notes](docs/RESEARCH.md) for official sources, revision differences, setup requirements and implementation scope. The interface, code and simplified learning model were created for this project; no vendor meshes, logos, firmware or third-party product designs are bundled. `pymycobot` and `pyserial` remain separately installed third-party dependencies under their own licenses. This project is not affiliated with Elephant Robotics.

## Verify

```sh
python -m unittest discover -s tests -v
```

Tests cover input validation, non-executing plan parsing, policy persistence and training, feedback failures, cancellation, oversized moves and arrival timeout through a fake serial adapter. These checks do not certify physical operation.
