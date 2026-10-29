# myCobot 320 M5 research and application design

Reviewed 28 October 2026. Sources below are the manufacturer's documentation and source repositories. Hardware revisions and firmware must be matched before operation.

## Hardware baseline

The 320 M5 is a six-axis desktop arm. The manufacturer's specification page lists a 350 mm working radius, 1 kg payload and ±0.5 mm positioning accuracy, with Mac/Python development support. It is an ESP32-based M5 controller, distinct from the Raspberry Pi model. Payload, tooling and mounting affect practical operation; the application does not calculate these limits.

Sources:
- [Manufacturer M5 specifications](https://www.elephantrobotics.com/en/mycobot-320-m5-specifications-new-en/)
- [M5 manual: functional parameters](https://docs.elephantrobotics.com/docs/mycobot_320_m5_en/2-ProductFeature/2.1_320_M5_product/2.1.1-MachineSpecification.html)
- [SDK development documentation](https://docs.elephantrobotics.com/docs/mycobot_320_m5_en/6-SDKDevelopment/)

## Differences that affect setup

The marketing specification and manual disagree on power/current, temperature and joint-range details. Use the supplied unit's label and revision-specific manual for electrical setup; do not derive a replacement power supply from these notes. The public SDK has tighter J2/J3/J4 ranges than the marketing page. This application uses the intersection: ±165, ±135, ±145, ±145, ±165, ±175 degrees. These are validation bounds, not a guaranteed collision-free workspace.

## Mac → USB → controller

The official `MyCobot320` serial class exposes joint reads and commands. Its reviewed constructor defaults to 115200 baud and supports a serial timeout. Firmware and physical revision may require a different setting. The application offers 115200 and 1000000, requiring the operator to select the actual firmware setting. USB serial adapter drivers may be necessary depending on the unit; use manufacturer guidance and trusted driver sources. macOS devices normally appear under `/dev/cu.*`.

The Mac remains the host. A Python policy is not installed as an ESP32 executable. Firmware updates belong in the vendor tooling, outside this application's scope. The app never auto-powers the robot, releases servo torque, calibrates joints or modifies firmware.

Sources:
- [Official Python SDK](https://github.com/elephantrobotics/pymycobot)
- [MyCobot320 serial implementation](https://github.com/elephantrobotics/pymycobot/blob/main/pymycobot/mycobot320.py)
- [Joint limits](https://github.com/elephantrobotics/pymycobot/blob/main/pymycobot/robot_info.py)
- [Shared command methods including stop](https://github.com/elephantrobotics/pymycobot/blob/main/pymycobot/generate.py)

## Simulation and RL choices

Vendor ROS repositories are useful future integration points for visualization and robot descriptions. They are not evidence that a policy trained on an approximate model will transfer safely to this hardware. ROS on the Mac also adds setup complexity, so this first application uses Tk and a small pure-Python joint learning model.

- [Official ROS 1 repository](https://github.com/elephantrobotics/mycobot_ros)
- [Official ROS 2 repository](https://github.com/elephantrobotics/mycobot_ros2)

A production physics simulation would need exact revision geometry, inertial properties, actuator behavior, tool mass, calibration, collision geometry and measured transport latency. Task-dependent sensors and control frequency would need to be designed explicitly. No claim is made that those data have been collected here.

## Application structure

- `core.py`: immutable motion commands, strict literal-call parser, numeric validation and joint preview.
- `rl.py`: original synthetic learning task, reward-driven random search, validated JSON policy and trajectory generation.
- `hardware.py`: vendor SDK adapter, feedback checks, bounded moves and cancellation.
- `viewer.py`: original perspective-rendered 3D schematic with orbit, zoom and six joint labels. Dimensions and axes are illustrative.
- `app.py`: local desktop editor, joint sliders, connection controls, animated previews and training workflow; workers communicate to Tk through a queue.

Simulation never opens a serial connection. Hardware motion requires a deliberate connection, workspace acknowledgment and run confirmation. Errors abort the remainder of a plan. A software stop is a convenience, not a safety-rated emergency stop. There is no collision checking, force control, gripper integration, Cartesian planner or independent watchdog.

## First physical acceptance test (operator required)

1. Identify the exact 320 M5 revision, serial firmware and baud rate.
2. Follow the vendor mounting, power and stop instructions; remove payload and tooling for initial testing as appropriate to the manual.
3. Confirm the physical stop works before running the application.
4. Connect and compare displayed feedback with the actual pose. No movement should occur on connect.
5. Create a 1–2° move from the measured pose in a clear direction, preview, then run at speed 1–5.
6. Verify cancellation and feedback failure handling in controlled conditions using the physical stop as needed.
7. Expand the validated workspace and task only after these observations. A 10° step limit does not establish safe motion.

## Originality and dependencies

No product UI, meshes, logos, firmware, training datasets or vendor source files were copied into the deliverable. Links identify upstream documentation and dependencies. This is an independent application built around the public SDK API. No legal clearance or formal safety certification is implied.
