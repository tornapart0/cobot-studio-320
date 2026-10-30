# myCobot 320 M5 2022 joint mapping

The uploaded image is a low-resolution dimension/working-range drawing. Its labels cannot be read accurately enough to establish six joint axes, sign conventions or exact dimensions. The user confirmed the **2022 M5** revision. The implementation therefore uses the manufacturer's matching joint-frame definition, not guessed image measurements.

## Sources

- [2022 M5 robot description](https://github.com/elephantrobotics/mycobot_ros/blob/noetic/mycobot_description/urdf/mycobot_320_m5_2022/new_mycobot_pro_320_m5_2022.urdf)
- [Matching manufacturer slider-to-hardware bridge](https://github.com/elephantrobotics/mycobot_ros/blob/noetic/mycobot_320/new_mycobot_320/scripts/mycobot_320_slider.py)
- [Official Python SDK](https://github.com/elephantrobotics/pymycobot)

The manufacturer bridge converts the ordered ROS joint values from radians to degrees and sends them directly to `send_angles`. This supports direct J1–J6 order without sign inversion. The viewer performs the inverse unit conversion for its local-axis rotations. Positive angles follow the right-hand rule about each joint's transformed local +Z axis, rather than a screen-dependent clockwise/counterclockwise rule.

## Frame map

Offsets below are in the **parent joint frame** in millimetres. Origin RPY values are radians. They must be transformed cumulatively; adding these offsets directly in world coordinates is incorrect. After each origin transform, joint rotation occurs around that frame's local +Z.

| SDK joint / slider | Parent-frame origin XYZ (mm) | Origin RPY (rad) |
|---|---|---|
| J1 | 0, 0, 173.9 | 0, 0, 0 |
| J2 | 0, -88.78, 0 | 0, -1.5708, 1.5708 |
| J3 | 135, 0, -88.78 | 0, 0, 0 |
| J4 | 120, 0, 88.78 | 0, 0, 1.57080 |
| J5 | 0, -95, 0 | 1.5708, 0, 0 |
| J6 | 0, 65.5, 0 | -1.5708, 0, 0 |

Base coordinates: +Z upward; ground is Z=0. At all-zero angles, the joint heights are approximately 173.9, 173.9, 308.9, 428.9, 523.9, 523.9 mm. J2 has a -88.78 mm Y offset. The flange origin is approximately (0, -154.28, 523.9) mm. These are joint frame locations, not external case dimensions or tool-tip reach. No extra gripper/tool length is assumed. The all-zero pose is not artificially straightened.

J1 at +90° rotates the J2 offset from negative Y to positive X. The viewer shows joint-axis arrows and flange orientation. Camera orbit changes only the view and never changes SDK signs, model dimensions or limits. SDK J6 rotation changes flange orientation around its axis, even where flange-origin position is unchanged.

## Preview and hardware consistency

`kinematics.py` is the single forward-kinematics model for rendering and comparing measured versus desired flange origins. Hardware commands use SDK degrees directly. The route trainer produces `JointMove` commands; both preview and hardware use the same `next_pose` interpretation and joint IDs. Interpolation is display-only. Generated route scripts contain no more than six `robot.joint` commands; unchanged joints are omitted.

## What is and is not verified

Automated checks cover frame offsets, rigid chain distances, J1 direction, preservation of upstream origins, sequential command IDs, script round trips and measured-feedback behavior using a fake adapter. No physical arm was connected for verification.

Physical acceptance still requires comparing zero and small positive/negative movements for each joint against the actual 2022 unit, using a clear workspace and the vendor's physical stop. If the firmware zero, mounting, mechanical calibration or tooling differs, do not infer a correction by flipping slider signs. Resolve that difference against the unit's manufacturer instructions first.

Measured matching means all six encoder-reported angles are within 1° of B and the model-derived flange-origin difference is within 5 mm. The latter is calculated from joint feedback using this same model; it is **not an independent physical position measurement**. It does not verify tooling, backlash, torque, force or collisions. Exact Cartesian agreement would require independent measurement and calibration.
