# Finger-count whole-arm control

## Mapping

| Clear hand gesture | Selected robot joint |
|---|---|
| Closed fist (0 fingers) | J1 — base, first joint up from the base |
| 1 finger extended | J2 |
| 2 fingers extended | J3 |
| 3 fingers extended | J4 |
| 4 fingers extended | J5 |
| 5 fingers extended | J6 — topmost joint |

Show a steady count for about **0.3 seconds** to select a new joint, then twist/roll your hand clockwise or counterclockwise in the camera view to turn that joint. The visible count includes the thumb. You can use any combination of fingers with the same count. There is one mapping mode; there is no separate individual-fingers mode.

Finger count selects the joint rather than ordering it to jump to a fixed angle. All selections share the calibrated straight-hand direction. Straight requests 0°, 90° left requests the positive software joint limit, and 90° right requests the negative limit. Angles between those poses interpolate linearly; larger tilts saturate at the limit. Selection changes move smoothly toward the absolute requested angle, without resetting the reference. Other joints retain their positions. An unclear/partially curled count freezes the target instead of guessing. Use the **Invert** option to reverse an individual joint's rotation direction, then recalibrate. Preview and hardware share mapping state, selected joints and the original calibrated robot reference.

## Workflow

1. Reopen the updated app from **Cobot Sim & Push** in the Dock. Old running camera processes must be stopped/reopened so they provide finger data.
2. For simulation, no USB connection is needed. Before physical control, connect/read the powered 2022 M5 so the displayed robot pose is current.
3. In Mirror, select the external webcam number and mount rotation, then Start camera. Confirm the correct camera in the live view. Camera indices depend on macOS; 0 or 1 may be the USB camera. If needed, Install camera support sets up the isolated Python 3.11–3.13 MediaPipe/OpenCV runtime and downloads Google's model. The main Tk UI can keep Python 3.14.
4. Show an upright hand like reference image 1 and **Set straight hand as 0°**. Try fist and counts 1–5 in preview. The recognized gesture and corresponding joint are displayed above the video. Select which joints to allow; all six are selected initially. Changing sensitivity, inversion or allowed joints requires recalibration before a new physical session.
5. Enable the robot-output switch, check the workspace acknowledgment, then **Start live robot mirroring** and confirm the movement prompt. The robot follows continuously; **no mouse button needs to be held**. The live status shows actual SDK command count, targets and measured feedback.
6. **Stop mirroring**, STOP, Escape or camera Stop ends the session. Tracking loss or a connection/feedback fault also stops it. There is no fixed 30-second limit. A brief tracking pause preserves calibration and resumes preview after reacquisition, but hardware requires explicit Start again. No physical movement starts merely by recognizing a gesture.

Changing a count never powers or starts the robot. Start requires USB, valid hand tracking, calibration and the workspace acknowledgment. Explicit motor-power/servo-disabled feedback blocks motion and reports the cause rather than automatically enabling motors.

## Camera-relative interpretation

MediaPipe estimates 21 hand landmarks. The app estimates each finger's straightness from its 3D landmark joint angles, with a palm-relative thumb check. It counts confidently extended versus curled fingers. Intermediate values are treated as ambiguous. Recognition is geometric and unvalidated on this particular user's hand; lighting, occlusion, perspective and a fully occluded thumb can affect the count. The display lets the user check it before real movement.

Roll is palm direction in the camera image relative to the straight-hand calibration (left positive, right negative). A wrist-mounted camera moves with the robot, so the image also changes due to robot motion. This is discrete joint selection plus relative hand rotation, not anatomical shoulder/elbow imitation or recovered metric human 6-DOF. There is no hand-eye/world-frame camera compensation. The wrist camera, fingers and robot are not claimed to have an exact anatomical correspondence.

## Continuous control bounds

One worker owns USB during live control. It uses the same calibrated mapping state as preview. Targets span the full software limits: J1 ±165°, J2 ±135°, J3 ±145°, J4 ±145°, J5 ±165°, J6 ±175°. Low-pass filtering and deadband remain. Preview slews at up to 60°/s; physical control slews at up to 3°/s. The former ±10° calibration envelope has been removed for this mapping. Physical SDK speed remains 3%. Only one joint command is outstanding at a time, and each command is at most 0.5° from measured pose. Feedback must follow before the next command. Stale hand observations beyond 350 ms, a handedness change, explicit Stop, camera failure, feedback lag, or invalid measured joint angles aborts movement.

These bounds and the physical emergency stop remain separate controls. Software stop depends on USB and firmware; there is no collision avoidance or safety-rated independent watchdog. The camera and cable alter load/clearance. This iteration's physical gesture control is not validated on the real arm.

## Backend and privacy

Live tracking is **MediaPipe**, not OpenCap. OpenCap's Monocular beta supports single-camera recording, but its published processing workflow is not the local live wrist-webcam hand interface used here. No OpenCap account/API/cloud upload is used. Video stays local and is not recorded. The optional runtime and third-party model are stored locally and excluded from GitHub.

Sources:

- [OpenCap Monocular announcement](https://www.opencap.ai/news)
- [OpenCap processing pipeline](https://github.com/opencap-org/opencap-core/blob/main/main.py)
- [MediaPipe Python hand tracking](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker/python)
- [Google-hosted hand model](https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task)

Tests cover each count-to-joint role, selection debounce, no-jump changes, direction inversion, tracking pauses, count ambiguity, physical stop/fault handling and preview/SDK state consistency through fake hardware. They do not establish camera-recognition accuracy or safe physical operation.


## Simulation switch and rotation calibration

Robot output defaults off. Start the camera and calibrate to control only the
3D preview, without connecting the arm. Check “Send hand movements to robot”
and explicitly Start to enable hardware. Turning the switch off cancels the
stream and preserves calibration; turning it on never starts motion by itself.

Choose Roll (camera-plane twist), Tilt or Turn (estimated 3D palm normal), then
recalibrate. Roll uses the wrist-to-MCP palm direction rather than the
knuckle span, so curling fingers does not redefine the angle. Camera aspect
ratio is accounted for. At sensitivity 1, ±90° of hand tilt maps to the
selected joint’s positive/negative software limit, and straight maps to zero. Camera movement can change the measured hand orientation;
this is camera-relative control, not world-referenced motion capture.

The earlier prosthetic gripper PlatformIO controller was reviewed. It controls
a pressure-sensing servo with filtered feedback, confirmed thresholds, and
timed steps; it has no camera rotation tracker. Existing measured-angle
feedback and timed movement bounds in this app serve the comparable role.
No prosthetic pressure thresholds or unrelated servo limits were copied.

## Weight and tipping warning

The manufacturer’s 2022 page lists 3 kg, its newer manual about 3.3 kg, and
its specification page 3.36 kg. Rated payload is 1 kg, including the wrist
camera, mount, tool and carried load. Secure the base. The app cannot infer
a reliable tipping margin from total mass: link centers of mass, actual
payload, base support footprint, cable forces and acceleration are unknown.
The warning is advisory, not a tipping prediction or protective interlock.

Sources:
- https://docs.elephantrobotics.com/docs/gitbook-en/2-serialproduct/2.2-320/2.2.1-M5.html
- https://www.elephantrobotics.com/en/mycobot-320-m5-specifications-new-en/

Physical rotation tracking and switching behavior still require validation
on the actual arm. Automated checks use simulated feedback only.
