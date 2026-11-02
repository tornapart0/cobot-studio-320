# Wrist-camera mirror mode

## Backend choice

OpenCap now offers a Monocular beta for single-camera movement recording. Its published capture/processing workflow does not provide the local, live wrist-webcam hand-landmark interface used here. The original OpenCap core pipeline performs calibrated multi-camera reconstruction. This feature therefore uses **MediaPipe Hand Landmarker**, not OpenCap, for a local, live hand-gesture approximation. No OpenCap cloud upload, account or API is used.

Sources checked 2 November 2026:

- [OpenCap Monocular announcement](https://www.opencap.ai/news)
- [OpenCap processing pipeline](https://github.com/opencap-org/opencap-core/blob/main/main.py)
- [MediaPipe Python hand tracking](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker/python)
- [Google-hosted hand landmark model](https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task)

This is a gesture-to-joint mapper, not a validated anatomical retargeter. The robot has six revolute arm joints, no human fingers, and no gripper mapping is added. A single wrist-mounted camera does not reliably recover metric hand translation, shoulder/elbow motion or all six independent human degrees of freedom. Wrist movement changes its own camera image; there is no calibrated camera-to-robot hand-eye transform or world-frame stabilization in this version. Keep the person in front of the camera and start with the J4 wave mapping alone.

## Setup and use

1. Open **Mirror**. Select **Install camera support** if support is not already installed. It installs a separate Python 3.11–3.13 runtime environment with MediaPipe/OpenCV and downloads Google's model. The main Tk app can keep its existing Python 3.14 runtime. A suitable base Python must be installed.
2. Choose the external webcam's camera number (0–10). Numbering depends on macOS; 0 can be the built-in camera. Select the camera mount's 0/90/180/270-degree rotation, then **Start**. Video is captured only after Start. If macOS asks, allow Camera access for the Python runtime. Use the live view to confirm the correct physical camera; stop and try another number if needed. Rotation changes apply on restart.
3. Put the robot in a physically checked pose and show one clear palm. **Calibrate neutral hand** establishes the gesture reference. Camera pixels are not horizontally flipped. Use the preview to check each direction. **Invert** changes a joint's sign; sensitivity is 0.2×–2×. Recalibrate after changing these mapping settings. Preview maps all six joints around the last measured robot pose (zero before connection).
4. To control hardware, first use Connect/read and the workspace acknowledgment. Select which physical joints to drive. **Only J4 is selected initially.** J6 is the topmost joint, so J4 is the third joint down from the top, as requested for waving. Enable other joints only after checking their preview and the resulting physical path.
5. Select **Enable arm mirror**, review the physical-movement confirmation, then **hold** the mirror button. Real mirroring anchors to fresh measured joint feedback when the held session starts. Release the button or move the pointer off it to stop. Escape, STOP, focus loss, camera Stop, tracking loss and invalid feedback also stop/disarm it. Each session is capped at 30 seconds and requires enabling again.
6. Hand tracking is lost after 350 ms without a fresh valid observation, or when the tracked handedness changes. Reacquisition does not automatically move the robot; calibrate and enable again. One handedness is tracked, not a verified person's identity. Detection thresholds and handedness confidence filtering reduce uncertainty but do not establish tracking accuracy.

Live video and hand coordinates stay on this Mac. They are not recorded or uploaded. Stop releases the camera. Only model/package installation needs network access. Third-party model files and camera environments are not committed to GitHub.

## Default mapping

| Robot joint | Human camera-relative signal | Interpretation |
|---|---|---|
| J1 | Palm left/right | Screen-position gesture, not world X |
| J2 | Palm up/down | Screen-position gesture, not shoulder flexion |
| J3 | Apparent palm width | Near/far proxy, not measured depth |
| J4 | Palm roll / side-to-side waving | Requested third joint from the top |
| J5 | Estimated palm-normal tilt | Approximate palm pitch |
| J6 | Estimated palm-normal turn | Approximate palm yaw |

The six signals are coupled and depend on viewpoint. No inverse kinematics or imitation learning is claimed. Calibration is neutral subtraction; no ML model is trained from your video. The existing A → B trainer is separate.

## Physical bounds and ownership

Only the hardware worker owns serial access during a held session; other arm tasks cannot run concurrently. Targets are low-pass filtered, have a deadband and a 3°/s target slew limit, and remain within ±10° of the starting measured pose and the configured joint limits. Physical SDK speed is capped to 3%, independently of the ordinary Arm speed modifier. Only one joint command is outstanding at a time, commands are limited to 0.5° from measured pose, and the worker waits for its feedback before sending another. Feedback not following within two seconds, a 12° envelope violation or movement of an inactive joint beyond 2° aborts the session. These are software bounds, not collision prevention or a safety-rated watchdog.

Wrist-mounted cameras and their cables change load and clearance. The physical emergency stop remains necessary: software STOP depends on serial communication and firmware response. Nothing moves on app launch, camera start, model installation or preview calibration.

## Validation status

Automated tests exercise mapping, J4 isolation, angle wrapping, sign inversion, slew/travel bounds, stale/lost tracking, selected-joint control and held-control release with fake hardware. The desktop UI is smoke-tested. No physical hand-to-robot mirroring has been validated in this iteration. A MediaPipe runtime/model check is separate from camera and hardware acceptance.
