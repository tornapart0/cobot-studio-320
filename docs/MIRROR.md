# Wrist-camera palm following

The only tracking camera is mounted rigidly on the robot wrist. The app uses
MediaPipe hand landmarks and a local eye-in-hand visual controller. There is no
external camera, wearable sensor, OpenCap service, or cloud video upload.

## What moves

- Move an open palm left/right/up/down: multiple robot joints work together to
  center the palm in the camera image.
- Rotate the palm: the wrist camera aligns the palm direction with the saved
  reference. This controls camera-axis roll, not full palm pitch/yaw imitation.
- Optionally enable distance following: a larger palm image requests retreat;
  a smaller palm requests approach, to restore the saved palm size.
- Finger count does not select joints in this interface.

This is target following relative to the camera. It does not copy the human
elbow/shoulder pose or reconstruct a measured absolute hand position.

## Setup

1. Stop the old session, quit, and reopen Cobot Sim & Push from the Dock.
2. In Mirror, choose the wrist camera number and displayed image rotation.
   Start camera and confirm the correct feed. Camera numbering is macOS dependent.
3. Enter the camera mounting roll, pitch, yaw and XYZ offset **relative to the
   robot's tool frame**, using the physical mount. Camera axes are image-right X,
   image-down Y, and lens-forward Z. Rotation is Rz(yaw) Ry(pitch) Rx(roll).
   Offsets are millimetres in the tool frame. Zero assumes aligned camera/tool
   axes and coincident origins; it is not an automatically measured calibration.
   The displayed image rotation is accounted for separately by the controller.
4. Enter approximate lens-to-palm distance (default 350 mm). The controller
   assumes a 60° vertical field of view for its gain estimate. These are nominal
   values; this release does not perform intrinsic/hand-eye camera calibration.
5. Show one clear open palm and set the palm size/rotation reference. The desired
   position is the image center, while desired size and roll are saved from this
   observation. Center your palm before setting the reference to avoid initial motion.
6. Test small hand movements with robot output off. The 3D arm is simulated only;
   camera use and calibration do not open the serial port.
7. Before physical following, connect/read the robot so sliders show its actual
   pose and set the reference again. Confirm the mount settings, clear workspace,
   enable robot output, and press Start live hand following. Begin with small
   motions and verify direction. This release has not been physically validated.
8. Stop, Escape, camera Stop, lost tracking, or a feedback fault ends physical
   following. Turning robot output off cancels the stream. Turning it on never
   starts movement by itself. Recovered tracking resumes preview, not hardware.

Settings changes require a new reference. Distance following defaults off:
hand tilt, partial occlusion, finger motion and perspective can change apparent
palm size without a true distance change. It is not a distance sensor.

## Controller

The measured palm centroid, log apparent width and palm roll are image features.
At each update an approximate palm point/direction is placed in the robot frame
using the current wrist-camera pose. The controller numerically differentiates
the image features against the six joint angles, including the camera mounting
transform. A damped least-squares solution reduces the image error with bounded
joint velocities. Damping bounds behavior near singularities; it does not imply
all targets are reachable or guarantee collision-free motion.

In live mode, each calculation uses fresh measured joint angles and a fresh
camera observation. Camera movement therefore participates in the feedback
loop rather than being interpreted as a new absolute hand-angle command.
Commands are small single-joint updates across the coordinated solution, not
simultaneous six-joint streaming. The USB/feedback cadence limits response.

In simulation, the physical camera is not moved by the virtual robot. Input is
anchored at the simulated camera's starting pose and reprojected through its
changing pose. A stationary hand consequently settles to an image alignment
rather than integrating an unchanged image error forever. Synthetic tests check
convergence; this approximate model is not a dynamics or collision simulation.

Preview joint velocities are bounded at 30°/s; live targets at 3°/s, SDK speed 3%.
Live targets accumulate by small bounded increments, with at most 0.5° lead over
measured positions. Only one SDK command is outstanding. Feedback must get within
0.08° of each following command within two seconds. Tracking older than 350 ms,
low confidence, hand switching, invalid joint feedback or explicit Stop aborts.
Actual servo feedback resolution/noise may require tuning that arrival threshold.
Software limits remain J1 ±165°, J2 ±135°, J3 ±145°, J4 ±145°, J5 ±165°, J6 ±175°.
After live control, preview restarts at the last measured robot pose.

No collision avoidance or safety-rated watchdog is implemented. The physical
stop remains independent of the app. Secure the base. Published arm weights
range from about 3 to 3.36 kg; rated payload is 1 kg including camera, mount,
cables and tools. Total mass alone cannot establish tipping stability.

## Research and privacy

The implementation follows the image-feedback and damped Jacobian principles
in [Chaumette and Hutchinson, Visual Servo Control Part I (2006)](https://web.mit.edu/amcp/OldFiles/drg/Chaumette_Part_I.pdf).
It is an original implementation adapted to this app, not a claim to reproduce
the paper's experimental results.

[MediaPipe landmarks](https://github.com/google-ai-edge/mediapipe/blob/master/docs/solutions/hands.md)
provide relative hand geometry, not an absolute robot-frame hand position.
Frames stay local and are not recorded. Runtime dependencies and the Google
model remain in `.mirror-venv` and `.mirror-data`, excluded from GitHub.

Validation uses synthetic hand observations and fake serial hardware, covering
image-error reduction, camera offsets/rotation, stationary input, optional depth,
joint limits, small commands, lost tracking and stalled feedback. Real camera
recognition accuracy, mount calibration and physical following remain unverified.
