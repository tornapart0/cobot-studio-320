# Changes

## v0.3.0-beta.1

- Replaced the illustrative viewer model with manufacturer 2022 M5 joint-frame dimensions and local axes.
- Kept SDK J1–J6 direction/order consistent across model, sliders and single-joint commands.
- Train an order of at most six final joint moves; simulation frames are no longer streamed as route commands.
- Added continued training batches, hardware speed modifier and measured-B verification.
- Updated the installed Dock launcher app. New model and execution changes await physical validation, regardless of earlier release validation status.


## v0.2.0-beta.3

- Capture start A and end B with sliders, train on that exact route, simulate and push it.
- Save/load policies with endpoints; endpoint edits invalidate old training.
- Separate confirmed move to A, and fresh measured-start check before route execution.

## Viewer improvement

- Solid green ground plane with a visible border and subtle grid for clearer 3D perspective.

## v0.2.0-beta.2

- Green and black theme across the 3D view, controls, script editor and log.
- Retains system default fonts and the existing motion workflow.

## v0.2.0-beta.1

- Simpler green and white interface using standard controls and system fonts.
- Interactive 3D arm schematic with orbit, zoom and six labeled joints.
- Six sliders for adjusting the preview and setting training targets.
- Train, simulate and push workflows alongside the motion script editor.
- Target moves split into small intermediate steps, with measured-feedback checks.

Beta limitations: geometry is illustrative; simulation has no collision or dynamics model. Real-arm validation is required. macOS launch requires Python 3.11+ with Tk support. The download contains source and a launcher, not a bundled standalone Mac application.
