# QDrone Autonomous Landing — Milestone 1

This package implements only the first coding milestone from the supplied roadmap:

> One QDrone, direct OptiTrack localization, retained FrSky safety triggers, vertical takeoff from a captured random starting pose, and hold at one fixed desired pose.

Grid search, YOLO, centering, staged descent, and custom mission-state logic are intentionally not included.

## Configuration

- QDrone target: `192.168.2.13`
- Host address currently inherited from the Quanser Commander: `192.168.2.11`
- Motive/OptiTrack streaming ID: `1`
- Test altitude: `0.70 m`
- Position units: metres
- Angles: radians

The bundled calibration and rigid-body files are dated 2026-08-11. The observed calibration mean error was 0.441 mm.

## Create and validate the models

1. Start MATLAB R2021b with QUARC and the Quanser libraries available.
2. Add this package's `matlab` folder to the MATLAB path.
3. Run `build_milestone1(false)`.
4. Confirm that both models update without dimension or sample-time errors.
5. Only after a successful update, run `build_milestone1(true)` to invoke the configured QUARC builds.

The generated models are:

- `simulink/Mission_Server_QDrone_Autonomous.mdl`
- `simulink/QDrone_Autonomous_Commander.mdl`

The build script starts from untouched copies under `simulink/source_originals` every time.
The two files at the generated-model paths are initial source copies until
`build_milestone1(false)` completes successfully. Do not deploy or run them
before that command reports that both models updated successfully.

## Required signal packet

The existing Quanser Commander receives 14 values. The first 13 are the command payload:

1. `isTracking`
2. Arm trigger
3. Takeoff/Land trigger
4. Emergency-abort trigger
5. Joystick-issue signal
6–9. Measured `[x,y,z,yaw]`
10–13. Desired `[xd,yd,zd,yawd]`
14. Loopback timestamp

The build does not change this ordering.

## Bench test gate (motors/propellers disabled)

- Verify the OptiTrack block loads the bundled `.cal` and `.tra` files.
- Verify only rigid-body ID 1 is selected.
- Verify `isTracking == 1` while all five markers are visible.
- Carry the drone by hand and confirm corrected Research Studio axes and metres.
- Confirm desired X/Y/yaw capture the current measured pose.
- Confirm desired Z becomes exactly `0.70` only through the retained home-pose/trigger logic.
- Confirm the arm, takeoff/land, and emergency-stop trigger channels match the original Quanser model.
- Confirm communication-loss behavior by disconnecting the target with motors disabled.

## Tethered/flight test gate

- Use the original manual-flight models first and confirm arm, hover, land, and emergency behavior.
- Start from a safe central location with a clear volume above it.
- Keep an operator on the FrSky emergency controls.
- Arm manually; software must not auto-arm.
- Request takeoff and verify vertical motion only.
- Verify bounded hold at 0.70 m and no unexpected X/Y displacement.
- Use the retained Quanser LAND trigger to land.

Do not proceed to waypoints, grid search, or YOLO until this gate is passed repeatedly.

## Current limitation

The corrected Research Studio geofence bounds remain pending. The raw Motive screenshots are not used directly because Motive Y is vertical and the Quanser convention-correction subsystem changes the flight-frame axes. Horizontal autonomous motion remains disabled.
