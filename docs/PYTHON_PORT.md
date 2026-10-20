# Python port handoff

## Controller contract from Stage 6

Input measured_pose is [x, y, z, yaw]. Detection inputs are detected, error_x, error_y, with pixel errors defined as pad center minus image center. The embedded controller assumes 640 x 480 images, center (320, 240).

The source labels its mapping calibration-confirmed: positive error_x requests backward motion; positive error_y requests left motion. This differs from the old standalone alignToPadController.m and descendController.m. Preserve the Stage 6 mapping for parity, then verify against the actual camera mounting.

Deadband is 40 pixels on each axis (inclusive). Normalize forward by -error_x/320 and right by -error_y/240, clamp to [-1,1], then smooth with alpha=0.25. Each axis inside deadband requests zero independently. State 0 is no detection, 1 recentering, 2 centered, 3 centered for at least 10 consecutive updates. No-detection resets the counter and applies smoothing followed by an additional 0.7 decay; residual lateral offsets therefore remain temporarily.

Position offsets are 0.05 times the smoothed normalized commands. Rotate body offsets using yaw: dx=cos(yaw)*forward-sin(yaw)*right; dy=sin(yaw)*forward+cos(yaw)*right. Add these to measured x/y. Preserve z and yaw. Outputs forward_cmd/right_cmd are normalized smoothed values, not velocity commands. The dormant descent path subtracts 0.02 m per invocation, but enable_descent=0. State 3 does not establish touchdown.

The earlier recenter-only logic instead scales lateral output by 0.20 and emits a positive 0.05 descent request after centering. Do not mix these output semantics with pose offsets.

## Required external inputs absent from this snapshot

- Landing-pad-trained checkpoint and class mapping, training configuration, and evaluation results; the supplied yolo11n.pt is unverified.
- Live YOLO inference/camera sender and its exact packet format, timing, confidence threshold, and frame timestamps.
- Training images, labels, synthetic metadata, unseen evaluation images, and test1.jpg used by the old camera stub.
- Site-specific Motive/OptiTrack calibration, rigid body identifiers, coordinate conventions, and lab network configuration.
- Installed MATLAB/Simulink, QUARC/Quanser libraries, DSP network blocks where used, and the vehicle-side controller required by the mission server.

The Stage 6 model contains a UDP stream endpoint localhost:5010 and mission-server stream arguments localhost:18001 and localhost:18002. These strings alone do not specify the wire protocol. Inspect actual model blocks and the missing sender before implementing sockets.

## Suggested implementation sequence

1. Implement a pure Python controller matching the extracted Stage 6 function, with explicit reset and deterministic tests for signs, smoothing, deadband, detection loss, yaw rotation, and the 10-update state transition.
2. Add recorded-frame YOLO inference using the confirmed landing-pad checkpoint and explicit class filtering. Keep timestamps and image dimensions in the detection contract.
3. Replay pose/detection logs through the controller; compare outputs against MATLAB before connecting live I/O.
4. Implement camera, localization, and Quanser command adapters once their actual interfaces are available. Preserve communication-loss and arming interlocks from the model.
5. Validate monitor-only behavior first. Design stale-data handling, tracking loss behavior, descent gating, altitude limits, and touchdown detection before implementing autonomous descent.

## Earlier prototype limitations

main_landing.m adds a functions folder that was not supplied as a directory; its helper files are retained together here. estimateDronePosition returns a constant [0,0,1], sendDroneCommand only prints, and getCameraFrame substitutes a blank frame when its missing test image cannot be found. ALIGN and DESCEND do not guard a lost detection before using padCenter, which can contain NaNs. The prototype cannot demonstrate a working closed-loop landing. config.m also overrides several fields later in the file; preserve effective values if using it as historical reference.
