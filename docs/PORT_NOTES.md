# Port notes: assumptions and deviations

## What is a port and what is an addition

| Python | Source | Verified against |
|---|---|---|
| `controllers.YOLOPoseCommandGenerator` | chart_51 (Stage 6) | Octave, bit-exact (0 ulp), 9 900 steps + 985 steps with `enable_descent = 1` |
| `controllers.LandingPadRecenterLogic` | chart_12 (Recenter tests) | Octave, bit-exact |
| `controllers.legacy` (`align_to_pad`, `descend`, `waypoint`, `generate_grid_waypoints`, `legacy_config`) | `reference/matlab/legacy/*.m` | Octave, bit-exact |
| `blocks.extract_object_data`, `blocks.detect_stream_connection_loss` | chart_13/23, chart_33/42 | Octave, bit-exact incl. MATLAB index errors |
| `blocks.QuarcThreshold`, `StreamCommMonitor`, `InverseModulus`, `MeasurementConditioning`, `motive_to_rsf_*`, `stage6_desired_pose`, `JoystickInterlock` | Simulink/QUARC blocks | **Not oracle-verified**: built from the block parameters and QUARC documentation; unit tests only |
| `io.CommandGate` | none (Python addition) | unit tests |
| `io.UdpDetectionReceiver/Sender`, `replay`, `scenario`, `detection.*` | none | unit tests |

## MATLAB semantics reproduced on purpose

* `min`/`max` ignore NaN, so a NaN pixel error clamps to −1 (full command).
* `detected == 1` and `detected == 0` are separate tests. Any other value
  (2, 0.5, NaN, −1) gives state 0 and resets the counter **without** the
  0.7 decay.
* The centred counter grows without bound.
* The deadband is inclusive (`abs(e) <= 40`).
* State 3 in the recenter chart zeroes the lateral requests, but the counter
  can only reach 10 on a centred step where they are already 0, so those
  lines have no observable effect (no mutant is generated for them).
* Grid colon ranges: Octave computes `base + k*inc` and clamps the last
  element to the limit. The fixtures are from Octave; MATLAB's colon uses a
  different algorithm and may differ in the last ulp. Not checked against
  MATLAB.

## Assumptions in the block equivalents (not oracle-verified)

* QUARC Threshold: output goes high once the condition has held for
  **more than** the duration. The docs do not settle the boundary.
* Stream Server "Receive most recent data": the newest complete datagram is
  used; a partial one is rejected. Before the first packet the default
  `[0 0 0]` is output.
* Inverse Modulus unwraps when consecutive samples jump by more than half the
  modulus.

## Python additions that change behaviour (deliberately)

`CommandGate` sits between the controller and any command sink. It is
`monitor_only=True` by default, and refuses when: not armed, takeoff toggle
off, emergency abort, joystick issue, QDrone comm issue, OptiTrack not
tracking, non-finite pose, any descent request, a horizontal step above
√2·0.05 m, a yaw change, or a detection older than 0.2 s. Stage 6 itself
enforces none of these on the host (findings F2, F3 in
[SIMULINK_SPEC.md](SIMULINK_SPEC.md)).

## Not ported

* "Prevent Gimbal Lock" Euler conversion (conventions unverified).
* Legacy `detectLandingPad.m` (Image Processing Toolbox) and the
  `main_landing.m` state machine.
* QDrone/QBot packet byte encoder and a live command sink (types unknown).
* OptiTrack/NatNet adapter and a live camera source (interfaces only, with
  loopback implementations).

## Quanser Python API

`quanser.communications.Stream` (Quanser SDK / QUARC Python API) wraps the
same Stream API the QUARC blocks use and supports `udp://` URIs, so
`io.quanser_stream.QuanserStreamDetectionReceiver` implements the 5010
receive against it. It is not on PyPI; it ships with the Quanser SDK
(Windows `%QSDK_DIR%python`, Linux `python3-quanser-apis`) and is
preinstalled on the QDrone 2. It was **not available here, so that adapter
is untested**; the plain-socket `UdpDetectionReceiver` is tested.
No Quanser Python API for OptiTrack was found (the QUARC block uses the
Windows Motive API; NatNet would be the Python route).
